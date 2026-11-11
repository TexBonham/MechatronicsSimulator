"""Live streaming of a running run (FAC-006, FAC-007): the driver's rows go into a facade-owned `Recorder`; its
subscription (REC-019, REC-020) feeds progress and decimated tick batches to a listener. The recording the runner
archives is the driver's own; this side only reads rows."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Protocol

from mechsim.facade.decimate import bucket_of, bucket_select
from mechsim.record.layout import Recording
from mechsim.record.stream import Recorder
from mechsim.record.tick import TickRecord
from mechsim.testing.driver import (
    BuildResult,
    EndSpec,
    RecordView,
    RunConfig,
    RunReport,
    ScheduledCommand,
    ScheduledFault,
    SimulationDriver,
)
from mechsim.testing.schema import Case, InitialState

CHUNK_ROWS = 2048


class LiveWiring(Protocol):
    """How the facade reaches live rows: a driver that hands each row to `on_row` as the kernel writes it, and the
    record view of a partial recording for the build being run."""

    def driver(self, on_row: Callable[[TickRecord], None]) -> SimulationDriver: ...

    def view(self, recording: Recording, build: BuildResult) -> RecordView: ...


@dataclass(frozen=True)
class Subscription:
    series: tuple[str, ...]
    t_start: float
    t_end: float
    points: int


@dataclass(frozen=True)
class Batch:
    sequence: int
    series: tuple[str, ...]
    times: tuple[float, ...]
    columns: tuple[tuple[float, ...], ...]


class RunListener(Protocol):
    def progress(self, t: float) -> None: ...

    def batch(self, batch: Batch) -> None: ...

    def failed(self, message: str) -> None: ...


@dataclass
class _Stream:
    subscription: Subscription
    next_bucket: int = 0
    sequence: int = 0
    dead: bool = False


@dataclass
class LiveDriver:
    """`SimulationDriver` around the wiring's driver: every call delegated, rows mirrored into its own recorder."""

    wiring: LiveWiring
    listener: RunListener
    subscriptions: Sequence[Subscription]
    _inner: SimulationDriver | None = None
    _build: BuildResult | None = None
    _recorder: Recorder | None = None
    _rows: int = 0
    _streams: list[_Stream] = field(default_factory=list)

    def __post_init__(self) -> None:
        self._inner = self.wiring.driver(self._on_row)
        self._streams = [_Stream(subscription) for subscription in self.subscriptions]

    @property
    def inner(self) -> SimulationDriver:
        if self._inner is None:
            raise RuntimeError("live driver has no inner driver")
        return self._inner

    def load(self, build: BuildResult, case: Case) -> None:
        self._build = build
        self.inner.load(build, case)

    def reported_controller_state(self) -> Mapping[str, bool | str]:
        return self.inner.reported_controller_state()

    def configure(self, config: RunConfig) -> None:
        self._recorder = Recorder(config.base_rate)
        self._recorder.subscribe(self._on_tick)
        self.inner.configure(config)

    def set_initial_state(self, state: InitialState) -> None:
        self.inner.set_initial_state(state)

    def schedule_command(self, command: ScheduledCommand) -> None:
        self.inner.schedule_command(command)

    def schedule_fault(self, fault: ScheduledFault) -> None:
        self.inner.schedule_fault(fault)

    def run(self, end: EndSpec) -> RunReport:
        report = self.inner.run(end)
        if self._recorder is not None and self._recorder.rows > 0:
            self._flush(final=True)
        return report

    def recording(self) -> RecordView:
        return self.inner.recording()

    def _on_row(self, row: TickRecord) -> None:
        if self._recorder is not None:
            self._recorder.write_row(row)

    def _on_tick(self, row: TickRecord) -> None:
        self._rows += 1
        if self._rows % CHUNK_ROWS == 0:
            self._flush(final=False)

    def _flush(self, final: bool) -> None:
        if self._recorder is None or self._build is None:
            raise RuntimeError("live driver flushed before load and configure")
        view = self.wiring.view(self._recorder.recording(), self._build)
        times = list(view.times())
        self.listener.progress(times[-1])
        for stream in self._streams:
            if not stream.dead:
                self._emit(stream, view, times, final)

    def _emit(self, stream: _Stream, view: RecordView, times: list[float], final: bool) -> None:
        """Buckets whose span the recording has passed (all of them at the end) go out once each, in order."""
        sub = stream.subscription
        stop = sub.points
        if not final and times[-1] < sub.t_end:
            stop = bucket_of(times[-1], sub.t_start, sub.t_end, sub.points)
        if stop <= stream.next_bucket:
            return
        kept = bucket_select(times, sub.t_start, sub.t_end, sub.points, stream.next_bucket, stop)
        stream.next_bucket = stop
        if len(kept) == 0:
            return
        columns: list[tuple[float, ...]] = []
        for name in sub.series:
            try:
                values = view.series(name).values
            except KeyError:
                stream.dead = True
                self.listener.failed(f"no series {name!r} in the recording")
                return
            columns.append(tuple(float(values[index]) for index in kept))
        self.listener.batch(Batch(stream.sequence, sub.series, tuple(times[index] for index in kept), tuple(columns)))
        stream.sequence += 1


def live_factory(
    wiring: LiveWiring, listener: RunListener, subscriptions: Sequence[Subscription]
) -> Callable[[], LiveDriver]:
    def make() -> LiveDriver:
        return LiveDriver(wiring, listener, tuple(subscriptions))

    return make


__all__ = ["Batch", "LiveDriver", "LiveWiring", "RunListener", "Subscription", "live_factory"]
