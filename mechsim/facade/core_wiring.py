"""Live-row wiring over the core driver (FAC-006, FAC-009): imported only when the core extension is built."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from mechsim.builder.result import BuildResult as CoreBuild
from mechsim.record.layout import Recording
from mechsim.record.tick import TickRecord
from mechsim.testing.core_driver import CoreDriver, DriverError, Kinematics, RecordingView, tick_record
from mechsim.testing.driver import BuildResult, RecordView, SimulationDriver


class CoreLiveWiring:
    """A `CoreDriver` whose recorder-sink subscriber (REC-019) hands each row on as a `TickRecord`."""

    def driver(self, on_row: Callable[[TickRecord], None]) -> SimulationDriver:
        def forward(row: Any) -> None:
            on_row(tick_record(row))

        return CoreDriver(subscribers=(forward,))

    def view(self, recording: Recording, build: BuildResult) -> RecordView:
        if not isinstance(build, CoreBuild) or build.physics is None:
            raise DriverError("live view needs a core build with physics")
        return RecordingView(recording, Kinematics.of(build.physics))
