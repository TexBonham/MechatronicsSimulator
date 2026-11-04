"""Run-twice determinism of the recorder (REC-022, REC-023; feeds INV-014).

Synthetic scope: two runs are stood in for by two passes of one seeded synthetic tick generator through the Python
recorder and archive writer. This verifies the recorder side only: identical rows give byte-identical archives and
a single changed value is found and named. Integration rewires the rows to the real kernel run twice per shipped
design and test case, which is what INV-014 itself asks; until then the INV-014 stub stays.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from mechsim.record import Recorder, archives_identical
from mechsim.record.tick import SeriesSample, TickRecord
from tests.unit.test_record_support import BASE_RATE, finished, identity, synthetic_rows

SEED = 20261104
TICKS = 400


def _run(path: Path, rows: list[TickRecord]) -> Path:
    recorder = Recorder(BASE_RATE)
    for row in rows:
        recorder.write_row(row)
    recorder.flush(path, identity(seed=SEED), finished(len(rows)))
    return path


@pytest.mark.req("REC-022")
def test_rec_022_same_inputs_give_identical_archives(tmp_path: Path) -> None:
    """Two archives from the same design, test case, seed and controller mode shall be identical in every
    recorded series and attribute; here identical to the byte."""
    first = _run(tmp_path / "first.h5", synthetic_rows(SEED, TICKS))
    second = _run(tmp_path / "second.h5", synthetic_rows(SEED, TICKS))
    report = archives_identical(first, second)
    print(f"run-twice: identical={report.identical} content_identical={report.content_identical}")
    assert report.identical
    assert report.content_identical
    assert report.first_difference is None


@pytest.mark.req("REC-022")
def test_rec_022_one_changed_value_is_detected(tmp_path: Path) -> None:
    rows = synthetic_rows(SEED, TICKS)
    first = _run(tmp_path / "first.h5", rows)
    changed = list(rows)
    old = changed[250]
    pins = (SeriesSample(old.pin_signals[0].name, old.pin_signals[0].unit, 1.0e9), *old.pin_signals[1:])
    changed[250] = TickRecord(
        old.time, old.physics, pins, old.rails, old.actuator_efforts, old.controller_state, old.events
    )
    second = _run(tmp_path / "second.h5", changed)
    report = archives_identical(first, second)
    assert not report.identical
    assert not report.content_identical
    assert report.first_difference == "dataset signals/pins/mcu.pa0: first difference at index (250,)"
