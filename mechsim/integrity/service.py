"""The runner's integrity hook (`testing.driver.IntegrityCheck`): configuration and materials from the design's
folder, inputs from the build and the recording, the report back (INT-022)."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

from mechsim.integrity.config import FILE_NAME, IntegrityConfig, load_config
from mechsim.integrity.inputs import run_inputs
from mechsim.integrity.plugin import MaterialView
from mechsim.integrity.table import CARRIED_NOTE, IntegrityReport, evaluate, window_note
from mechsim.materials.database import MaterialSet
from mechsim.testing.driver import RecordView
from mechsim.testing.schema import DESIGNS_ROOT, Case


class IntegrityService:
    """`(build, case, recording) -> IntegrityReport | None`. The configuration is `integrity.yaml` beside the design
    file, in `designs_root / <design name>` unless `design_dir` maps the design to its folder; without that file the
    run carries no table (None). `config` replaces the file. The case's `integrity.design_load_carried` (TST-043) sets
    the load factor to 1 with a line, `integrity.evaluate_from` (TST-044) cuts the inputs there with a line."""

    def __init__(
        self,
        designs_root: Path = DESIGNS_ROOT,
        config: IntegrityConfig | None = None,
        design_dir: Callable[[Any], Path] | None = None,
    ) -> None:
        self.designs_root = designs_root
        self.config = config
        self.design_dir = design_dir

    def _folder(self, design: Any) -> Path:
        if self.design_dir is not None:
            return self.design_dir(design)
        return self.designs_root / str(design.name)

    def __call__(self, build: Any, case: Case, recording: RecordView) -> IntegrityReport | None:
        design = build.design
        folder = self._folder(design)
        config = self.config
        if config is None:
            path = folder / FILE_NAME
            if not path.is_file():
                return None
            config = load_config(path)
        overrides: Path | None = None
        if design.materials is not None and design.materials.overrides is not None:
            overrides = folder / design.materials.overrides
        materials = MaterialSet.from_files(overrides)
        views: dict[str, MaterialView] = {}
        for member in getattr(build, "members", ()):
            views[str(member.material)] = materials.integrity_view(str(member.material))
        carried = False
        start = 0.0
        if case is not None and case.integrity is not None:
            carried = case.integrity.design_load_carried
            start = case.integrity.start_seconds()
        inputs = run_inputs(build, recording, views, design_load_carried=carried)
        lines: list[str] = []
        if carried and design.structural is not None:
            lines.append(CARRIED_NOTE)
        if start > 0.0:
            lines.append(window_note(start))
            inputs = inputs.windowed(start)
        return evaluate(config, inputs, lines, tuple(getattr(recording, "validity", ())))
