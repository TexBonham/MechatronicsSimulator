"""The runner's integrity hook (`testing.driver.IntegrityCheck`): configuration and materials from the design's
folder, inputs from the build and the recording, the report back (INT-022)."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

from mechsim.integrity.config import IntegrityConfig, config_for
from mechsim.integrity.inputs import run_inputs
from mechsim.integrity.plugin import MaterialView
from mechsim.integrity.table import IntegrityReport, evaluate
from mechsim.materials.database import MaterialSet
from mechsim.testing.driver import RecordView
from mechsim.testing.schema import DESIGNS_ROOT, Case


class IntegrityService:
    """`(build, case, recording) -> IntegrityReport`. The design's folder is `designs_root / <design name>` unless
    `design_dir` maps the design to it; `config` overrides the folder's `integrity.yaml`."""

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

    def __call__(self, build: Any, case: Case, recording: RecordView) -> IntegrityReport:
        design = build.design
        folder = self._folder(design)
        config = self.config
        if config is None:
            config = config_for(folder)
        overrides: Path | None = None
        if design.materials is not None and design.materials.overrides is not None:
            overrides = folder / design.materials.overrides
        materials = MaterialSet.from_files(overrides)
        views: dict[str, MaterialView] = {}
        for member in getattr(build, "members", ()):
            views[str(member.material)] = materials.integrity_view(str(member.material))
        return evaluate(config, run_inputs(build, recording, views))
