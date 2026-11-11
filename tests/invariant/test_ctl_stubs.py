"""CTL invariant tier: CTL-014 checked over the controller runtime's include graph; the rest are stubs (T3)."""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
INCLUDE_DIRS = (
    ROOT / "core/controller/include",
    ROOT / "core/kernel/include",
    ROOT / "core/kernel/pins/include",
    ROOT / "core/signals/include",
)
# The runtime side of the controller: the interface, both runtime implementations and the channel map.
RUNTIME_SOURCES = (
    ROOT / "core/controller/include/mechsim/controller/controller_runtime.hpp",
    ROOT / "core/controller/include/mechsim/controller/native_controller_runtime.hpp",
    ROOT / "core/controller/include/mechsim/controller/channel_map.hpp",
    ROOT / "core/controller/src/native_controller_runtime.cpp",
    ROOT / "core/controller/src/null_controller_runtime.cpp",
    ROOT / "core/controller/src/channel_map.cpp",
)
# Kernel headers carrying physical state, or the coupling table through which it travels (KER-020).
PHYSICAL_STATE_HEADERS = (
    "mechsim/kernel/physics_types.hpp",
    "mechsim/kernel/physics_adapter.hpp",
    "mechsim/kernel/physics_party.hpp",
    "mechsim/kernel/tick_record.hpp",
    "mechsim/kernel/coupling.hpp",
)
INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.MULTILINE)


def _resolve(name: str, including: Path) -> Path | None:
    for directory in (including.parent, *INCLUDE_DIRS):
        candidate = directory / name
        if candidate.is_file():
            return candidate
    return None


def _closure(source: Path) -> set[str]:
    """Every repository header reached from `source`, as include names; system and board headers end the walk."""
    reached: set[str] = set()
    pending = [source]
    seen: set[Path] = set()
    while pending:
        current = pending.pop()
        if current in seen:
            continue
        seen.add(current)
        for name in INCLUDE.findall(current.read_text(encoding="utf-8")):
            reached.add(name)
            resolved = _resolve(name, current)
            if resolved is not None:
                pending.append(resolved)
    return reached


@pytest.mark.req("CTL-014")
@pytest.mark.parametrize("source", RUNTIME_SOURCES, ids=lambda path: path.name)
def test_ctl_014_runtime_reaches_no_physical_state(source: Path) -> None:
    """The controller runtime shall receive only pin inputs and power rail state, never true physical state, in
    either mode: no header it includes, directly or transitively, carries a physics type or the coupling table."""
    reached = _closure(source)
    assert "mechsim/pins/pins.hpp" in reached
    assert sorted(set(PHYSICAL_STATE_HEADERS) & reached) == []


@pytest.mark.req("CTL-014")
def test_ctl_014_runtime_interface_takes_pins_and_rail_only() -> None:
    """Every ControllerRuntime entry point takes firmware, pin tables, a rail or a voltage, nothing else."""
    text = (ROOT / "core/controller/include/mechsim/controller/controller_runtime.hpp").read_text(encoding="utf-8")
    body = text[text.index("class ControllerRuntime {") : text.index("};", text.index("class ControllerRuntime {"))]
    signatures = re.findall(r"virtual void (\w+)\(([^)]*)\)", body)
    assert signatures == [
        ("load", "const FirmwareImage& firmware"),
        ("reset", "const pins::PowerRail& rail"),
        ("tick", "const PinInputs& inputs, PinOutputs& outputs"),
        ("freeze", "bool frozen"),
        ("brownout", "double voltage"),
    ]


@pytest.mark.req("CTL-014")
def test_ctl_014_closure_walks_transitive_includes(tmp_path: Path) -> None:
    """The checker itself: a physics header two includes away is found."""
    (tmp_path / "a.hpp").write_text('#include "b.hpp"\n', encoding="utf-8")
    (tmp_path / "b.hpp").write_text('#include "mechsim/kernel/tick_record.hpp"\n', encoding="utf-8")
    reached = _closure(tmp_path / "a.hpp")
    assert "mechsim/kernel/tick_record.hpp" in reached
    assert "mechsim/kernel/physics_types.hpp" in reached


@pytest.mark.req("CTL-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_017() -> None:
    """No Renode type shall cross the `core/controller/` runtime boundary."""
    raise NotImplementedError


@pytest.mark.req("CTL-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_022() -> None:
    """An emulated run shall be deterministic, with Renode's virtual time driven only by the kernel."""
    raise NotImplementedError
