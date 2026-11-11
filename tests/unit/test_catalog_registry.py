"""Unit tier: the catalog registry merges every shipped catalog file and rejects duplicate names (DES-039)."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from mechsim.library.registry import RegistryError, default_registry, load_registry
from mechsim.library.schema import CATALOG_DIR

SHIPPED_TYPES = [
    "belt_pulley",
    "brake",
    "estop_relay",
    "imu",
    "incremental_encoder",
    "lead_screw",
    "limit_switch",
    "load_cell",
    "load_cell_amplifier",
    "power_supply",
    "step_dir_driver",
    "stepper_behavioral",
    "uart_link",
    "watchdog",
]


def copy_catalog(directory: Path, names: list[str]) -> None:
    for name in names:
        shutil.copy(CATALOG_DIR / f"{name}.yaml", directory / f"{name}.yaml")


@pytest.mark.req("DES-039")
def test_registry_loads_every_shipped_type_and_part() -> None:
    registry = load_registry()
    assert registry.type_names() == SHIPPED_TYPES
    assert "nema17_class_generic" in registry.part_names()
    assert len(registry.part_names()) == len(SHIPPED_TYPES)
    assert registry.part("nema17_class_generic").type == "stepper_behavioral"
    assert registry.component_type("limit_switch").category == "sensor"


@pytest.mark.req("DES-039")
def test_default_registry_is_loaded_once() -> None:
    assert default_registry() is default_registry()
    assert default_registry().type_names() == SHIPPED_TYPES


@pytest.mark.req("DES-039")
@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-027")
@pytest.mark.parametrize(
    ("type_name", "part_name", "category"),
    [
        ("load_cell", "load_cell_5kg_generic", "sensor"),
        ("load_cell_amplifier", "load_cell_amplifier_generic", "front_end"),
        ("imu", "imu_6axis_generic", "sensor"),
        ("brake", "brake_24v_generic", "safety"),
    ],
)
def test_milestone_2_types_resolve_from_the_default_registry(type_name: str, part_name: str, category: str) -> None:
    registry = default_registry()
    assert registry.has_type(type_name)
    assert registry.component_type(type_name).category == category
    assert registry.part(part_name).type == type_name
    assert registry.resolve_part(part_name).type == type_name


@pytest.mark.req("DES-039")
def test_duplicate_type_across_files_rejected(tmp_path: Path) -> None:
    copy_catalog(tmp_path, ["limit_switch"])
    text = (CATALOG_DIR / "limit_switch.yaml").read_text(encoding="utf-8")
    renamed = text.replace("microswitch_generic", "microswitch_other")
    (tmp_path / "limit_switch_copy.yaml").write_text(renamed, encoding="utf-8")
    with pytest.raises(RegistryError, match=r"duplicate component type 'limit_switch' in .*limit_switch\.yaml and"):
        load_registry(tmp_path)


@pytest.mark.req("DES-039")
def test_duplicate_part_across_files_rejected(tmp_path: Path) -> None:
    copy_catalog(tmp_path, ["limit_switch"])
    text = (CATALOG_DIR / "limit_switch.yaml").read_text(encoding="utf-8")
    renamed = text.replace("name: limit_switch", "name: limit_switch_other").replace(
        "type: limit_switch", "type: limit_switch_other"
    )
    assert renamed != text
    (tmp_path / "other.yaml").write_text(renamed, encoding="utf-8")
    with pytest.raises(RegistryError, match="duplicate catalog part 'microswitch_generic'"):
        load_registry(tmp_path)


@pytest.mark.req("DES-039")
def test_empty_catalog_directory_rejected(tmp_path: Path) -> None:
    with pytest.raises(RegistryError, match="no catalog files"):
        load_registry(tmp_path)


@pytest.mark.req("DES-039")
def test_subset_directory_loads_only_its_files(tmp_path: Path) -> None:
    copy_catalog(tmp_path, ["belt_pulley", "lead_screw"])
    registry = load_registry(tmp_path)
    assert registry.type_names() == ["belt_pulley", "lead_screw"]
    assert not registry.has_type("limit_switch")
    assert registry.has_part("tr8x8_generic")


@pytest.mark.req("DES-030")
@pytest.mark.req("DES-039")
def test_resolve_part_with_override() -> None:
    registry = load_registry()
    base = registry.part("gt2_20t_6mm_generic").values["efficiency"]
    resolved = registry.resolve_part("gt2_20t_6mm_generic", {"efficiency": "0.5"})
    assert resolved.values["efficiency"] == pytest.approx(0.5)
    assert registry.part("gt2_20t_6mm_generic").values["efficiency"] == base
    assert registry.resolve_part("gt2_20t_6mm_generic").values["efficiency"] == base


@pytest.mark.req("DES-030")
@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"efficiency": "1.5"}, "above maximum"),
        ({"efficiency": "0.001"}, "below minimum"),
        ({"efficiency": "0.5 m"}, "expects dimension"),
        ({"colour": "1 m"}, "unknown parameter 'colour'"),
        ({"self_locking": "1 m"}, "needs true or false"),
    ],
)
def test_resolve_part_with_bad_override_rejected(overrides: dict[str, str | bool], message: str) -> None:
    with pytest.raises(RegistryError, match=message):
        load_registry().resolve_part("gt2_20t_6mm_generic", overrides)


@pytest.mark.req("DES-039")
@pytest.mark.parametrize("lookup", ["component_type", "part", "resolve_part"])
def test_unknown_lookup_rejected(lookup: str) -> None:
    registry = load_registry()
    with pytest.raises(RegistryError, match="no "):
        getattr(registry, lookup)("ghost")
