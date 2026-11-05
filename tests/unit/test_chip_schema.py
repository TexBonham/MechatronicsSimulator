"""Unit tier: chip definitions as data, the pin map a design's pin-binding table is checked against (DES-029)."""

from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError
from ruamel.yaml import YAML

from mechsim.design import chip as chip_schema
from mechsim.design.chip import Chip, ChipError, load_chip, load_chips, shipped_chips, validate_chip_file

ROOT = Path(__file__).resolve().parents[2]
PLACEHOLDER_CHIP = ROOT / "chips" / "stm32_placeholder" / "chip.yaml"


def shipped() -> dict[str, Any]:
    data: dict[str, Any] = YAML(typ="safe").load(PLACEHOLDER_CHIP.read_text(encoding="utf-8"))
    return copy.deepcopy(data)


def rejected(data: Any, *fragments: str) -> None:
    with pytest.raises(ValidationError) as caught:
        Chip.model_validate(data)
    text = str(caught.value)
    for fragment in fragments:
        assert fragment in text, text


def write_chip(folder: Path, data: dict[str, Any]) -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / "chip.yaml"
    yaml = YAML(typ="safe")
    with path.open("w", encoding="utf-8") as stream:
        yaml.dump(data, stream)
    return path


@pytest.mark.req("DES-029")
def test_shipped_placeholder_chip_lists_the_gantry_pins() -> None:
    chip = load_chip(PLACEHOLDER_CHIP)
    assert chip.status == "placeholder"
    assert chip.family == "STM32"
    assert chip.platform_description == "placeholder"
    assert chip.svd == "placeholder"
    modes = chip.pin_modes()
    for index in range(9):
        assert "digital_output" in modes[f"PA{index}"]
    for index in range(7):
        assert "digital_input" in modes[f"PB{index}"]
    assert "digital_output" in modes["PB6"]
    for name in ("PA0", "PA3", "PA6"):
        assert "pulse_output" in modes[name]
    assert "uart_tx" in modes["PA9"]
    assert "uart_rx" in modes["PA10"]
    for index in range(6):
        assert "counter_input" in modes[f"PC{index}"]
    assert shipped_chips() == {"stm32_placeholder": chip}


@pytest.mark.req("DES-029")
@pytest.mark.parametrize("field", ["name", "family", "status", "platform_description", "svd", "pins"])
def test_required_field_missing(field: str) -> None:
    data = shipped()
    del data[field]
    rejected(data, field)


@pytest.mark.req("DES-029")
@pytest.mark.parametrize("field", ["name", "modes"])
def test_pin_field_missing(field: str) -> None:
    data = shipped()
    del data["pins"][0][field]
    rejected(data, field)


@pytest.mark.req("DES-029")
def test_duplicate_pin_rejected() -> None:
    data = shipped()
    data["pins"][1]["name"] = "PA0"
    rejected(data, "pins[1]: duplicate pin 'PA0'")


@pytest.mark.req("DES-029")
def test_duplicate_mode_on_a_pin_rejected() -> None:
    data = shipped()
    data["pins"][0]["modes"] = ["digital_input", "digital_input"]
    rejected(data, "pin 'PA0' lists mode 'digital_input' more than once")


@pytest.mark.req("DES-029")
@pytest.mark.parametrize(
    "mode",
    [
        "digital_in",
        "bitbang",
        "",
        "DIGITAL_INPUT",
        1,
        "gpio_input",
        "timer_encoder",
        "timer_pwm",
        "adc",
        "uart",
        "dac",
        "spi",
        "i2c",
    ],
)
def test_unknown_mode_rejected(mode: Any) -> None:
    data = shipped()
    data["pins"][0]["modes"] = [mode]
    rejected(data, "modes")


@pytest.mark.req("DES-029")
@pytest.mark.parametrize(
    "mode", ["digital_input", "digital_output", "counter_input", "adc_input", "pulse_output", "uart_rx", "uart_tx"]
)
def test_every_peripheral_mode_accepted(mode: str) -> None:
    data = shipped()
    data["pins"][0]["modes"] = [mode]
    assert Chip.model_validate(data).pins[0].modes == [mode]


@pytest.mark.req("DES-029")
@pytest.mark.parametrize(
    ("field", "value"),
    [("pins", []), ("modes", []), ("modes", "digital_input")],
)
def test_empty_or_wrong_type_collections_rejected(field: str, value: Any) -> None:
    data = shipped()
    if field == "pins":
        data["pins"] = value
    else:
        data["pins"][0]["modes"] = value
    rejected(data, field)


@pytest.mark.req("DES-029")
@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("name", "STM32"),
        ("name", ""),
        ("family", ""),
        ("family", 32),
        ("status", "draft"),
        ("status", True),
        ("platform_description", "/abs/board.repl"),
        ("platform_description", "../board.repl"),
        ("svd", "a/../b.svd"),
        ("svd", ""),
        ("description", 1),
    ],
)
def test_bad_header_field_rejected(field: str, value: Any) -> None:
    data = shipped()
    data[field] = value
    rejected(data, field)


@pytest.mark.req("DES-029")
@pytest.mark.parametrize("name", ["pa0", "0PA", "", "P-A0"])
def test_bad_pin_name_rejected(name: str) -> None:
    data = shipped()
    data["pins"][0]["name"] = name
    rejected(data, "name")


@pytest.mark.req("DES-029")
@pytest.mark.parametrize("where", ["root", "pin"])
def test_unknown_key_rejected(where: str) -> None:
    data = shipped()
    target = data
    if where == "pin":
        target = data["pins"][0]
    target["alternate_function"] = "AF1"
    rejected(data, "alternate_function")


@pytest.mark.req("DES-029")
def test_placeholder_status_may_leave_both_files_placeholder() -> None:
    chip = Chip.model_validate(shipped())
    assert chip.status == "placeholder"


@pytest.mark.req("DES-029")
@pytest.mark.parametrize("field", ["platform_description", "svd"])
def test_verified_chip_must_name_both_files(field: str) -> None:
    data = shipped()
    data["status"] = "verified"
    data["platform_description"] = "board.repl"
    data["svd"] = "part.svd"
    data[field] = "placeholder"
    rejected(data, f"{field}: a verified chip must name its file")


@pytest.mark.req("DES-029")
def test_verified_chip_with_files_loads(tmp_path: Path) -> None:
    data = shipped()
    data["name"] = "real_part"
    data["status"] = "verified"
    data["platform_description"] = "board.repl"
    data["svd"] = "svd/part.svd"
    folder = tmp_path / "real_part"
    path = write_chip(folder, data)
    (folder / "board.repl").write_text("", encoding="utf-8")
    assert validate_chip_file(path) == [f"{path}: svd: missing file 'svd/part.svd'"]
    (folder / "svd").mkdir()
    (folder / "svd" / "part.svd").write_text("", encoding="utf-8")
    assert validate_chip_file(path) == []
    assert load_chip(path).status == "verified"


@pytest.mark.req("DES-029")
def test_placeholder_chip_naming_a_missing_file_rejected(tmp_path: Path) -> None:
    data = shipped()
    data["platform_description"] = "board.repl"
    path = write_chip(tmp_path / "stm32_placeholder", data)
    with pytest.raises(ChipError, match=re.escape("platform_description: missing file 'board.repl'")):
        load_chip(path)


@pytest.mark.req("DES-029")
def test_chip_name_must_match_its_folder(tmp_path: Path) -> None:
    path = write_chip(tmp_path / "other_chip", shipped())
    assert validate_chip_file(path) == [f"{path}: name: chip 'stm32_placeholder' must match its folder 'other_chip'"]


@pytest.mark.req("DES-029")
def test_yaml_parse_error_names_the_file(tmp_path: Path) -> None:
    folder = tmp_path / "broken"
    folder.mkdir()
    path = folder / "chip.yaml"
    path.write_text("pins: [\n", encoding="utf-8")
    errors = validate_chip_file(path)
    assert len(errors) == 1
    assert errors[0].startswith(f"{path}: YAML parse error")


@pytest.mark.req("DES-029")
def test_validation_errors_are_prefixed_by_the_file(tmp_path: Path) -> None:
    data = shipped()
    data["status"] = "draft"
    path = write_chip(tmp_path / "stm32_placeholder", data)
    errors = validate_chip_file(path)
    assert len(errors) > 0
    assert all(error.startswith(f"{path}: ") for error in errors)


@pytest.mark.req("DES-029")
def test_load_chips_collects_good_chips_and_every_error(tmp_path: Path) -> None:
    write_chip(tmp_path / "stm32_placeholder", shipped())
    bad = shipped()
    bad["name"] = "bad_chip"
    bad["pins"] = []
    write_chip(tmp_path / "bad_chip", bad)
    chips, errors = load_chips(tmp_path)
    assert list(chips) == ["stm32_placeholder"]
    assert len(errors) == 1
    assert "bad_chip" in errors[0]
    assert load_chips(tmp_path / "empty") == ({}, [])


@pytest.mark.req("DES-029")
def test_cli_validates_and_prints_schema(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert chip_schema.main(["--validate", str(PLACEHOLDER_CHIP)]) == 0
    bad = shipped()
    bad["status"] = "draft"
    path = write_chip(tmp_path / "stm32_placeholder", bad)
    assert chip_schema.main(["--validate", str(path)]) == 1
    assert "status" in capsys.readouterr().err
    assert chip_schema.main([]) == 0
    assert capsys.readouterr().out == chip_schema.schema_text()
