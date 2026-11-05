"""Catalog alignment (brief M1-X1 item 5): torque-speed table start, E-stop relay pins, power supply output."""

import pytest

from mechsim.library.registry import default_registry


@pytest.mark.req("LIB-034")
def test_stepper_curve_starts_at_zero_speed_with_the_holding_torque() -> None:
    registry = default_registry()
    part = registry.resolve_part("nema17_class_generic")
    assert part.torque_speed_curve is not None
    assert part.torque_speed_curve[0].speed == 0.0
    assert part.torque_speed_curve[0].torque == part.values["holding_torque"]
    with pytest.raises(ValueError, match="not the holding torque"):
        registry.resolve_part("nema17_class_generic", {"holding_torque": "0.5 N*m"})


@pytest.mark.req("LIB-034")
def test_estop_relay_has_no_heartbeat_and_supply_output_is_v_out() -> None:
    registry = default_registry()
    relay_pins = [pin.name for pin in registry.component_type("estop_relay").electrical.pins]
    assert "heartbeat" not in relay_pins
    supply_pins = [pin.name for pin in registry.component_type("power_supply").electrical.pins]
    assert "v_out" in supply_pins
