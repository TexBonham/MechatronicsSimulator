#include "test_gantry_plant.cpp"

#include <algorithm>

using gantry_test::has_event;
using gantry_test::Plant;

namespace {

bool ready(const mechsim_board_outputs& out)
{
    return out.state_code == GANTRY_STATE_READY;
}

}  // namespace

TEST_CASE("a design-supplied tick rate reaches the gantry firmware", "[FW-010][CTL-002]")
{
    REQUIRE(MECHSIM_APP_TICK_HZ == 500U);
    REQUIRE(mechsim_app_tick_hz() == 500U);
}

TEST_CASE("at 500 Hz the cruise step rate doubles per tick and the move still lands exactly", "[FW-022][FW-015]")
{
    Plant plant(0, 0);
    plant.home();
    std::uint32_t peak = 0U;
    plant.tick("target 0.4 0.2\n");
    REQUIRE(plant.run_until(
                [&peak](const mechsim_board_outputs& o) {
                    peak = std::max(peak, o.step_count[MECHSIM_CH_X_DRIVER_STEP]);
                    return ready(o);
                },
                100000)
        > 0);
    plant.tick();
    REQUIRE(peak == 32U);
    REQUIRE(plant.position[0] == 32000);
    REQUIRE(plant.position[1] == 16000);
}

TEST_CASE("queue policy holds one target during a move and refuses a second", "[FW-015][FW-030]")
{
    Plant plant(0, 0);
    plant.home();
    mechsim_board_outputs out = plant.tick("target 0.05 0\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    out = plant.tick("target 0.02 0.01\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    out = plant.tick("target 0.03 0.03\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_QUEUE_FULL);
    REQUIRE(has_event(plant.take_events(), GANTRY_EVENT_REFUSED, GANTRY_FAULT_QUEUE_FULL));
    out = plant.tick("target 9 0\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_OUTSIDE_WORKSPACE);

    std::int64_t peak_x = 0;
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return ready(o); }, 100000) > 0);
    out = plant.tick();
    peak_x = plant.position[0];
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.digital_out[MECHSIM_CH_X_DRIVER_DIR] == GANTRY_DIR_NEGATIVE_LEVEL);
    REQUIRE(plant.run_until(
                [&plant, &peak_x](const mechsim_board_outputs& o) {
                    peak_x = std::max(peak_x, plant.position[0]);
                    return ready(o);
                },
                100000)
        > 0);
    plant.tick();
    REQUIRE(peak_x == 4000);
    REQUIRE(plant.position[0] == 1600);
    REQUIRE(plant.position[1] == 800);
    const std::vector<std::string> lines = plant.take_lines();
    REQUIRE(std::count(lines.begin(), lines.end(), std::string("status 3 0")) == 2);
}

TEST_CASE("a queued target starts on the tick after arrival, ahead of that tick's commands", "[FW-015][FW-030]")
{
    Plant plant(0, 0);
    plant.home();
    plant.tick("target 0.01 0\n");
    plant.tick("target 0.02 0\n");
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return ready(o); }, 100000) > 0);
    const mechsim_board_outputs out = plant.tick("home\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.fault_code == GANTRY_FAULT_BUSY);
    REQUIRE(plant.position[0] == 800);
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return ready(o); }, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 1600);
    for (int tick = 0; tick < 20; ++tick) {
        REQUIRE(plant.tick().state_code == GANTRY_STATE_READY);
    }
}

TEST_CASE("a transmit queue too small for the pending lines latches a fault", "[FW-029][FW-030]")
{
    Plant plant(800, 800);
    mechsim_board_outputs out = plant.tick("x\nx\nx\nx\nx\n");
    REQUIRE(out.state_code == GANTRY_STATE_NOT_HOMED);
    REQUIRE(out.uart_tx_count == 55U);
    out = plant.tick("x\nx\nx\nx\nx\nx\n");
    REQUIRE(out.state_code == GANTRY_STATE_FAULT);
    REQUIRE(out.fault_code == GANTRY_FAULT_TX_OVERFLOW);
}

TEST_CASE("G13: under the queue policy a lift during a move waits in the one slot shared with targets",
    "[FW-027][FW-026][FW-030]")
{
    Plant plant(0, 0, 0);
    plant.home();
    mechsim_board_outputs out = plant.tick("target 0.05 0\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    out = plant.tick("lift 0.02\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    out = plant.tick("target 0.01 0\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_QUEUE_FULL);
    out = plant.tick("lift 0.03\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_QUEUE_FULL);
    while (out.state_code == GANTRY_STATE_MOVING) {
        REQUIRE(out.step_count[MECHSIM_CH_LIFT_DRIVER_STEP] == 0U);
        out = plant.tick();
    }
    REQUIRE(ready(out));
    out = plant.tick();
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.digital_out[MECHSIM_CH_LIFT_DRIVER_DIR] == GANTRY_DIR_POSITIVE_LEVEL);
    out = plant.tick("target 0 0\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    while (out.state_code == GANTRY_STATE_MOVING) {
        REQUIRE(out.step_count[MECHSIM_CH_X_DRIVER_STEP] == 0U);
        out = plant.tick();
    }
    out = plant.tick();
    REQUIRE(plant.position[0] == 4000);
    REQUIRE(plant.position[2] == 1600);
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(plant.run_until(ready, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 0);
    REQUIRE(plant.position[2] == 1600);
}

TEST_CASE("G5: a queued lift up is refused with overload when it would start under an overload", "[FW-027][FW-018]")
{
    Plant plant(0, 0, 0);
    plant.home();
    plant.tick("target 0.05 0\n");
    REQUIRE(plant.tick("lift 0.02\n").fault_code == GANTRY_FAULT_NONE);
    plant.load_adc = gantry_test::load_counts(GANTRY_OVERLOAD_LIMIT_MILLIKG + 1);
    mechsim_board_outputs out = plant.last;
    while (out.state_code == GANTRY_STATE_MOVING) {
        out = plant.tick();
    }
    plant.take_lines();
    out = plant.tick();
    REQUIRE(out.state_code == GANTRY_STATE_READY);
    REQUIRE(out.fault_code == GANTRY_FAULT_OVERLOAD);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 3 10"});
    REQUIRE(has_event(plant.take_events(), GANTRY_EVENT_REFUSED, GANTRY_FAULT_OVERLOAD));
    for (int tick = 0; tick < 20; ++tick) {
        REQUIRE(plant.tick().step_count[MECHSIM_CH_LIFT_DRIVER_STEP] == 0U);
    }
}

TEST_CASE("G13: a lift while a lift is stopping for an overload is refused as busy", "[FW-027][FW-019]")
{
    Plant plant(0, 0, 0);
    plant.home();
    plant.tick("lift 0.3\n");
    for (int tick = 0; tick < 200; ++tick) {
        plant.tick();
    }
    plant.load_adc = gantry_test::load_counts(GANTRY_OVERLOAD_LIMIT_MILLIKG + 1);
    plant.tick();
    REQUIRE(plant.tick().fault_code == GANTRY_FAULT_OVERLOAD);
    mechsim_board_outputs out = plant.tick("lift 0\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.fault_code == GANTRY_FAULT_BUSY);
    REQUIRE(plant.tick("target 0.1 0\n").fault_code == GANTRY_FAULT_BUSY);
    REQUIRE(plant.run_until(ready, 100000) > 0);
    out = plant.tick();
    REQUIRE(out.state_code == GANTRY_STATE_READY);
}
