#include "test_gantry_plant.cpp"

#include <algorithm>

using gantry_test::Event;
using gantry_test::has_event;
using gantry_test::parse_position;
using gantry_test::Plant;
using gantry_test::transition;

// ===== Helpers =======================================================================================================

namespace {

constexpr std::int64_t accuracy_um = 10000;
constexpr std::uint32_t max_steps_per_tick = 16U;
constexpr std::uint32_t slow_steps_per_tick = 4U;
constexpr std::int64_t slow_zone_steps = 4000;

std::uint32_t steps_x(const mechsim_board_outputs& out)
{
    return out.step_count[MECHSIM_CH_X_DRIVER_STEP];
}

std::uint32_t steps_y(const mechsim_board_outputs& out)
{
    return out.step_count[MECHSIM_CH_Y_DRIVER_STEP];
}

bool is_state(const mechsim_board_outputs& out, std::uint8_t state)
{
    return out.state_code == state;
}

struct MoveTrace {
    std::vector<std::uint32_t> x;
    std::vector<std::uint32_t> y;
    std::vector<std::int64_t> x_position;
};

MoveTrace run_move(Plant& plant, const std::string& command)
{
    MoveTrace trace;
    mechsim_board_outputs out = plant.tick(command);
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    trace.x.push_back(steps_x(out));
    trace.y.push_back(steps_y(out));
    trace.x_position.push_back(plant.position[0]);
    for (int tick = 0; tick < 200000 && out.state_code == GANTRY_STATE_MOVING; ++tick) {
        out = plant.tick();
        trace.x.push_back(steps_x(out));
        trace.y.push_back(steps_y(out));
        trace.x_position.push_back(plant.position[0]);
        REQUIRE(out.digital_out[MECHSIM_CH_X_DRIVER_ENABLE] == GANTRY_ENABLE_ACTIVE_LEVEL);
    }
    REQUIRE(out.state_code == GANTRY_STATE_READY);
    plant.tick();
    return trace;
}

void require_profile_limits(const std::vector<std::uint32_t>& steps)
{
    std::int64_t previous = 0;
    for (std::uint32_t value : steps) {
        REQUIRE(value <= max_steps_per_tick);
        REQUIRE(std::llabs(static_cast<std::int64_t>(value) - previous) <= 2);
        previous = static_cast<std::int64_t>(value);
    }
    REQUIRE(previous <= 2);
}

}  // namespace

// ===== G1 not homed ==================================================================================================

TEST_CASE("gantry tick rate is the header default", "[FW-010][CTL-002]")
{
    REQUIRE(MECHSIM_APP_TICK_HZ == 1000U);
    REQUIRE(mechsim_app_tick_hz() == 1000U);
}

TEST_CASE("G1: every target before homing is refused with no motion", "[FW-013][FW-029][FW-030][CTL-018]")
{
    Plant plant(800, 800);
    mechsim_board_outputs out = plant.tick();
    REQUIRE(out.state_code == GANTRY_STATE_NOT_HOMED);
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    REQUIRE(plant.take_lines().empty());

    const char* commands[] = {"target 0.1 0.1\n", "target 0 0\n", "target 5 5\n", "target -1 0\n"};
    for (const char* command : commands) {
        out = plant.tick(command);
        REQUIRE(out.state_code == GANTRY_STATE_NOT_HOMED);
        REQUIRE(out.fault_code == GANTRY_FAULT_NOT_HOMED);
        REQUIRE(plant.take_lines() == std::vector<std::string>{"status 1 1"});
        const std::vector<Event> events = plant.take_events();
        REQUIRE(events.size() == 2U);
        REQUIRE(events[0].code == GANTRY_EVENT_COMMAND);
        REQUIRE(events[0].arg == GANTRY_COMMAND_TARGET);
        REQUIRE(events[1].code == GANTRY_EVENT_REFUSED);
        REQUIRE(events[1].arg == GANTRY_FAULT_NOT_HOMED);
    }
    for (int tick = 0; tick < 2000; ++tick) {
        out = plant.tick();
        REQUIRE(steps_x(out) == 0U);
        REQUIRE(steps_y(out) == 0U);
        REQUIRE(out.digital_out[MECHSIM_CH_X_DRIVER_ENABLE] == GANTRY_ENABLE_INACTIVE_LEVEL);
        REQUIRE(out.digital_out[MECHSIM_CH_Y_DRIVER_ENABLE] == GANTRY_ENABLE_INACTIVE_LEVEL);
        REQUIRE(out.state_code == GANTRY_STATE_NOT_HOMED);
        REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    }
    REQUIRE(plant.position[0] == 800);
    REQUIRE(plant.position[1] == 800);
    REQUIRE(plant.take_lines().empty());
}

TEST_CASE("G1: a reset returns to NOT_HOMED and refuses targets again", "[FW-013][CTL-002]")
{
    Plant plant(400, 400);
    plant.home();
    REQUIRE(plant.tick().state_code == GANTRY_STATE_READY);
    mechsim_app_init();
    mechsim_board_outputs out = plant.tick("target 0.1 0.1\n");
    REQUIRE(out.state_code == GANTRY_STATE_NOT_HOMED);
    REQUIRE(out.fault_code == GANTRY_FAULT_NOT_HOMED);
    REQUIRE(steps_x(out) == 0U);
    REQUIRE(out.digital_out[MECHSIM_CH_X_DRIVER_ENABLE] == GANTRY_ENABLE_INACTIVE_LEVEL);
}

// ===== G2 homing =====================================================================================================

TEST_CASE("G2: homing drives each axis to its min switch, sets the origin and reports ready",
    "[FW-014][FW-029][FW-030][CTL-018]")
{
    Plant plant(1600, 800);
    mechsim_board_outputs out = plant.tick("home\n");
    REQUIRE(out.state_code == GANTRY_STATE_HOMING);
    REQUIRE(out.digital_out[MECHSIM_CH_X_DRIVER_DIR] == GANTRY_DIR_NEGATIVE_LEVEL);
    REQUIRE(out.digital_out[MECHSIM_CH_Y_DRIVER_DIR] == GANTRY_DIR_NEGATIVE_LEVEL);
    REQUIRE(out.digital_out[MECHSIM_CH_X_DRIVER_ENABLE] == GANTRY_ENABLE_ACTIVE_LEVEL);
    std::vector<Event> events = plant.take_events();
    REQUIRE(has_event(events, GANTRY_EVENT_COMMAND, GANTRY_COMMAND_HOME));
    REQUIRE(has_event(events, GANTRY_EVENT_STATE, transition(GANTRY_STATE_NOT_HOMED, GANTRY_STATE_HOMING)));

    std::uint32_t peak = 0U;
    const std::int64_t ticks = plant.run_until(
        [&peak](const mechsim_board_outputs& o) {
            if (steps_x(o) > peak) {
                peak = steps_x(o);
            }
            return o.state_code != GANTRY_STATE_HOMING;
        },
        100000);
    REQUIRE(ticks > 0);
    REQUIRE(plant.last.state_code == GANTRY_STATE_READY);
    REQUIRE(peak <= 2U);
    REQUIRE(plant.position[0] <= 0);
    REQUIRE(plant.position[0] * 1000000 / gantry_test::steps_per_m > -accuracy_um);
    REQUIRE(plant.position[1] <= 0);
    REQUIRE(plant.position[1] * 1000000 / gantry_test::steps_per_m > -accuracy_um);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 3 0"});
    events = plant.take_events();
    REQUIRE(has_event(events, GANTRY_EVENT_ORIGIN, GANTRY_AXIS_X));
    REQUIRE(has_event(events, GANTRY_EVENT_ORIGIN, GANTRY_AXIS_Y));
    REQUIRE(has_event(events, GANTRY_EVENT_STATE, transition(GANTRY_STATE_HOMING, GANTRY_STATE_READY)));

    for (int tick = 0; tick < 50; ++tick) {
        out = plant.tick();
        REQUIRE(steps_x(out) == 0U);
        REQUIRE(steps_y(out) == 0U);
    }
    plant.run_until([](const mechsim_board_outputs& o) { return o.uart_tx_count > 0U; }, GANTRY_STATUS_PERIOD_TICKS);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"current_position 0.000000 0.000000"});
}

TEST_CASE("G2: homing an axis already on its switch sets the origin on the first tick", "[FW-014]")
{
    Plant plant(0, 0);
    mechsim_board_outputs out = plant.tick("home\n");
    REQUIRE(out.state_code == GANTRY_STATE_READY);
    REQUIRE(steps_x(out) == 0U);
    REQUIRE(steps_y(out) == 0U);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 3 0"});
    const std::vector<Event> events = plant.take_events();
    REQUIRE(has_event(events, GANTRY_EVENT_STATE, transition(GANTRY_STATE_NOT_HOMED, GANTRY_STATE_HOMING)));
    REQUIRE(has_event(events, GANTRY_EVENT_STATE, transition(GANTRY_STATE_HOMING, GANTRY_STATE_READY)));
}

TEST_CASE("G2: homing re-zeroes the origin from READY", "[FW-014]")
{
    Plant plant(0, 0);
    plant.home();
    run_move(plant, "target 0.02 0.01\n");
    plant.take_lines();
    plant.home();
    REQUIRE(plant.position[0] <= 0);
    REQUIRE(plant.position[1] <= 0);
    plant.run_until([](const mechsim_board_outputs& o) { return o.uart_tx_count > 0U; }, GANTRY_STATUS_PERIOD_TICKS);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"current_position 0.000000 0.000000"});
}

TEST_CASE("G2: a switch that never trips faults the homing", "[FW-014][FW-030]")
{
    Plant plant(800, 800);
    plant.switches_work = false;
    plant.tick("home\n");
    const std::int64_t ticks =
        plant.run_until([](const mechsim_board_outputs& o) { return o.state_code == GANTRY_STATE_FAULT; }, 200000);
    REQUIRE(ticks > 0);
    REQUIRE(plant.position[0] <= 800 - 96000);
    mechsim_board_outputs out = plant.last;
    REQUIRE(out.fault_code == GANTRY_FAULT_HOMING_FAILED);
    REQUIRE(steps_x(out) == 0U);
    REQUIRE(out.digital_out[MECHSIM_CH_X_DRIVER_ENABLE] == GANTRY_ENABLE_INACTIVE_LEVEL);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 5 7"});
    const std::vector<Event> events = plant.take_events();
    REQUIRE(has_event(events, GANTRY_EVENT_STATE, transition(GANTRY_STATE_HOMING, GANTRY_STATE_FAULT)));
    REQUIRE(has_event(events, GANTRY_EVENT_FAULT, GANTRY_FAULT_HOMING_FAILED));

    out = plant.tick("target 0.1 0.1\n");
    REQUIRE(out.state_code == GANTRY_STATE_FAULT);
    REQUIRE(out.fault_code == GANTRY_FAULT_HOMING_FAILED);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 5 6"});
    REQUIRE(has_event(plant.take_events(), GANTRY_EVENT_REFUSED, GANTRY_FAULT_IN_FAULT));

    plant.switches_work = true;
    plant.on_min_switch_override = true;
    out = plant.tick("home\n");
    REQUIRE(out.state_code == GANTRY_STATE_READY);
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
}

// ===== G3 moves ======================================================================================================

TEST_CASE("G3: a target inside the workspace is reached within the accuracy and the limits",
    "[FW-015][FW-022][FW-029][FW-030]")
{
    Plant plant(800, 800);
    plant.home();
    const std::int64_t origin_x = plant.position[0];
    const std::int64_t origin_y = plant.position[1];
    const MoveTrace trace = run_move(plant, "target 0.5 0.3\n");
    require_profile_limits(trace.x);
    require_profile_limits(trace.y);
    REQUIRE(*std::max_element(trace.x.begin(), trace.x.end()) == max_steps_per_tick);
    REQUIRE((plant.position[0] - origin_x) == 40000);
    REQUIRE((plant.position[1] - origin_y) == 24000);
    for (std::int64_t x : trace.x_position) {
        REQUIRE(x <= gantry_test::workspace_x_steps);
    }

    const std::vector<std::string> lines = plant.take_lines();
    REQUIRE(lines.size() >= 3U);
    REQUIRE(lines.back() == "status 3 0");
    std::int64_t x_um = 0;
    std::int64_t y_um = 0;
    REQUIRE(parse_position(lines[lines.size() - 2U], x_um, y_um));
    REQUIRE(std::llabs(x_um - 500000) < accuracy_um);
    REQUIRE(std::llabs(y_um - 300000) < accuracy_um);
    REQUIRE(lines.size() >= 10U);
    for (std::size_t index = 0U; index + 2U < lines.size(); ++index) {
        REQUIRE(parse_position(lines[index], x_um, y_um));
    }

    const std::vector<Event> events = plant.take_events();
    REQUIRE(has_event(events, GANTRY_EVENT_COMMAND, GANTRY_COMMAND_TARGET));
    REQUIRE(has_event(events, GANTRY_EVENT_STATE, transition(GANTRY_STATE_READY, GANTRY_STATE_MOVING)));
    REQUIRE(has_event(events, GANTRY_EVENT_STATE, transition(GANTRY_STATE_MOVING, GANTRY_STATE_READY)));
}

TEST_CASE("G3: the profile ramps up and slows near the target", "[FW-022][FW-023]")
{
    Plant plant(0, 0);
    plant.home();
    const MoveTrace trace = run_move(plant, "target 0.3 0\n");
    REQUIRE(trace.x.front() <= 1U);
    std::uint32_t last_nonzero = 0U;
    for (std::size_t index = trace.x.size(); index > 0U; --index) {
        if (trace.x[index - 1U] != 0U) {
            last_nonzero = trace.x[index - 1U];
            break;
        }
    }
    REQUIRE(last_nonzero <= 1U);
    for (std::uint32_t value : trace.y) {
        REQUIRE(value == 0U);
    }
}

TEST_CASE("G3: moves into the slow zones at both travel ends stay below the slow speed", "[FW-023][FW-015]")
{
    Plant plant(0, 0);
    plant.home();
    MoveTrace trace = run_move(plant, "target 1.0 0\n");
    for (std::size_t index = 1U; index < trace.x.size(); ++index) {
        if (trace.x_position[index - 1U] >= gantry_test::workspace_x_steps - slow_zone_steps) {
            REQUIRE(trace.x[index] <= slow_steps_per_tick + 1U);
        }
    }
    REQUIRE(plant.position[0] == gantry_test::workspace_x_steps);
    REQUIRE(*std::max_element(trace.x.begin(), trace.x.end()) == max_steps_per_tick);
    plant.take_lines();

    trace = run_move(plant, "target 0 0\n");
    REQUIRE(plant.last.digital_out[MECHSIM_CH_X_DRIVER_DIR] == GANTRY_DIR_NEGATIVE_LEVEL);
    for (std::size_t index = 1U; index < trace.x.size(); ++index) {
        if (trace.x_position[index - 1U] <= slow_zone_steps) {
            REQUIRE(trace.x[index] <= slow_steps_per_tick + 1U);
        }
    }
    REQUIRE(plant.position[0] == 0);
    REQUIRE(plant.take_lines().back() == "status 3 0");
}

TEST_CASE("G3: a zero-length move arrives on the same tick with no steps", "[FW-015][FW-030]")
{
    Plant plant(0, 0);
    plant.home();
    const mechsim_board_outputs out = plant.tick("target 0 0\n");
    REQUIRE(out.state_code == GANTRY_STATE_READY);
    REQUIRE(steps_x(out) == 0U);
    REQUIRE(steps_y(out) == 0U);
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"current_position 0.000000 0.000000", "status 3 0"});
    const std::vector<Event> events = plant.take_events();
    REQUIRE(events.size() == 3U);
    REQUIRE(events[0].code == GANTRY_EVENT_COMMAND);
    REQUIRE(events[1].arg == transition(GANTRY_STATE_READY, GANTRY_STATE_MOVING));
    REQUIRE(events[2].arg == transition(GANTRY_STATE_MOVING, GANTRY_STATE_READY));
}

TEST_CASE("G3: a one-step move emits exactly one step", "[FW-015][FW-022]")
{
    Plant plant(0, 0);
    plant.home();
    const MoveTrace trace = run_move(plant, "target 0.000013 0\n");
    std::uint32_t total = 0U;
    for (std::uint32_t value : trace.x) {
        total = total + value;
    }
    REQUIRE(total == 1U);
    REQUIRE(plant.position[0] == 1);
}

TEST_CASE("G3: a target at the workspace corner is accepted and reached", "[FW-015][FW-016]")
{
    Plant plant(0, 0);
    plant.home();
    run_move(plant, "target 1.000000 0.800000\n");
    REQUIRE(plant.position[0] == gantry_test::workspace_x_steps);
    REQUIRE(plant.position[1] == gantry_test::workspace_y_steps);
    REQUIRE(plant.take_lines().back() == "status 3 0");
}

TEST_CASE("G3: a new target while moving is rejected and the first move completes", "[FW-015][FW-030]")
{
    Plant plant(0, 0);
    plant.home();
    mechsim_board_outputs out = plant.tick("target 0.1 0.1\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    plant.take_lines();
    plant.take_events();
    out = plant.tick("target 0.2 0.2\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.fault_code == GANTRY_FAULT_BUSY);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 4 3"});
    REQUIRE(has_event(plant.take_events(), GANTRY_EVENT_REFUSED, GANTRY_FAULT_BUSY));
    out = plant.tick("home\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.fault_code == GANTRY_FAULT_BUSY);
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 8000);
    REQUIRE(plant.position[1] == 8000);
}

TEST_CASE("G3: commands during homing are refused as busy", "[FW-014][FW-030]")
{
    Plant plant(8000, 8000);
    mechsim_board_outputs out = plant.tick("home\n");
    REQUIRE(out.state_code == GANTRY_STATE_HOMING);
    plant.take_lines();
    out = plant.tick("target 0.1 0.1\nhome\n");
    REQUIRE(out.state_code == GANTRY_STATE_HOMING);
    REQUIRE(out.fault_code == GANTRY_FAULT_BUSY);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 2 3", "status 2 3"});
}

// ===== G4 workspace ==================================================================================================

TEST_CASE("G4: a target outside the workspace is refused with no motion and READY kept",
    "[FW-016][FW-029][FW-030]")
{
    Plant plant(0, 0);
    plant.home();
    const char* commands[] = {"target 1.000001 0\n", "target 0 0.800001\n", "target -0.000001 0\n",
        "target 0 -0.5\n", "target 99999999999 0\n", "target 2 2\n"};
    for (const char* command : commands) {
        mechsim_board_outputs out = plant.tick(command);
        REQUIRE(out.state_code == GANTRY_STATE_READY);
        REQUIRE(out.fault_code == GANTRY_FAULT_OUTSIDE_WORKSPACE);
        REQUIRE(std::count(plant.tx.begin(), plant.tx.end(), '\n') >= 1);
        REQUIRE(plant.take_lines().back() == "status 3 2");
        const std::vector<Event> events = plant.take_events();
        REQUIRE(events.size() == 2U);
        REQUIRE(events[1].code == GANTRY_EVENT_REFUSED);
        REQUIRE(events[1].arg == GANTRY_FAULT_OUTSIDE_WORKSPACE);
        for (int tick = 0; tick < 20; ++tick) {
            out = plant.tick();
            REQUIRE(steps_x(out) == 0U);
            REQUIRE(steps_y(out) == 0U);
            REQUIRE(out.state_code == GANTRY_STATE_READY);
            REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
        }
        for (const std::string& line : plant.take_lines()) {
            REQUIRE(line == "current_position 0.000000 0.000000");
        }
    }
    REQUIRE(plant.position[0] == 0);
    REQUIRE(plant.position[1] == 0);
}

TEST_CASE("G4: a target outside the workspace during a move is refused and the move continues", "[FW-016]")
{
    Plant plant(0, 0);
    plant.home();
    plant.tick("target 0.05 0\n");
    mechsim_board_outputs out = plant.tick("target 3 0\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(out.fault_code == GANTRY_FAULT_OUTSIDE_WORKSPACE);
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 4000);
}

// ===== Operator link =================================================================================================

TEST_CASE("operator lines are buffered across ticks and parsed strictly", "[FW-029][FW-011]")
{
    Plant plant(0, 0);
    plant.home();
    mechsim_board_outputs out = plant.tick("tar");
    REQUIRE(out.state_code == GANTRY_STATE_READY);
    REQUIRE(plant.take_events().empty());
    plant.tick("get 0.0");
    out = plant.tick("1 0.02\r\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 800);
    REQUIRE(plant.position[1] == 1600);
    plant.take_lines();

    out = plant.tick("target   .01   0.\n");
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 800);
    REQUIRE(plant.position[1] == 0);
    plant.take_lines();
    plant.take_events();

    out = plant.tick("\n\r\n   \n");
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    REQUIRE(plant.take_lines().empty());
    REQUIRE(plant.take_events().empty());

    const char* malformed[] = {"target 0.1\n", "target 0.1 0.2 0.3\n", "target a 0\n", "target 0.1234567 0\n",
        "target . 0\n", "target - 0\n", "target 0.1.2 0\n", "target +-1 0\n", "TARGET 0 0\n", "home now\n",
        "homes\n", "targe 0 0\n", "move 0 0\n", "target 0 0 0 0 0\n"};
    for (const char* command : malformed) {
        out = plant.tick(command);
        REQUIRE(out.state_code == GANTRY_STATE_READY);
        REQUIRE(out.fault_code == GANTRY_FAULT_MALFORMED);
        REQUIRE(steps_x(out) == 0U);
        REQUIRE(plant.take_lines() == std::vector<std::string>{"status 3 4"});
        const std::vector<Event> events = plant.take_events();
        REQUIRE(events.size() == 2U);
        REQUIRE(events[1].arg == GANTRY_FAULT_MALFORMED);
    }

    out = plant.tick("target +0.000000 -0\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    REQUIRE(out.state_code == GANTRY_STATE_MOVING);
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 0);
}

TEST_CASE("an overlong line is refused once and the next line is parsed", "[FW-029]")
{
    Plant plant(0, 0);
    plant.home();
    const std::string long_part(40, '0');
    plant.tick("target " + long_part);
    mechsim_board_outputs out = plant.tick(long_part + "\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_MALFORMED);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 3 4"});
    std::string exact(GANTRY_LINE_MAX - 4U, ' ');
    out = plant.tick(exact + "home\n");
    REQUIRE(out.state_code == GANTRY_STATE_READY);
    REQUIRE(out.fault_code == GANTRY_FAULT_NONE);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"status 3 0"});
    std::string over(GANTRY_LINE_MAX - 3U, ' ');
    out = plant.tick(over + "home\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_MALFORMED);
}

// ===== Queues and event log ==========================================================================================

TEST_CASE("transmit beyond 64 bytes continues on the next tick in order", "[FW-029][FW-011]")
{
    Plant plant(800, 800);
    std::string burst;
    for (int index = 0; index < 12; ++index) {
        burst += "x\n";
    }
    mechsim_board_outputs out = plant.tick(burst);
    REQUIRE(out.uart_tx_count == MECHSIM_UART_TX_BYTES);
    REQUIRE(out.event_count == MECHSIM_EVENT_LOG_COUNT);
    out = plant.tick();
    REQUIRE(out.uart_tx_count == MECHSIM_UART_TX_BYTES);
    REQUIRE(out.event_count == 24U - MECHSIM_EVENT_LOG_COUNT);
    out = plant.tick();
    REQUIRE(out.uart_tx_count == 12U * 11U - 2U * MECHSIM_UART_TX_BYTES);
    REQUIRE(out.event_count == 0U);
    out = plant.tick();
    REQUIRE(out.uart_tx_count == 0U);
    REQUIRE(out.event_count == 0U);
    REQUIRE(plant.take_lines() == std::vector<std::string>(12U, "status 1 4"));
    const std::vector<Event> events = plant.take_events();
    REQUIRE(events.size() == 24U);
    for (std::size_t index = 0U; index < events.size(); index = index + 2U) {
        REQUIRE(events[index].code == GANTRY_EVENT_COMMAND);
        REQUIRE(events[index + 1U].code == GANTRY_EVENT_REFUSED);
    }
}

TEST_CASE("a flood that overflows the event queue latches a fault", "[FW-030]")
{
    Plant plant(800, 800);
    std::string burst;
    for (int index = 0; index < 32; ++index) {
        burst += "x\n";
    }
    mechsim_board_outputs out = plant.tick(burst);
    REQUIRE(out.state_code == GANTRY_STATE_NOT_HOMED);
    out = plant.tick(burst);
    REQUIRE(out.state_code == GANTRY_STATE_FAULT);
    REQUIRE(out.fault_code == GANTRY_FAULT_EVENT_OVERFLOW);
    REQUIRE(out.digital_out[MECHSIM_CH_WATCHDOG_HEARTBEAT] == MECHSIM_LEVEL_LOW);
}

TEST_CASE("every transition is logged in order across a full session", "[FW-030][CTL-018]")
{
    Plant plant(80, 80);
    plant.tick("home\n");
    plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000);
    plant.tick("target 0.01 0.01\n");
    plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000);
    std::vector<std::uint16_t> transitions;
    for (const Event& event : plant.take_events()) {
        if (event.code == GANTRY_EVENT_STATE) {
            transitions.push_back(event.arg);
        }
    }
    REQUIRE(transitions == std::vector<std::uint16_t>{transition(GANTRY_STATE_NOT_HOMED, GANTRY_STATE_HOMING),
                               transition(GANTRY_STATE_HOMING, GANTRY_STATE_READY),
                               transition(GANTRY_STATE_READY, GANTRY_STATE_MOVING),
                               transition(GANTRY_STATE_MOVING, GANTRY_STATE_READY)});
}

// ===== Heartbeat, status, outputs ====================================================================================

TEST_CASE("heartbeat toggles every tick outside FAULT and holds low in FAULT", "[FW-011]")
{
    Plant plant(160, 160);
    std::uint8_t expected = MECHSIM_LEVEL_HIGH;
    for (int tick = 0; tick < 400; ++tick) {
        std::string rx;
        if (tick == 3) {
            rx = "home\n";
        }
        if (tick == 300) {
            rx = "target 0.01 0\n";
        }
        const mechsim_board_outputs out = plant.tick(rx);
        REQUIRE(out.digital_out[MECHSIM_CH_WATCHDOG_HEARTBEAT] == expected);
        if (expected == MECHSIM_LEVEL_HIGH) {
            expected = MECHSIM_LEVEL_LOW;
        } else {
            expected = MECHSIM_LEVEL_HIGH;
        }
    }
    REQUIRE(plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_READY); }, 100000) > 0);
    plant.switches_work = false;
    plant.tick("home\n");
    REQUIRE(plant.last.state_code == GANTRY_STATE_HOMING);
    plant.run_until([](const mechsim_board_outputs& o) { return is_state(o, GANTRY_STATE_FAULT); }, 200000);
    for (int tick = 0; tick < 10; ++tick) {
        REQUIRE(plant.tick().digital_out[MECHSIM_CH_WATCHDOG_HEARTBEAT] == MECHSIM_LEVEL_LOW);
    }
}

TEST_CASE("current_position status integrates the per-tick encoder deltas", "[FW-029][CTL-004]")
{
    Plant plant(0, 0);
    plant.home();
    plant.extra_counts[0] = 600;
    plant.extra_counts[1] = -1;
    plant.run_until([](const mechsim_board_outputs& o) { return o.uart_tx_count > 0U; }, GANTRY_STATUS_PERIOD_TICKS);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"current_position 0.010000 -0.000016"});
    plant.extra_counts[0] = -600;
    plant.extra_counts[1] = 60001;
    plant.run_until([](const mechsim_board_outputs& o) { return o.uart_tx_count > 0U; }, GANTRY_STATUS_PERIOD_TICKS);
    REQUIRE(plant.take_lines() == std::vector<std::string>{"current_position 0.000000 1.000000"});
}

TEST_CASE("every output field is written each tick", "[FW-010][CTL-003]")
{
    mechsim_app_init();
    mechsim_board_inputs inputs{};
    mechsim_board_outputs outputs{};
    std::memset(&outputs, 0xAA, sizeof(outputs));
    mechsim_app_tick(&inputs, &outputs);
    for (std::uint32_t axis = 0U; axis < MECHSIM_AXIS_COUNT; ++axis) {
        REQUIRE(outputs.step_count[axis] == 0U);
        REQUIRE(outputs.direction[axis] == MECHSIM_LEVEL_LOW);
        REQUIRE(outputs.enable[axis] == MECHSIM_LEVEL_LOW);
    }
    for (std::uint32_t pin = 0U; pin < MECHSIM_DIGITAL_OUT_COUNT; ++pin) {
        if (pin == MECHSIM_CH_WATCHDOG_HEARTBEAT) {
            REQUIRE(outputs.digital_out[pin] == MECHSIM_LEVEL_HIGH);
        } else if ((pin == MECHSIM_CH_X_DRIVER_DIR) || (pin == MECHSIM_CH_Y_DRIVER_DIR)) {
            REQUIRE(outputs.digital_out[pin] == GANTRY_DIR_NEGATIVE_LEVEL);
        } else if ((pin == MECHSIM_CH_X_DRIVER_ENABLE) || (pin == MECHSIM_CH_Y_DRIVER_ENABLE)) {
            REQUIRE(outputs.digital_out[pin] == GANTRY_ENABLE_INACTIVE_LEVEL);
        } else if (pin == MECHSIM_CH_LIFT_DRIVER_ENABLE) {
            REQUIRE(outputs.digital_out[pin] == GANTRY_ENABLE_ACTIVE_LEVEL);
        } else {
            REQUIRE(outputs.digital_out[pin] == MECHSIM_LEVEL_LOW);
        }
    }
    for (std::uint32_t index = 0U; index < MECHSIM_UART_TX_BYTES; ++index) {
        REQUIRE(outputs.uart_tx[index] == 0U);
    }
    REQUIRE(outputs.uart_tx_count == 0U);
    REQUIRE(outputs.event_count == 0U);
    REQUIRE(outputs.state_code == GANTRY_STATE_NOT_HOMED);
    REQUIRE(outputs.fault_code == GANTRY_FAULT_NONE);
}
