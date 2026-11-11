// The gantry firmware built with the lift at the TR8x8 screw's own scale (8 mm/rev: 400000 steps/m, 300000 counts/m)
// while X and Y keep the belts' 80000 and 60000: every per-axis conversion is exercised with unequal scales.
#include "test_gantry_plant.cpp"

#include <algorithm>

using gantry_test::Plant;

namespace {

bool ready_state(const mechsim_board_outputs& out)
{
    return out.state_code == GANTRY_STATE_READY;
}

void run_until_fault(Plant& plant)
{
    REQUIRE(plant.run_until([](const mechsim_board_outputs& out) { return out.state_code == GANTRY_STATE_FAULT; },
                100000)
        > 0);
}

}  // namespace

TEST_CASE("Per-axis step scale: a target and a lift of one length issue each axis's own steps", "[FW-018][FW-029]")
{
    // 0.1 m is 8000 steps on X and Y at 80000 steps/m and 40000 on the lift at 400000 steps/m.
    STATIC_REQUIRE(GANTRY_STEPS_PER_M_LIFT == 400000);
    Plant plant(0, 0, 0);
    plant.home();
    plant.tick("target 0.1 0.1\n");
    REQUIRE(plant.run_until(ready_state, 100000) > 0);
    plant.tick("lift 0.1\n");
    REQUIRE(plant.run_until(ready_state, 100000) > 0);
    plant.tick();
    REQUIRE(plant.position[0] == 8000);
    REQUIRE(plant.position[1] == 8000);
    REQUIRE(plant.position[2] == 40000);
    REQUIRE(plant.take_lines().back() == "status 3 0");
}

TEST_CASE("Per-axis step scale: the lift's travel and speed are in its own steps", "[FW-018][FW-022]")
{
    // 0.5 m of travel is 200000 steps; 0.2 m/s is 80 steps per tick at 1 kHz; a lift past the travel is refused.
    Plant plant(0, 0, 0);
    plant.home();
    std::uint32_t peak = 0U;
    mechsim_board_outputs out = plant.tick("lift 0.5\n");
    while (out.state_code == GANTRY_STATE_MOVING) {
        peak = std::max(peak, out.step_count[MECHSIM_CH_LIFT_DRIVER_STEP]);
        out = plant.tick();
    }
    plant.tick();
    REQUIRE(peak == 80U);
    REQUIRE(plant.position[2] == 200000);
    out = plant.tick("lift 0.500001\n");
    REQUIRE(out.fault_code == GANTRY_FAULT_OUTSIDE_WORKSPACE);
}

TEST_CASE("G10: the mismatch threshold is 2 mm on every axis at its own count scale", "[FW-020]")
{
    // 2 mm at 60000 counts/m on X and Y is 120 counts; at 300000 counts/m on the lift (2400 counts per 8 mm turn) it is
    // 600 counts. A 121-count error on the lift is far inside 2 mm there and is tolerated.
    STATIC_REQUIRE(gantry_test::mismatch_counts[0] == 120);
    STATIC_REQUIRE(gantry_test::mismatch_counts[1] == 120);
    STATIC_REQUIRE(gantry_test::mismatch_counts[2] == 600);

    Plant small(0, 0, 0);
    small.home();
    small.tick("lift 0.2\n");
    small.extra_counts[2] = gantry_test::mismatch_counts[0] + 1;
    REQUIRE(small.run_until(ready_state, 100000) > 0);
    REQUIRE(small.last.fault_code == GANTRY_FAULT_NONE);

    Plant tolerated(0, 0, 0);
    tolerated.home();
    tolerated.tick("lift 0.2\n");
    tolerated.extra_counts[2] = gantry_test::mismatch_counts[2];
    REQUIRE(tolerated.run_until(ready_state, 100000) > 0);
    REQUIRE(tolerated.last.fault_code == GANTRY_FAULT_NONE);

    Plant over(0, 0, 0);
    over.home();
    over.tick("lift 0.2\n");
    over.extra_counts[2] = gantry_test::mismatch_counts[2] + 1;
    run_until_fault(over);
    REQUIRE(over.last.fault_code == GANTRY_FAULT_ENCODER_MISMATCH);
}
