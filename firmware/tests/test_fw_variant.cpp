#include <catch2/catch_test_macros.hpp>

#include <cstdint>

extern "C" {
#include "mechsim_board.h"
}

TEST_CASE("a design-supplied tick rate reaches the header macro and the firmware", "[FW-010][CTL-002]")
{
    REQUIRE(MECHSIM_APP_TICK_HZ == 250U);
    REQUIRE(mechsim_app_tick_hz() == 250U);
}

TEST_CASE("a ready count of one reaches READY on the first tick", "[FW-010][CTL-002][CTL-018]")
{
    mechsim_app_init();
    const mechsim_board_inputs inputs{};
    mechsim_board_outputs outputs{};
    mechsim_app_tick(&inputs, &outputs);
    REQUIRE(outputs.state_code == 2U);
    mechsim_app_tick(&inputs, &outputs);
    REQUIRE(outputs.state_code == 2U);
}
