#include <catch2/catch_test_macros.hpp>

#include "mechsim/engine/version.hpp"

TEST_CASE("core version string is the scaffolding version", "[smoke]") {
    REQUIRE(mechsim::engine::coreVersion() == "0.0.0");
}
