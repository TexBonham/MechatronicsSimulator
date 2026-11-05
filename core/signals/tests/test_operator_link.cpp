#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "mechsim/signals/operator_link.hpp"
#include "signal_test_support.hpp"

using mechsim::signals::AsciiLineCodec;
using mechsim::signals::LineCodec;
using mechsim::signals::MessageDirection;
using mechsim::signals::MessageSpec;
using mechsim::signals::OperatorLink;
using mechsim::signals::OperatorMessage;
using mechsim::signals::SignalConfigError;
using mechsim::signals::SignalRunError;
using mechsim::signals::StatusMessage;
using mechsim::signals::UartTransport;
using signal_test::state_at;
using signal_test::Stepper;

namespace {

// Test data copied from the reference design's operator_messages (designs/gantry/design.yaml); the codec itself
// holds no message names.
std::vector<MessageSpec> gantry_messages(bool mirrored = false) {
    MessageDirection to = MessageDirection::to_controller;
    MessageDirection from = MessageDirection::from_controller;
    if (mirrored) {
        to = MessageDirection::from_controller;
        from = MessageDirection::to_controller;
    }
    return {MessageSpec{"target", "target (x, y)", {"x", "y"}, to}, MessageSpec{"home", "home", {}, to},
            MessageSpec{"current_position", "current position", {"x", "y"}, from},
            MessageSpec{"ready_fault", "ready / fault", {}, from}};
}

std::span<const std::uint8_t> bytes_of(std::string_view text) {
    return {reinterpret_cast<const std::uint8_t*>(text.data()), text.size()};  // NOLINT(*-reinterpret-cast)
}

std::string text_of(std::span<const std::uint8_t> bytes) {
    return {bytes.begin(), bytes.end()};
}

OperatorLink make_link(std::uint32_t baud) {
    return {"operator", std::make_unique<AsciiLineCodec>(gantry_messages()), UartTransport(baud)};
}

}  // namespace

TEST_CASE("Codec and UART transport are separate classes behind one link", "[SIG-025]") {
    static_assert(std::is_abstract_v<LineCodec>);
    static_assert(std::is_base_of_v<LineCodec, AsciiLineCodec>);
    static_assert(!std::is_base_of_v<LineCodec, UartTransport>);
    CHECK_THROWS_AS(OperatorLink("operator", nullptr, UartTransport(115200)), std::invalid_argument);
    CHECK_THROWS_AS(UartTransport(0), SignalConfigError);
}

TEST_CASE("Every reference-design message round-trips through the ASCII codec with its label verbatim",
          "[SIG-026][SIG-027]") {
    // The mirrored codec plays the far end: it decodes what the link encodes and encodes what the link decodes.
    const AsciiLineCodec link(gantry_messages());
    const AsciiLineCodec far_end(gantry_messages(true));
    const std::vector<OperatorMessage> commands{{"target", {0.5, 0.25}}, {"home", {}}};
    CHECK(link.encode(commands[0]) == "target 0.5 0.25\n");
    CHECK(link.encode(commands[1]) == "home\n");
    for (const OperatorMessage& command : commands) {
        std::string line = link.encode(command);
        line.pop_back();
        const OperatorMessage decoded = far_end.decode(line);
        CHECK(decoded.name == command.name);
        CHECK(decoded.fields == command.fields);
    }
    // Shortest round-trip formatting: 0.1 and -1.0e-7 come back bit for bit.
    const std::vector<OperatorMessage> status{{"current_position", {0.1, -1.0e-7}}, {"ready_fault", {}}};
    for (const OperatorMessage& message : status) {
        std::string line = far_end.encode(message);
        line.pop_back();
        const OperatorMessage decoded = link.decode(line);
        CHECK(decoded.name == message.name);
        CHECK(decoded.fields == message.fields);
    }
    CHECK(link.spec("target").label == "target (x, y)");
    CHECK(link.spec("current_position").label == "current position");
    CHECK(link.spec("ready_fault").label == "ready / fault");
}

TEST_CASE("The codec rejects malformed lines, wrong directions and bad message sets", "[SIG-026][SIG-027]") {
    const AsciiLineCodec link(gantry_messages());
    for (const std::string_view bad : {"", "unknown 1", "current_position 1", "current_position 1 2 3",
                                       "current_position 1  2", "current_position 1 2 ", "current_position 1 x",
                                       "current_position 1 2\r", "current_position nan 2", "target 1 2", "home"}) {
        CHECK_THROWS_AS(link.decode(bad), SignalRunError);
    }
    CHECK_THROWS_AS(link.encode(OperatorMessage{"current_position", {1.0, 2.0}}), SignalRunError);
    CHECK_THROWS_AS(link.encode(OperatorMessage{"target", {1.0}}), SignalRunError);
    CHECK_THROWS_AS(link.encode(OperatorMessage{"target", {1.0, std::numeric_limits<double>::infinity()}}),
                    SignalRunError);
    CHECK_THROWS_AS(link.encode(OperatorMessage{"jog", {}}), SignalRunError);
    CHECK_THROWS_AS(AsciiLineCodec({MessageSpec{"home", "home", {}, MessageDirection::to_controller},
                                    MessageSpec{"home", "again", {}, MessageDirection::to_controller}}),
                    SignalConfigError);
    CHECK_THROWS_AS(AsciiLineCodec({MessageSpec{"go home", "go home", {}, MessageDirection::to_controller}}),
                    SignalConfigError);
    CHECK_THROWS_AS(AsciiLineCodec({MessageSpec{"target", "t", {"x y"}, MessageDirection::to_controller}}),
                    SignalConfigError);
}

TEST_CASE("Scripted commands leave at the baud rate, a line split across ticks", "[SIG-026]") {
    // 115200 baud, 8N1 = 10 bits per byte, 1 kHz: 115.2 bits per tick. Tick 1 has 115.2 bits, floor(11.52) = 11
    // bytes, leaving 5.2 bits; tick 2 has 120.4 bits = 12 bytes of credit for the remaining 5 of the 16-byte line.
    OperatorLink link = make_link(115200);
    Stepper run(link);
    link.post(OperatorMessage{"target", {0.5, 0.25}});
    CHECK(run.step(state_at(0.0, 0.0), {0.0})[0] == 11.0);
    std::string received = text_of(link.controller_rx());
    CHECK(received == "target 0.5 ");
    CHECK(run.step(state_at(0.0, 0.0), {0.0})[0] == 5.0);
    received += text_of(link.controller_rx());
    CHECK(received == "target 0.5 0.25\n");
    CHECK(run.step(state_at(0.0, 0.0), {0.0})[0] == 0.0);
    const std::vector<OperatorMessage> posted = link.take_posted();
    CHECK(posted.size() == 1);
}

TEST_CASE("At most 64 bytes reach the controller per tick, the rest carried", "[SIG-026]") {
    // 10 Mbaud at 1 kHz paces 1000 bytes per tick, so only the 64-byte pin-input buffer limits: 100 queued bytes
    // leave as 64 then 36.
    OperatorLink link = make_link(10000000);
    Stepper run(link);
    for (int k = 0; k < 20; ++k) {
        link.post(OperatorMessage{"home", {}});
    }
    CHECK(run.step(state_at(0.0, 0.0), {0.0})[0] == 64.0);
    CHECK(run.step(state_at(0.0, 0.0), {0.0})[0] == 36.0);
}

TEST_CASE("A status line split across two ticks is parsed once complete; overflow is rejected", "[SIG-026]") {
    OperatorLink link = make_link(115200);
    const Stepper run(link);
    link.accept_controller_bytes(bytes_of("current_position 0.1"));
    CHECK(link.take_status().empty());
    link.accept_controller_bytes(bytes_of(" 0.2\nready_fault\nready"));
    const std::vector<StatusMessage> status = link.take_status();
    REQUIRE(status.size() == 2);
    CHECK(status[0].label == "current position");
    CHECK(status[0].message.fields == std::vector<double>{0.1, 0.2});
    CHECK(status[1].label == "ready / fault");
    link.accept_controller_bytes(bytes_of("_fault\n"));
    CHECK(link.take_status().size() == 1);

    const std::array<std::uint8_t, 65> flood{};
    CHECK_THROWS_AS(link.accept_controller_bytes(flood), SignalRunError);
    const std::array<std::uint8_t, 64> full{};
    CHECK_NOTHROW(link.accept_controller_bytes(full));
}
