#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

// The pin tables and rail state shared by the signal layer and the controller runtime (CTL-014, SIG-024): plain
// data, no physics type, so the controller side of invariant 1 is checkable by include.
namespace mechsim::pins {

// Fixed contract sizes, identical to the board interface C header (board_interface/, brief C6); not overridable.
inline constexpr std::size_t digital_in_channels = 32;
inline constexpr std::size_t counter_channels = 8;
inline constexpr std::size_t adc_channels = 16;
inline constexpr std::size_t uart_buffer_bytes = 64;
inline constexpr std::size_t stepper_channels = 8;
inline constexpr std::size_t digital_out_channels = 32;
inline constexpr std::size_t event_log_entries = 16;

// Polled semantics (CTL-004, ADR 0011 C3): levels latched at the sync point, counter deltas accumulated during the
// tick, bytes that arrived during the tick, nothing interrupts the tick. No rail field: the rail reaches the
// runtime through reset() and brownout() (CTL-013).
struct PinInputs {
    std::array<std::uint8_t, digital_in_channels> digital_in{};  // 0 or 1
    std::array<std::int32_t, counter_channels> counter_delta{};  // counts accumulated during the tick
    std::array<std::uint16_t, adc_channels> adc_sample{};
    std::array<std::uint8_t, uart_buffer_bytes> uart_rx{};
    std::uint16_t uart_rx_count = 0;
};

struct EventLogEntry {
    std::uint8_t code = 0;
    std::uint16_t arg = 0;
};

struct PinOutputs {
    std::array<std::uint32_t, stepper_channels> step_count{};  // pulses this tick
    std::array<std::uint8_t, stepper_channels> direction{};
    std::array<std::uint8_t, stepper_channels> enable{};
    std::array<std::uint8_t, digital_out_channels> digital_out{};
    std::array<std::uint8_t, uart_buffer_bytes> uart_tx{};
    std::uint16_t uart_tx_count = 0;
    std::uint8_t state_code = 0;  // CTL-018 controller state
    std::uint8_t fault_code = 0;  // CTL-018 fault
    std::array<EventLogEntry, event_log_entries> event_log{};  // CTL-018 fault log
    std::uint8_t event_count = 0;
};

// State of one power rail each tick (SIG-015, SIG-022, REC-006). The controller runtime receives this and pin
// inputs, nothing else (CTL-014).
struct PowerRail {
    std::string name;
    double voltage = 0.0;  // V
    bool on = false;
};

// Payload kinds in the kernel's coupling table (KER-008).
inline constexpr const char* pin_inputs_kind = "PinInputs";
inline constexpr const char* pin_outputs_kind = "PinOutputs";
inline constexpr const char* power_rails_kind = "PowerRails";

}  // namespace mechsim::pins
