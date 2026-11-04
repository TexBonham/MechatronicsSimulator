#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/signals/pin_binding.hpp"

namespace mechsim::controller {

// One slot of the pin contract per assignable field array of PinInputs and PinOutputs (board interface, brief C6).
// Direction and enable are not slots: they are derived per axis from digital_out channels (AxisChannels).
enum class PinSlot : std::uint8_t {
    digital_in,
    counter_delta,
    adc_sample,
    uart_rx,
    step_count,
    digital_out,
    uart_tx,
};

// Channels a slot holds: the array length, or 1 for the single UART byte stream in each direction.
[[nodiscard]] std::size_t slot_capacity(PinSlot slot) noexcept;
// Whether a pin bound in this peripheral mode may feed this slot (CTL-006).
[[nodiscard]] bool slot_accepts(PinSlot slot, signals::PeripheralMode mode) noexcept;

// The builder's assignment of one bound signal to one channel of one slot.
struct ChannelAssignment {
    std::string signal;
    PinSlot slot = PinSlot::digital_in;
    std::size_t channel = 0;
};

// The digital_out channels that carry one stepper axis's direction and driver enable.
struct AxisChannels {
    std::size_t dir_channel = 0;
    std::size_t enable_channel = 0;
    friend bool operator==(const AxisChannels&, const AxisChannels&) = default;
};

// Signal name to channel per slot, from the builder (CTL-006). The signal layer places each bound signal's value in
// the slot and channel listed here. Entries keep insertion order (determinism).
class ChannelMap {
public:
    // Throws ControllerConfigError on an empty signal name, a channel beyond the slot, a repeated signal or a
    // repeated slot and channel.
    void add(ChannelAssignment assignment);

    [[nodiscard]] std::optional<ChannelAssignment> find(std::string_view signal) const;
    [[nodiscard]] std::span<const ChannelAssignment> entries() const noexcept;

    // Throws ControllerConfigError unless every bound signal has an assignment whose slot accepts its mode and every
    // assignment names a bound signal.
    void check_bindings(const signals::PinBindingTable& bindings) const;

    // Throws ControllerConfigError on an axis beyond stepper_channels, an axis assigned twice, equal direction and
    // enable channels, or a channel no digital_out assignment holds.
    void assign_axis(std::size_t axis, AxisChannels channels);
    [[nodiscard]] std::optional<AxisChannels> axis(std::size_t axis) const;

private:
    std::vector<ChannelAssignment> entries_;
    std::array<std::optional<AxisChannels>, stepper_channels> axes_{};
};

// Fills direction and enable per axis from the mapped digital_out channels, the single source (fw.md ruling);
// an unmapped axis reads low.
void derive_axis_levels(const ChannelMap& map, PinOutputs& outputs);

}  // namespace mechsim::controller
