#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

enum class MessageDirection : std::uint8_t {
    to_controller,    // scripted operator command
    from_controller,  // status
};

// One message of the design's operator message set (SIG-027): wire name, label verbatim, field names (SI values).
struct MessageSpec {
    std::string name;
    std::string label;
    std::vector<std::string> fields;
    MessageDirection direction = MessageDirection::to_controller;
};

struct OperatorMessage {
    std::string name;
    std::vector<double> fields;
};

// Line codec interface (SIG-025): one message per line, LF terminated. A binary codec is a later class behind it.
class LineCodec {
public:
    LineCodec() = default;
    virtual ~LineCodec() = default;
    LineCodec(const LineCodec&) = delete;
    LineCodec& operator=(const LineCodec&) = delete;
    LineCodec(LineCodec&&) = delete;
    LineCodec& operator=(LineCodec&&) = delete;

    // Throws SignalRunError on an unknown message, a wrong direction, a wrong field count or a non-finite value.
    [[nodiscard]] virtual std::string encode(const OperatorMessage& message) const = 0;
    // line excludes the LF. Throws SignalRunError on a malformed line, an unknown or wrong-direction name.
    [[nodiscard]] virtual OperatorMessage decode(std::string_view line) const = 0;
    [[nodiscard]] virtual const MessageSpec& spec(std::string_view name) const = 0;
};

// ASCII grammar (owner ruling 2026-11-04):
//   line   = name *(SP value) LF
//   name   = the message's design name (identifier)
//   value  = decimal number in SI, shortest round-trip form; one per field, in field order
//   nothing else: no CR, no repeated or trailing space, no empty line
class AsciiLineCodec final : public LineCodec {
public:
    // Throws SignalConfigError on a repeated or non-identifier name or field.
    explicit AsciiLineCodec(std::vector<MessageSpec> messages);

    [[nodiscard]] std::string encode(const OperatorMessage& message) const override;
    [[nodiscard]] OperatorMessage decode(std::string_view line) const override;
    [[nodiscard]] const MessageSpec& spec(std::string_view name) const override;

private:
    [[nodiscard]] const MessageSpec* find(std::string_view name) const;

    std::vector<MessageSpec> messages_;
};

// UART transport (SIG-025): bytes to the controller paced by the baud rate (8N1, 10 bits per byte) and capped at
// the 64-byte pin-input buffer, the rest carried to later ticks; bytes from the controller split into lines, a
// partial line carried across ticks.
class UartTransport {
public:
    static constexpr std::size_t buffer_bytes = 64;

    // Throws SignalConfigError on a zero baud rate.
    explicit UartTransport(std::uint32_t baud);

    // Rate of the ticks that call fill(), as rational hertz numerator / denominator.
    void start(std::int64_t rate_numerator, std::int64_t rate_denominator);
    void send(std::string_view bytes);
    // Moves this tick's bytes into out; returns the count.
    std::size_t fill(std::span<std::uint8_t, buffer_bytes> out);
    // Throws SignalRunError above 64 bytes in one tick (never truncated).
    void receive(std::span<const std::uint8_t> bytes);
    // Complete lines received so far, LF removed, in order.
    [[nodiscard]] std::vector<std::string> take_lines();
    [[nodiscard]] std::size_t queued() const noexcept;

private:
    std::uint32_t baud_;
    std::int64_t bits_per_tick_numerator_ = 0;
    std::int64_t bits_per_byte_denominator_ = 1;
    std::int64_t credit_ = 0;
    std::deque<std::uint8_t> outgoing_;
    std::string partial_;
    std::vector<std::string> lines_;
};

// A received status message, label carried for the recorder (REC-009).
struct StatusMessage {
    std::string label;
    OperatorMessage message;
};

// Outputs `rx` (bytes delivered to the controller this tick); input `tx` (bytes the controller sent this tick).
class OperatorLink final : public BuiltinComponent {
public:
    // Throws std::invalid_argument on a null codec.
    OperatorLink(std::string instance, std::unique_ptr<LineCodec> codec, UartTransport transport);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;

    // Encodes a scripted operator message and queues its bytes.
    void post(const OperatorMessage& message);
    // Bytes the controller transmitted this tick; decodes every completed line.
    void accept_controller_bytes(std::span<const std::uint8_t> bytes);
    [[nodiscard]] std::span<const std::uint8_t> controller_rx() const noexcept;
    [[nodiscard]] std::vector<StatusMessage> take_status();
    // Messages posted since the last call, for the recorder.
    [[nodiscard]] std::vector<OperatorMessage> take_posted();
    [[nodiscard]] const LineCodec& codec() const noexcept;

private:
    void on_start() override;

    std::unique_ptr<LineCodec> codec_;
    UartTransport transport_;
    std::array<std::uint8_t, UartTransport::buffer_bytes> rx_{};
    std::size_t rx_count_ = 0;
    std::vector<StatusMessage> status_;
    std::vector<OperatorMessage> posted_;
};

}  // namespace mechsim::signals
