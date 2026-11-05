#include "mechsim/signals/operator_link.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "mechsim/signals/signal_name.hpp"

namespace mechsim::signals {

namespace {

constexpr std::int64_t bits_per_byte = 10;  // 8N1: start, 8 data, stop
constexpr std::size_t number_chars = 32;

std::vector<std::string_view> split_spaces(std::string_view line) {
    std::vector<std::string_view> parts;
    std::size_t begin = 0;
    while (true) {
        const std::size_t space = line.find(' ', begin);
        if (space == std::string_view::npos) {
            parts.push_back(line.substr(begin));
            return parts;
        }
        parts.push_back(line.substr(begin, space - begin));
        begin = space + 1;
    }
}

}  // namespace

AsciiLineCodec::AsciiLineCodec(std::vector<MessageSpec> messages) : messages_(std::move(messages)) {
    for (std::size_t i = 0; i < messages_.size(); ++i) {
        const MessageSpec& m = messages_[i];
        if (!is_instance_name(m.name)) {
            throw SignalConfigError("operator message name '" + m.name + "' is not an identifier");
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (messages_[j].name == m.name) {
                throw SignalConfigError("operator message '" + m.name + "' is repeated");
            }
        }
        for (const std::string& field : m.fields) {
            if (!is_instance_name(field)) {
                throw SignalConfigError("operator message field '" + field + "' is not an identifier");
            }
        }
    }
}

const MessageSpec* AsciiLineCodec::find(std::string_view name) const {
    const auto it =
        std::find_if(messages_.begin(), messages_.end(), [name](const MessageSpec& m) { return m.name == name; });
    if (it == messages_.end()) {
        return nullptr;
    }
    return &*it;
}

const MessageSpec& AsciiLineCodec::spec(std::string_view name) const {
    const MessageSpec* found = find(name);
    if (found == nullptr) {
        throw SignalRunError("unknown operator message '" + std::string(name) + "'");
    }
    return *found;
}

std::string AsciiLineCodec::encode(const OperatorMessage& message) const {
    const MessageSpec& s = spec(message.name);
    if (s.direction != MessageDirection::to_controller) {
        throw SignalRunError("operator message '" + message.name + "' travels from the controller");
    }
    if (message.fields.size() != s.fields.size()) {
        throw SignalRunError("operator message '" + message.name + "' takes " + std::to_string(s.fields.size()) +
                             " fields");
    }
    std::string line = message.name;
    for (const double value : message.fields) {
        if (!std::isfinite(value)) {
            throw SignalRunError("operator message '" + message.name + "' has a non-finite field");
        }
        std::array<char, number_chars> buffer{};
        const std::to_chars_result result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
        line += ' ';
        line.append(buffer.data(), result.ptr);
    }
    line += '\n';
    return line;
}

OperatorMessage AsciiLineCodec::decode(std::string_view line) const {
    const std::vector<std::string_view> parts = split_spaces(line);
    const MessageSpec* s = find(parts.front());
    if (s == nullptr) {
        throw SignalRunError("status line '" + std::string(line) + "' names no known message");
    }
    if (s->direction != MessageDirection::from_controller) {
        throw SignalRunError("status line '" + std::string(line) + "' names an operator command");
    }
    if (parts.size() - 1 != s->fields.size()) {
        throw SignalRunError("status line '" + std::string(line) + "' has the wrong field count");
    }
    OperatorMessage message{s->name, {}};
    for (std::size_t i = 1; i < parts.size(); ++i) {
        const std::string_view text = parts[i];
        double value = 0.0;
        const std::from_chars_result result = std::from_chars(text.data(), text.data() + text.size(), value);
        if (text.empty() || result.ec != std::errc() || result.ptr != text.data() + text.size() ||
            !std::isfinite(value)) {
            throw SignalRunError("status line '" + std::string(line) + "' has a malformed field");
        }
        message.fields.push_back(value);
    }
    return message;
}

UartTransport::UartTransport(std::uint32_t baud) : baud_(baud) {
    if (baud_ == 0) {
        throw SignalConfigError("UART baud rate must be positive");
    }
}

void UartTransport::start(std::int64_t rate_numerator, std::int64_t rate_denominator) {
    bits_per_tick_numerator_ = static_cast<std::int64_t>(baud_) * rate_denominator;
    bits_per_byte_denominator_ = bits_per_byte * rate_numerator;
    credit_ = 0;
}

void UartTransport::send(std::string_view bytes) {
    for (const char c : bytes) {
        outgoing_.push_back(static_cast<std::uint8_t>(c));
    }
}

std::size_t UartTransport::fill(std::span<std::uint8_t, buffer_bytes> out) {
    credit_ += bits_per_tick_numerator_;
    const auto paced = static_cast<std::size_t>(credit_ / bits_per_byte_denominator_);
    const std::size_t count = std::min({paced, buffer_bytes, outgoing_.size()});
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = outgoing_.front();
        outgoing_.pop_front();
    }
    credit_ -= static_cast<std::int64_t>(count) * bits_per_byte_denominator_;
    // An idle line banks at most one byte of credit, so a message queued after a pause still leaves at line rate.
    if (outgoing_.empty()) {
        credit_ = std::min(credit_, bits_per_byte_denominator_);
    }
    return count;
}

void UartTransport::receive(std::span<const std::uint8_t> bytes) {
    if (bytes.size() > buffer_bytes) {
        throw SignalRunError("controller sent " + std::to_string(bytes.size()) + " UART bytes in one tick, over 64");
    }
    for (const std::uint8_t byte : bytes) {
        if (byte == '\n') {
            lines_.push_back(std::move(partial_));
            partial_.clear();
        } else {
            partial_ += static_cast<char>(byte);
        }
    }
}

std::vector<std::string> UartTransport::take_lines() {
    std::vector<std::string> lines = std::move(lines_);
    lines_.clear();
    return lines;
}

std::size_t UartTransport::queued() const noexcept {
    return outgoing_.size();
}

OperatorLink::OperatorLink(std::string instance, std::unique_ptr<LineCodec> codec, UartTransport transport)
    : BuiltinComponent(std::move(instance), {"uart_tx"}, {"uart_rx"}, {}), codec_(std::move(codec)),
      transport_(std::move(transport)) {
    if (codec_ == nullptr) {
        throw std::invalid_argument("operator link needs a codec");
    }
}

void OperatorLink::on_start() {
    rx_count_ = 0;
    const kernel::Rate base = clock().base_rate;
    transport_.start(base.numerator(), base.denominator() * clock().period.count());
}

void OperatorLink::sample(const PhysicalState& /*state*/) {}

void OperatorLink::latch_outputs() {
    rx_count_ = transport_.fill(rx_);
    set_output(0, static_cast<double>(rx_count_));
}

void OperatorLink::post(const OperatorMessage& message) {
    transport_.send(codec_->encode(message));
    posted_.push_back(message);
}

std::vector<OperatorMessage> OperatorLink::take_posted() {
    std::vector<OperatorMessage> posted = std::move(posted_);
    posted_.clear();
    return posted;
}

void OperatorLink::accept_controller_bytes(std::span<const std::uint8_t> bytes) {
    transport_.receive(bytes);
    for (const std::string& line : transport_.take_lines()) {
        OperatorMessage message = codec_->decode(line);
        std::string label = codec_->spec(message.name).label;
        status_.push_back(StatusMessage{std::move(label), std::move(message)});
    }
}

std::span<const std::uint8_t> OperatorLink::controller_rx() const noexcept {
    return {rx_.data(), rx_count_};
}

std::vector<StatusMessage> OperatorLink::take_status() {
    std::vector<StatusMessage> status = std::move(status_);
    status_.clear();
    return status;
}

const LineCodec& OperatorLink::codec() const noexcept {
    return *codec_;
}

}  // namespace mechsim::signals
