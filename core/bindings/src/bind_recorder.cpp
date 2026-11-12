#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "bindings.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/tick_record.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

using kernel::RecorderSink;
using kernel::SubscriptionId;
using kernel::TickCallback;
using kernel::TickRecord;

class PyRecorderSink : public RecorderSink, public py::trampoline_self_life_support {
public:
    void write_row(const TickRecord& row) override { call_override<void>(this, "write_row", TickRecord(row)); }

    [[nodiscard]] SubscriptionId subscribe(TickCallback callback) override {
        return call_override<SubscriptionId>(this, "subscribe", std::move(callback));
    }

    void unsubscribe(SubscriptionId id) override { call_override<void>(this, "unsubscribe", id); }
};

// Buffers every row in memory for Python to drain, and forwards rows to subscribers in subscription order inside
// write_row (REC-019). Rows are copied out, so no consumer can change the run (REC-020).
class BufferedRecorderSink final : public RecorderSink {
public:
    void write_row(const TickRecord& row) override {
        rows_.push_back(row);
        for (const auto& subscriber : subscribers_) {
            subscriber.second(row);
        }
    }

    [[nodiscard]] SubscriptionId subscribe(TickCallback callback) override {
        const SubscriptionId id(next_id_);
        ++next_id_;
        subscribers_.emplace_back(id, std::move(callback));
        return id;
    }

    void unsubscribe(SubscriptionId id) override {
        for (auto it = subscribers_.begin(); it != subscribers_.end(); ++it) {
            if (it->first == id) {
                subscribers_.erase(it);
                return;
            }
        }
    }

    [[nodiscard]] std::size_t pending() const noexcept { return rows_.size(); }
    [[nodiscard]] std::size_t subscriber_count() const noexcept { return subscribers_.size(); }

    [[nodiscard]] std::vector<TickRecord> take() {
        std::vector<TickRecord> out;
        out.swap(rows_);
        return out;
    }

private:
    std::vector<TickRecord> rows_;
    std::uint64_t next_id_ = 1;
    std::vector<std::pair<SubscriptionId, TickCallback>> subscribers_;
};

template <std::size_t N>
py::list to_list(const std::array<double, N>& values) {
    py::list out;
    for (const double value : values) {
        out.append(value);
    }
    return out;
}

py::dict wrench_dict(const kernel::Wrench& wrench) {
    py::dict out;
    out["moment"] = to_list(wrench.moment);
    out["force"] = to_list(wrench.force);
    return out;
}

py::list series_list(const std::vector<kernel::SeriesSample>& samples) {
    py::list out;
    for (const kernel::SeriesSample& sample : samples) {
        py::dict entry;
        entry["name"] = sample.name;
        entry["unit"] = sample.unit;
        entry["value"] = sample.value;
        out.append(entry);
    }
    return out;
}

py::dict physics_dict(const kernel::PhysicsOutputs& physics) {
    py::dict out;
    out["steps"] = physics.steps;
    py::list bodies;
    for (const kernel::BodyState& body : physics.bodies) {
        py::dict entry;
        entry["position"] = to_list(body.position);
        entry["orientation"] = to_list(body.orientation);
        entry["angular_velocity"] = to_list(body.angular_velocity);
        entry["linear_velocity"] = to_list(body.linear_velocity);
        bodies.append(entry);
    }
    out["bodies"] = bodies;
    py::list joints;
    for (const kernel::JointForce& joint : physics.joints) {
        py::dict entry;
        entry["generalized_force"] = py::cast(joint.generalized_force);
        entry["reaction"] = wrench_dict(joint.reaction);
        joints.append(entry);
    }
    out["joints"] = joints;
    py::list constraints;
    for (const kernel::ConstraintForce& constraint : physics.constraints) {
        py::dict entry;
        entry["force"] = py::cast(constraint.force);
        entry["translational_error"] = constraint.translational_error;
        entry["rotational_error"] = constraint.rotational_error;
        constraints.append(entry);
    }
    out["constraints"] = constraints;
    py::list contacts;
    for (const kernel::ContactForce& contact : physics.contacts) {
        py::dict entry;
        entry["body_a"] = contact.body_a;
        entry["body_b"] = contact.body_b;
        entry["point"] = to_list(contact.point);
        entry["force"] = to_list(contact.force);
        contacts.append(entry);
    }
    out["contacts"] = contacts;
    py::list members;
    for (const kernel::MemberEndLoads& member : physics.members) {
        py::dict entry;
        entry["start"] = wrench_dict(member.start);
        entry["end"] = wrench_dict(member.end);
        members.append(entry);
    }
    out["members"] = members;
    py::list ropes;
    for (const kernel::RopeTension& rope : physics.ropes) {
        py::dict entry;
        entry["tension"] = rope.tension;
        entry["slack"] = rope.slack;
        ropes.append(entry);
    }
    out["ropes"] = ropes;
    py::list transmissions;
    for (const kernel::TransmissionState& transmission : physics.transmissions) {
        py::dict entry;
        entry["slip"] = transmission.slip;
        entry["slipping"] = transmission.slipping;
        entry["shaft_load_torque"] = transmission.shaft_load_torque;
        entry["force"] = transmission.force;
        entry["stretch"] = transmission.stretch;
        entry["power_flow"] = static_cast<int>(transmission.power_flow);
        transmissions.append(entry);
    }
    out["transmissions"] = transmissions;
    out["joint_positions"] = py::cast(physics.joint_positions);
    py::list drive_torques;
    for (const kernel::JointDriveTorque& drive : physics.joint_drive_torques) {
        py::dict entry;
        entry["joint"] = drive.joint;
        entry["torque"] = drive.torque;
        drive_torques.append(entry);
    }
    out["joint_drive_torques"] = drive_torques;
    py::list payloads;
    for (const kernel::PayloadState& payload : physics.payloads) {
        py::dict entry;
        entry["body"] = payload.body;
        std::int64_t carrier = -1;
        if (payload.carrier != kernel::kWorldBody) {
            carrier = static_cast<std::int64_t>(payload.carrier);
        }
        entry["carrier"] = carrier;
        entry["attached"] = payload.attached;
        entry["force"] = to_list(payload.force);
        payloads.append(entry);
    }
    out["payloads"] = payloads;
    return out;
}

py::dict row_dict(const TickRecord& row) {
    py::dict out;
    out["time"] = row.time.count();
    out["physics"] = physics_dict(row.physics);
    out["pin_signals"] = series_list(row.pin_signals);
    out["rails"] = series_list(row.rails);
    out["actuator_efforts"] = series_list(row.actuator_efforts);
    out["controller_state"] = row.controller_state;
    py::dict coupling;
    for (const kernel::CouplingSample& sample : row.coupling) {
        coupling[py::str(sample.name)] = coupling_to_python(sample.value);
    }
    out["coupling"] = coupling;
    py::list events;
    for (const kernel::RecordedEvent& event : row.events) {
        py::dict entry;
        entry["time"] = event.time.count();
        entry["source"] = event.source;
        entry["kind"] = event.kind;
        entry["detail"] = event.detail;
        events.append(entry);
    }
    out["events"] = events;
    return out;
}

py::list drain_dicts(BufferedRecorderSink& sink) {
    py::list out;
    for (const TickRecord& row : sink.take()) {
        out.append(row_dict(row));
    }
    return out;
}

using SeriesMember = std::vector<kernel::SeriesSample> TickRecord::*;

// One float array per series name, in name order; NaN where a row lacks the name.
py::dict series_arrays(const std::vector<TickRecord>& rows, SeriesMember member) {
    std::map<std::string, std::vector<double>> columns;
    for (std::size_t r = 0; r < rows.size(); ++r) {
        for (const kernel::SeriesSample& sample : rows[r].*member) {
            auto found = columns.find(sample.name);
            if (found == columns.end()) {
                found = columns.emplace(sample.name, std::vector<double>(rows.size(),
                                                                         std::numeric_limits<double>::quiet_NaN()))
                            .first;
            }
            found->second[r] = sample.value;
        }
    }
    py::dict out;
    for (const auto& column : columns) {
        out[py::str(column.first)] = py::array_t<double>(static_cast<py::ssize_t>(column.second.size()),
                                                         column.second.data());
    }
    return out;
}

template <std::size_t N, typename Get>
py::array_t<double> body_array(const std::vector<TickRecord>& rows, std::size_t bodies, Get get) {
    py::array_t<double> out({static_cast<py::ssize_t>(rows.size()), static_cast<py::ssize_t>(bodies),
                             static_cast<py::ssize_t>(N)});
    auto view = out.mutable_unchecked<3>();
    for (std::size_t r = 0; r < rows.size(); ++r) {
        for (std::size_t b = 0; b < bodies; ++b) {
            const std::array<double, N>& values = get(rows[r].physics.bodies[b]);
            for (std::size_t k = 0; k < N; ++k) {
                view(static_cast<py::ssize_t>(r), static_cast<py::ssize_t>(b), static_cast<py::ssize_t>(k)) =
                    values[k];
            }
        }
    }
    return out;
}

// Columns: time and steps (int64, [rows]); body_* ([rows, bodies, 3 or 4]); rope_tension ([rows, ropes]);
// pin_signals, rails, actuator_efforts as {name: [rows]}. Body and rope counts must be the same in every row.
py::dict drain_arrays(BufferedRecorderSink& sink) {
    const std::vector<TickRecord> rows = sink.take();
    std::size_t bodies = 0;
    std::size_t ropes = 0;
    if (!rows.empty()) {
        bodies = rows.front().physics.bodies.size();
        ropes = rows.front().physics.ropes.size();
    }
    const auto count = static_cast<py::ssize_t>(rows.size());
    py::array_t<std::int64_t> time(count);
    py::array_t<std::int64_t> steps(count);
    py::array_t<double> tension({count, static_cast<py::ssize_t>(ropes)});
    auto time_view = time.mutable_unchecked<1>();
    auto steps_view = steps.mutable_unchecked<1>();
    auto tension_view = tension.mutable_unchecked<2>();
    for (std::size_t r = 0; r < rows.size(); ++r) {
        const TickRecord& row = rows[r];
        if (row.physics.bodies.size() != bodies || row.physics.ropes.size() != ropes) {
            throw std::invalid_argument("body or rope count changes between rows; drain as dicts instead");
        }
        const auto index = static_cast<py::ssize_t>(r);
        time_view(index) = row.time.count();
        steps_view(index) = row.physics.steps;
        for (std::size_t k = 0; k < ropes; ++k) {
            tension_view(index, static_cast<py::ssize_t>(k)) = row.physics.ropes[k].tension;
        }
    }
    py::dict out;
    out["time"] = time;
    out["steps"] = steps;
    out["body_position"] = body_array<3>(rows, bodies, [](const kernel::BodyState& b) -> const kernel::Vec3& {
        return b.position;
    });
    out["body_orientation"] = body_array<4>(rows, bodies, [](const kernel::BodyState& b) -> const kernel::Quat& {
        return b.orientation;
    });
    out["body_angular_velocity"] = body_array<3>(
        rows, bodies, [](const kernel::BodyState& b) -> const kernel::Vec3& { return b.angular_velocity; });
    out["body_linear_velocity"] = body_array<3>(
        rows, bodies, [](const kernel::BodyState& b) -> const kernel::Vec3& { return b.linear_velocity; });
    out["rope_tension"] = tension;
    out["pin_signals"] = series_arrays(rows, &TickRecord::pin_signals);
    out["rails"] = series_arrays(rows, &TickRecord::rails);
    out["actuator_efforts"] = series_arrays(rows, &TickRecord::actuator_efforts);
    return out;
}

// A Python callback receives its own copy of each row (REC-020); the pybind11 function wrapper takes the GIL.
TickCallback copying_callback(std::function<void(TickRecord)> callback) {
    return [callback = std::move(callback)](const TickRecord& row) { callback(row); };
}

}  // namespace

void bind_recorder(py::module_& m) {
    py::class_<kernel::SeriesSample>(m, "SeriesSample")
        .def(py::init<>())
        .def_readwrite("name", &kernel::SeriesSample::name)
        .def_readwrite("unit", &kernel::SeriesSample::unit)
        .def_readwrite("value", &kernel::SeriesSample::value);
    py::class_<kernel::RecordedEvent>(m, "RecordedEvent")
        .def(py::init<>())
        .def_readwrite("time", &kernel::RecordedEvent::time)
        .def_readwrite("source", &kernel::RecordedEvent::source)
        .def_readwrite("kind", &kernel::RecordedEvent::kind)
        .def_readwrite("detail", &kernel::RecordedEvent::detail);
    py::class_<TickRecord>(m, "TickRecord", "One recorder row per physics tick (KER-009, REC-011).")
        .def(py::init<>())
        .def_readwrite("time", &TickRecord::time)
        .def_readwrite("physics", &TickRecord::physics)
        .def_readwrite("pin_signals", &TickRecord::pin_signals)
        .def_readwrite("rails", &TickRecord::rails)
        .def_readwrite("actuator_efforts", &TickRecord::actuator_efforts)
        .def_readwrite("controller_state", &TickRecord::controller_state)
        .def_readwrite("events", &TickRecord::events)
        .def("to_dict", &row_dict);
    py::class_<SubscriptionId>(m, "SubscriptionId")
        .def(py::init<>())
        .def(py::init<std::uint64_t>(), py::arg("value"))
        .def("value", &SubscriptionId::value)
        .def("__eq__", [](SubscriptionId a, SubscriptionId b) { return a == b; })
        .def("__hash__", [](SubscriptionId id) { return py::hash(py::int_(id.value())); });

    py::class_<RecorderSink, PyRecorderSink, py::smart_holder>(
        m, "RecorderSink", "Where the kernel streams each tick (KER-022); subclass it in Python to receive rows.")
        .def(py::init<>())
        .def("write_row", &RecorderSink::write_row, py::arg("row"))
        .def("subscribe", &RecorderSink::subscribe, py::arg("callback"))
        .def("unsubscribe", &RecorderSink::unsubscribe, py::arg("id"));

    py::class_<kernel::NullRecorderSink, RecorderSink, py::smart_holder>(m, "NullRecorderSink")
        .def(py::init<>())
        .def(
            "subscribe",
            [](kernel::NullRecorderSink& sink, std::function<void(TickRecord)> callback) {
                return sink.subscribe(copying_callback(std::move(callback)));
            },
            py::arg("callback"))
        .def_property_readonly("rows_written", &kernel::NullRecorderSink::rows_written)
        .def_property_readonly("subscriber_count", &kernel::NullRecorderSink::subscriber_count);

    py::class_<BufferedRecorderSink, RecorderSink, py::smart_holder>(
        m, "BufferedRecorderSink", "Buffers rows for drain_dicts() or drain_arrays(); forwards them to subscribers.")
        .def(py::init<>())
        .def(
            "subscribe",
            [](BufferedRecorderSink& sink, std::function<void(TickRecord)> callback) {
                return sink.subscribe(copying_callback(std::move(callback)));
            },
            py::arg("callback"), "Calls callback(row) inside each write_row, in subscription order (REC-019).")
        .def_property_readonly("pending", &BufferedRecorderSink::pending)
        .def_property_readonly("subscriber_count", &BufferedRecorderSink::subscriber_count)
        .def("drain_dicts", &drain_dicts, "Removes and returns the buffered rows as dicts, oldest first.")
        .def("drain_arrays", &drain_arrays, "Removes and returns the buffered rows as numpy columns.");
}

}  // namespace mechsim::bindings
