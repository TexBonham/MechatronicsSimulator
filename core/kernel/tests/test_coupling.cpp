#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/time.hpp"

using mechsim::kernel::CouplingError;
using mechsim::kernel::CouplingReader;
using mechsim::kernel::CouplingTable;
using mechsim::kernel::CouplingValue;
using mechsim::kernel::CouplingWriter;
using mechsim::kernel::KernelConfigError;

TEST_CASE("Coupling table holds one publisher per name and keeps values in name order", "[KER-008][KER-013]") {
    CouplingTable table;
    table.declare("z.last", "p");
    table.declare("a.first", "q");
    CHECK(table.contains("a.first"));
    CHECK_FALSE(table.contains("b"));
    CHECK(table.publisher("z.last") == "p");
    CHECK_THROWS_AS(table.declare("a.first", "r"), KernelConfigError);
    CHECK_THROWS_AS(table.declare("", "r"), KernelConfigError);
    CHECK(std::holds_alternative<std::monostate>(table.value("a.first")));
    CHECK_THROWS_AS(table.value("missing"), CouplingError);
    CHECK_THROWS_AS(table.publisher("missing"), CouplingError);

    table.set("z.last", CouplingValue{2.5});
    const auto snapshot = table.snapshot();
    REQUIRE(snapshot.size() == 2);
    CHECK(snapshot[0].name == "a.first");
    CHECK(snapshot[1].name == "z.last");
    CHECK(snapshot[1].value == CouplingValue{2.5});

    CHECK_THROWS_AS(table.set("missing", CouplingValue{1.0}), CouplingError);
    CHECK_THROWS_AS(table.set("z.last", CouplingValue{}), CouplingError);
    CHECK_THROWS_AS(table.set("z.last", CouplingValue{true}), CouplingError);
    CHECK_THROWS_AS(table.set("z.last", CouplingValue{std::int64_t{1}}), CouplingError);
    table.set("z.last", CouplingValue{3.5});
    CHECK(table.value("z.last") == CouplingValue{3.5});
}

TEST_CASE("Readers see only consumed names and writers write only published names", "[KER-008][KER-020]") {
    CouplingTable table;
    table.declare("r.real", "p");
    table.declare("r.int", "p");
    table.declare("r.flag", "p");
    table.declare("r.unset", "p");
    table.declare("hidden", "p");
    const std::vector<std::string> published{"r.real", "r.int", "r.flag"};
    CouplingWriter writer(table, published);
    writer.set("r.real", CouplingValue{1.5});
    writer.set("r.int", CouplingValue{std::int64_t{7}});
    writer.set("r.flag", CouplingValue{true});
    CHECK_THROWS_AS(writer.set("hidden", CouplingValue{1.0}), CouplingError);

    const std::vector<std::string> consumed{"r.real", "r.int", "r.flag", "r.unset"};
    const CouplingReader reader(table, consumed);
    CHECK(reader.real("r.real") == 1.5);
    CHECK(reader.integer("r.int") == 7);
    CHECK(reader.flag("r.flag"));
    CHECK(reader.has_value("r.real"));
    CHECK_FALSE(reader.has_value("r.unset"));
    CHECK_THROWS_AS(reader.value("hidden"), CouplingError);
    CHECK_THROWS_AS(reader.real("r.int"), CouplingError);
    CHECK_THROWS_AS(reader.integer("r.flag"), CouplingError);
    CHECK_THROWS_AS(reader.flag("r.real"), CouplingError);
    CHECK_THROWS_AS(reader.real("r.unset"), CouplingError);
}
