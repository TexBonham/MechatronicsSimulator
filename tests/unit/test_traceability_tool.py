"""Unit tests for tools/traceability.py, the requirement traceability checker and matrix generator."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

import traceability
from traceability import (
    Citation,
    Requirement,
    Violation,
    collect,
    cpp_citations,
    main,
    parse_spec,
    python_citations,
    render_matrix,
    split_row,
)

TOOL = Path(traceability.__file__).resolve()
HEADER = "| ID | Requirement | Source | Milestone | Tier | Tolerance |\n|---|---|---|---|---|---|\n"


def write(root: Path, relative: str, text: str) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="")
    return path


def spec(*rows: str) -> str:
    return (
        "# Spec: x (X)\n\n## Purpose\n\nText.\n\n## Requirements\n\n" + HEADER + "".join(rows) + "\n## Out of scope\n"
    )


def row(req_id: str, text: str = "The x shall y.", milestone: str = "M1", tier: str = "unit") -> str:
    return f"| {req_id} | {text} | ADR 0008 | {milestone} | {tier} | n/a |\n"


def cited_test(*ids: str) -> str:
    decorators = "".join(f'@pytest.mark.req("{req_id}")\n' for req_id in ids)
    return f"import pytest\n\n\n{decorators}def test_x() -> None:\n    pass\n"


def messages(root: Path) -> list[str]:
    return [str(violation) for violation in collect(root).violations]


def commit_matrix(root: Path) -> None:
    assert main(["--write", "--root", str(root)]) in (0, 1)


# ===== Spec table parsing =============================================================================================


def test_parse_spec_reads_rows_with_file_and_line() -> None:
    requirements, violations = parse_spec(
        spec(row("ENG-001"), row("ENG-002", milestone="M2", tier="reference")), "s.md"
    )
    assert violations == []
    assert requirements == [
        Requirement("ENG-001", "s.md", 11, "The x shall y.", "M1", "unit"),
        Requirement("ENG-002", "s.md", 12, "The x shall y.", "M2", "reference"),
    ]


def test_parse_spec_crlf() -> None:
    requirements, violations = parse_spec(spec(row("ENG-001")).replace("\n", "\r\n"), "s.md")
    assert violations == []
    assert [(item.req_id, item.line, item.tier) for item in requirements] == [("ENG-001", 11, "unit")]


def test_parse_spec_withdrawn_row() -> None:
    requirements, _ = parse_spec(spec(row("ENG-001", "Withdrawn: merged into ENG-002.")), "s.md")
    assert requirements[0].withdrawn


def test_parse_spec_withdrawn_only_as_prefix() -> None:
    requirements, _ = parse_spec(spec(row("ENG-001", "The x shall log Withdrawn: events.")), "s.md")
    assert not requirements[0].withdrawn


def test_parse_spec_escaped_pipe_in_requirement() -> None:
    requirements, violations = parse_spec(spec(row("ENG-001", r"The x shall keep \|v\| below 1 m/s.")), "s.md")
    assert violations == []
    assert requirements[0].text == "The x shall keep |v| below 1 m/s."


@pytest.mark.parametrize("bad", ["eng-001", "ENG-01", "ENG-0001", "E-001", "ENGINE-001", "ENG001", "ENG_001", ""])
def test_parse_spec_id_failing_pattern(bad: str) -> None:
    requirements, violations = parse_spec(spec(row(bad)), "s.md")
    assert requirements == []
    assert violations == [Violation("s.md", 11, f"requirement ID '{bad}' does not match the pattern")]


def test_parse_spec_heading_without_rows() -> None:
    text = "# Spec\n\n## Requirements\n\nNone yet.\n\n## Open parameters\n\n| a | b |\n|---|---|\n"
    requirements, violations = parse_spec(text, "s.md")
    assert requirements == []
    assert violations == [Violation("s.md", 3, "'## Requirements' heading has no table rows")]


def test_parse_spec_heading_at_end_of_file_without_rows() -> None:
    _, violations = parse_spec("# Spec\n## Requirements\n", "s.md")
    assert violations == [Violation("s.md", 2, "'## Requirements' heading has no table rows")]


def test_parse_spec_header_and_separator_only() -> None:
    _, violations = parse_spec("## Requirements\n\n" + HEADER, "s.md")
    assert violations == [Violation("s.md", 1, "'## Requirements' table has no requirement rows")]


def test_parse_spec_wrong_header() -> None:
    text = "## Requirements\n\n| ID | Text |\n|---|---|\n| ENG-001 | x |\n"
    requirements, violations = parse_spec(text, "s.md")
    assert requirements == []
    assert [violation.line for violation in violations] == [3]
    assert "header must be" in violations[0].message


def test_parse_spec_short_row() -> None:
    _, violations = parse_spec(spec("| ENG-001 | The x shall y. | ADR 0008 |\n"), "s.md")
    assert violations == [Violation("s.md", 11, "requirements row has 3 cells, expected 6")]


def test_parse_spec_no_requirements_heading_is_not_a_violation() -> None:
    assert parse_spec("# Spec\n\n## Purpose\n\nText.\n", "s.md") == ([], [])


def test_parse_spec_ignores_other_tables_and_subheadings() -> None:
    text = spec(row("ENG-001")) + "\n| Item | Seam |\n|---|---|\n| BAD-1 | x |\n"
    requirements, violations = parse_spec(text, "s.md")
    assert [item.req_id for item in requirements] == ["ENG-001"]
    assert violations == []


def test_parse_spec_level_three_heading_stays_in_section() -> None:
    text = "## Requirements\n\n### Group\n\n" + HEADER + row("ENG-001")
    requirements, violations = parse_spec(text, "s.md")
    assert [item.req_id for item in requirements] == ["ENG-001"]
    assert violations == []


def test_parse_spec_ignores_heading_inside_code_fence() -> None:
    text = "# Spec\n\n```markdown\n## Requirements\n```\n"
    assert parse_spec(text, "s.md") == ([], [])


def test_parse_spec_two_requirement_sections() -> None:
    text = spec(row("ENG-001")) + "## Requirements\n\n" + HEADER + row("ENG-002")
    requirements, _ = parse_spec(text, "s.md")
    assert [item.req_id for item in requirements] == ["ENG-001", "ENG-002"]


def test_split_row_without_outer_pipes() -> None:
    assert split_row("a | b") == ["a", "b"]


# ===== Python citations ===============================================================================================


def test_python_function_decorator() -> None:
    citations, violations = python_citations(cited_test("ENG-001"), "t.py")
    assert violations == []
    assert citations == [Citation("ENG-001", "t.py", 4)]


def test_python_stacked_decorators() -> None:
    source = (
        "import pytest\n\n\n"
        '@pytest.mark.req("ENG-001")\n'
        '@pytest.mark.xfail(strict=True, reason="stub")\n'
        '@pytest.mark.req("KER-002")\n'
        "def test_x() -> None:\n    raise NotImplementedError\n"
    )
    citations, violations = python_citations(source, "t.py")
    assert violations == []
    assert citations == [Citation("ENG-001", "t.py", 4), Citation("KER-002", "t.py", 6)]


def test_python_mark_alias() -> None:
    source = 'from pytest import mark\n\n\n@mark.req("ENG-001")\ndef test_x() -> None:\n    pass\n'
    citations, _ = python_citations(source, "t.py")
    assert citations == [Citation("ENG-001", "t.py", 4)]


def test_python_async_function() -> None:
    source = 'import pytest\n\n\n@pytest.mark.req("ENG-001")\nasync def test_x() -> None:\n    pass\n'
    citations, _ = python_citations(source, "t.py")
    assert citations == [Citation("ENG-001", "t.py", 4)]


def test_python_class_decorator_counts_for_every_test_method() -> None:
    source = (
        "import pytest\n\n\n"
        '@pytest.mark.req("ENG-001")\n'
        "class TestX:\n"
        "    def test_a(self) -> None:\n        pass\n\n"
        "    def helper(self) -> None:\n        pass\n\n"
        "    async def test_b(self) -> None:\n        pass\n\n"
        "    class TestNested:\n"
        "        def test_c(self) -> None:\n            pass\n"
    )
    citations, violations = python_citations(source, "t.py")
    assert violations == []
    assert citations == [
        Citation("ENG-001", "t.py", 6),
        Citation("ENG-001", "t.py", 12),
        Citation("ENG-001", "t.py", 16),
    ]


def test_python_class_decorator_without_test_methods_cites_nothing() -> None:
    source = 'import pytest\n\n\n@pytest.mark.req("ENG-001")\nclass TestX:\n    pass\n'
    assert python_citations(source, "t.py") == ([], [])


def test_python_class_and_method_decorators_combine() -> None:
    source = (
        "import pytest\n\n\n"
        '@pytest.mark.req("ENG-001")\n'
        "class TestX:\n"
        '    @pytest.mark.req("ENG-002")\n'
        "    def test_a(self) -> None:\n        pass\n"
    )
    citations, _ = python_citations(source, "t.py")
    assert citations == [Citation("ENG-001", "t.py", 7), Citation("ENG-002", "t.py", 6)]


def test_python_marker_outside_decorator() -> None:
    source = 'import pytest\n\npytestmark = pytest.mark.req("ENG-001")\n'
    citations, _ = python_citations(source, "t.py")
    assert citations == [Citation("ENG-001", "t.py", 3)]


def test_python_marker_inside_param_marks() -> None:
    source = 'import pytest\n\nCASES = [pytest.param(1, marks=pytest.mark.req("ENG-001"))]\n'
    citations, _ = python_citations(source, "t.py")
    assert citations == [Citation("ENG-001", "t.py", 3)]


def test_python_marker_with_several_ids() -> None:
    source = 'import pytest\n\n\n@pytest.mark.req("ENG-001", "ENG-002")\ndef test_x() -> None:\n    pass\n'
    citations, _ = python_citations(source, "t.py")
    assert [citation.req_id for citation in citations] == ["ENG-001", "ENG-002"]


def test_python_marker_in_string_is_not_a_citation() -> None:
    source = 'SOURCE = """@pytest.mark.req(\\"ENG-001\\")"""\n'
    assert python_citations(source, "t.py") == ([], [])


@pytest.mark.parametrize(
    "decorator",
    ["pytest.mark.requires", "other.mark.req", "pytest.marks.req", "self.mark.req", "req", "pytest.req"],
)
def test_python_other_decorators_are_ignored(decorator: str) -> None:
    source = f'import pytest\n\n\n@{decorator}("ENG-001")\ndef test_x() -> None:\n    pass\n'
    assert python_citations(source, "t.py") == ([], [])


@pytest.mark.parametrize("bad", ["eng-001", "ENG-01", "ENG-0001", "ENG001", "E-001", "ENG-001 ", ""])
def test_python_marker_id_failing_pattern(bad: str) -> None:
    citations, violations = python_citations(cited_test(bad), "t.py")
    assert citations == []
    assert violations == [Violation("t.py", 4, f"req marker ID '{bad}' fails the pattern")]


def test_python_marker_non_literal_argument() -> None:
    source = 'import pytest\n\nID = "ENG-001"\n\n\n@pytest.mark.req(ID)\ndef test_x() -> None:\n    pass\n'
    citations, violations = python_citations(source, "t.py")
    assert citations == []
    assert violations == [Violation("t.py", 6, "req marker argument is not a string literal")]


def test_python_marker_without_arguments() -> None:
    source = "import pytest\n\n\n@pytest.mark.req()\ndef test_x() -> None:\n    pass\n"
    _, violations = python_citations(source, "t.py")
    assert violations == [Violation("t.py", 4, "req marker takes one or more positional string IDs")]


def test_python_marker_with_keyword_argument() -> None:
    source = 'import pytest\n\n\n@pytest.mark.req(id="ENG-001")\ndef test_x() -> None:\n    pass\n'
    _, violations = python_citations(source, "t.py")
    assert violations == [Violation("t.py", 4, "req marker takes one or more positional string IDs")]


def test_python_bare_marker_decorator() -> None:
    source = "import pytest\n\n\n@pytest.mark.req\ndef test_x() -> None:\n    pass\n"
    citations, violations = python_citations(source, "t.py")
    assert citations == []
    assert violations == [Violation("t.py", 4, "req marker used without an ID")]


def test_python_syntax_error_reported_with_line() -> None:
    citations, violations = python_citations("import pytest\n\ndef broken(:\n", "t.py")
    assert citations == []
    assert len(violations) == 1
    assert violations[0].path == "t.py"
    assert violations[0].line == 3
    assert violations[0].message.startswith("cannot parse file")


def test_python_null_byte_reported() -> None:
    _, violations = python_citations(b"x = 1\x00\n", "t.py")
    assert len(violations) == 1
    assert violations[0].message.startswith("cannot parse file")


def test_python_crlf_source() -> None:
    citations, _ = python_citations(cited_test("ENG-001").replace("\n", "\r\n").encode(), "t.py")
    assert citations == [Citation("ENG-001", "t.py", 4)]


# ===== C++ citations ==================================================================================================


def test_cpp_test_case_tag() -> None:
    source = '#include <catch2/catch_test_macros.hpp>\n\nTEST_CASE("period", "[ENG-001]") {\n    REQUIRE(true);\n}\n'
    assert cpp_citations(source, "c.cpp") == ([Citation("ENG-001", "c.cpp", 3)], [])


def test_cpp_scenario_tag() -> None:
    source = 'SCENARIO("period", "[KER-004][smoke]") {\n}\n'
    assert cpp_citations(source, "c.cpp") == ([Citation("KER-004", "c.cpp", 1)], [])


def test_cpp_stub_tags() -> None:
    source = 'TEST_CASE("ENG-001 period", "[ENG-001][!shouldfail][stub]") {\n    FAIL("stub");\n}\n'
    assert cpp_citations(source, "c.cpp") == ([Citation("ENG-001", "c.cpp", 1)], [])


def test_cpp_several_tags_in_one_case() -> None:
    citations, _ = cpp_citations('TEST_CASE("x", "[ENG-001][ENG-002]") {}\n', "c.cpp")
    assert [citation.req_id for citation in citations] == ["ENG-001", "ENG-002"]


def test_cpp_tag_string_spanning_lines() -> None:
    source = 'TEST_CASE(\n    "a long name",\n    "[ENG-001]"\n    "[ENG-002]"\n    "[!shouldfail]") {\n}\n'
    citations, violations = cpp_citations(source, "c.cpp")
    assert violations == []
    assert citations == [Citation("ENG-001", "c.cpp", 1), Citation("ENG-002", "c.cpp", 1)]


def test_cpp_tag_split_across_literals() -> None:
    citations, _ = cpp_citations('TEST_CASE("x", "[ENG-" "001]") {}\n', "c.cpp")
    assert citations == [Citation("ENG-001", "c.cpp", 1)]


def test_cpp_line_continuation_inside_tag_string() -> None:
    source = 'TEST_CASE("x", "[ENG-001]\\\n[ENG-002]") {}\nTEST_CASE("y", "[KER-001]") {}\n'
    citations, _ = cpp_citations(source, "c.cpp")
    assert citations == [
        Citation("ENG-001", "c.cpp", 1),
        Citation("ENG-002", "c.cpp", 1),
        Citation("KER-001", "c.cpp", 3),
    ]


def test_cpp_name_holding_id_is_not_a_citation() -> None:
    assert cpp_citations('TEST_CASE("[ENG-001] period", "[smoke]") {}\n', "c.cpp") == ([], [])


def test_cpp_name_only_case() -> None:
    assert cpp_citations('TEST_CASE("period") {}\n', "c.cpp") == ([], [])


def test_cpp_comments_are_ignored() -> None:
    source = '// TEST_CASE("x", "[ENG-001]")\n/* TEST_CASE("y", "[ENG-002]")\n */\nTEST_CASE("z", "[ENG-003]") {}\n'
    assert cpp_citations(source, "c.cpp") == ([Citation("ENG-003", "c.cpp", 4)], [])


def test_cpp_macro_name_in_string_is_ignored() -> None:
    source = 'const char* s = "TEST_CASE(\\"x\\", \\"[ENG-001]\\")";\n'
    assert cpp_citations(source, "c.cpp") == ([], [])


def test_cpp_raw_string_tag() -> None:
    source = 'auto s = R"x(ignored "quote")x";\nTEST_CASE("x", R"([ENG-001])") {}\n'
    assert cpp_citations(source, "c.cpp") == ([Citation("ENG-001", "c.cpp", 2)], [])


def test_cpp_other_macros_are_ignored() -> None:
    source = (
        'TEMPLATE_TEST_CASE("x", "[ENG-001]", int) {}\n'
        'TEST_CASE_METHOD(Fixture, "x", "[ENG-002]") {}\n'
        "#define TEST_CASE_X 1\n"
        "int TEST_CASE = 0;\n"
    )
    assert cpp_citations(source, "c.cpp") == ([], [])


def test_cpp_nested_parentheses_and_char_literals() -> None:
    source = "TEST_CASE(name(')'), \"[ENG-001]\") {}\n"
    assert cpp_citations(source, "c.cpp") == ([Citation("ENG-001", "c.cpp", 1)], [])


@pytest.mark.parametrize("bad", ["eng-001", "ENG-01", "ENG-0001", "ENG001", "E-001", "ENG_001", "ENGINE-001"])
def test_cpp_tag_looking_like_id_but_failing(bad: str) -> None:
    citations, violations = cpp_citations(f'TEST_CASE("x", "[{bad}]") {{}}\n', "c.cpp")
    assert citations == []
    assert violations == [Violation("c.cpp", 1, f"Catch2 tag '[{bad}]' looks like an ID but fails the pattern")]


@pytest.mark.parametrize("tag", ["smoke", "!shouldfail", "stub", "!mayfail", ".", "x2", "cpp20", "integration", ""])
def test_cpp_ordinary_tags_are_not_violations(tag: str) -> None:
    assert cpp_citations(f'TEST_CASE("x", "[{tag}]") {{}}\n', "c.cpp") == ([], [])


def test_cpp_crlf_source() -> None:
    source = 'TEST_CASE("x",\r\n "[ENG-001]") {}\r\nTEST_CASE("y", "[ENG-002]") {}\r\n'
    citations, _ = cpp_citations(source, "c.cpp")
    assert citations == [Citation("ENG-001", "c.cpp", 1), Citation("ENG-002", "c.cpp", 3)]


def test_cpp_unterminated_constructs_do_not_crash() -> None:
    assert cpp_citations('TEST_CASE("x", "[ENG-001]"\n/* open', "c.cpp") == ([Citation("ENG-001", "c.cpp", 1)], [])
    assert cpp_citations('TEST_CASE("x", R"(', "c.cpp") == ([], [])
    assert cpp_citations("TEST_CASE", "c.cpp") == ([], [])


# ===== Repository collection ==========================================================================================


def test_zero_specs_is_not_a_violation(tmp_path: Path) -> None:
    write(tmp_path, "tests/test_x.py", "def test_x() -> None:\n    pass\n")
    collection = collect(tmp_path)
    assert collection.requirements == []
    assert collection.violations == []


def test_empty_root(tmp_path: Path) -> None:
    assert messages(tmp_path) == []


def test_covered_requirement_is_clean(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001"), row("ENG-002")))
    write(tmp_path, "tests/unit/test_x.py", cited_test("ENG-001"))
    write(tmp_path, "core/engine/tests/test_y.cpp", 'TEST_CASE("y", "[ENG-002]") {}\n')
    assert messages(tmp_path) == []


def test_requirement_without_citation(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001")))
    assert messages(tmp_path) == ["docs/spec/eng.md:11: requirement ENG-001 has no test citing it"]


def test_withdrawn_requirement_needs_no_citation(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001", "Withdrawn: superseded.")))
    assert messages(tmp_path) == []


def test_citation_of_unknown_id(tmp_path: Path) -> None:
    write(tmp_path, "tests/unit/test_x.py", cited_test("ENG-009"))
    write(tmp_path, "core/a/b.hpp", 'TEST_CASE("y", "[KER-001]") {}\n')
    assert messages(tmp_path) == [
        "core/a/b.hpp:1: test cites KER-001, which no spec defines",
        "tests/unit/test_x.py:4: test cites ENG-009, which no spec defines",
    ]


def test_duplicate_id_across_two_specs(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001")))
    write(tmp_path, "docs/spec/ker.md", spec(row("ENG-001", "Other text.")))
    write(tmp_path, "tests/test_x.py", cited_test("ENG-001"))
    assert messages(tmp_path) == ["docs/spec/ker.md:11: requirement ENG-001 is also defined at docs/spec/eng.md:11"]


def test_duplicate_id_within_one_spec(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001"), row("ENG-001")))
    write(tmp_path, "tests/test_x.py", cited_test("ENG-001"))
    assert messages(tmp_path) == ["docs/spec/eng.md:12: requirement ENG-001 is also defined at docs/spec/eng.md:11"]


def test_non_spec_files_in_spec_dir_are_skipped(tmp_path: Path) -> None:
    for name in ("0000-template.md", "README.md", "traceability.md"):
        write(tmp_path, f"docs/spec/{name}", spec(row("ENG-001")))
    write(tmp_path, "docs/spec/notes.txt", spec(row("ENG-002")))
    write(tmp_path, "docs/spec/sub/eng.md", spec(row("ENG-003")))
    assert collect(tmp_path).requirements == []


def test_spec_dir_violations_surface_through_collect(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", "## Requirements\n")
    write(tmp_path, "docs/spec/ker.md", spec(row("ker-001")))
    assert messages(tmp_path) == [
        "docs/spec/eng.md:1: '## Requirements' heading has no table rows",
        "docs/spec/ker.md:11: requirement ID 'ker-001' does not match the pattern",
    ]


def test_crlf_spec_file(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001")).replace("\n", "\r\n"))
    write(tmp_path, "tests/test_x.py", cited_test("ENG-001"))
    collection = collect(tmp_path)
    assert collection.violations == []
    assert [(item.req_id, item.text) for item in collection.requirements] == [("ENG-001", "The x shall y.")]


def test_spec_with_bom(tmp_path: Path) -> None:
    (tmp_path / "docs/spec").mkdir(parents=True)
    (tmp_path / "docs/spec/eng.md").write_bytes(b"\xef\xbb\xbf" + spec(row("ENG-001")).encode())
    assert [item.req_id for item in collect(tmp_path).requirements] == ["ENG-001"]


def test_undecodable_spec(tmp_path: Path) -> None:
    (tmp_path / "docs/spec").mkdir(parents=True)
    (tmp_path / "docs/spec/eng.md").write_bytes(b"## Requirements\n\xff\xfe\n")
    violations = collect(tmp_path).violations
    assert len(violations) == 1
    assert violations[0].path == "docs/spec/eng.md"
    assert violations[0].message.startswith("cannot decode file as UTF-8")


def test_undecodable_cpp(tmp_path: Path) -> None:
    (tmp_path / "core").mkdir()
    (tmp_path / "core/x.cpp").write_bytes(b"\xff\n")
    assert collect(tmp_path).violations[0].message.startswith("cannot decode file as UTF-8")


def test_syntax_error_file_under_tests(tmp_path: Path) -> None:
    write(tmp_path, "tests/unit/test_broken.py", "x = 1\n\nif True\n")
    violations = collect(tmp_path).violations
    assert [(violation.path, violation.line) for violation in violations] == [("tests/unit/test_broken.py", 3)]


def test_only_tests_and_core_are_scanned(tmp_path: Path) -> None:
    write(tmp_path, "tools/test_x.py", cited_test("ENG-001"))
    write(tmp_path, "mechsim/x.py", cited_test("ENG-001"))
    write(tmp_path, "firmware/x.cpp", 'TEST_CASE("x", "[ENG-001]") {}\n')
    write(tmp_path, "core/x.h", 'TEST_CASE("x", "[ENG-001]") {}\n')
    write(tmp_path, "core/x.cc", 'TEST_CASE("x", "[ENG-001]") {}\n')
    write(tmp_path, "tests/data.txt", cited_test("ENG-001"))
    assert collect(tmp_path).citations == []


def test_build_and_hidden_dirs_are_skipped(tmp_path: Path) -> None:
    write(tmp_path, "core/build/x.cpp", 'TEST_CASE("x", "[ENG-001]") {}\n')
    write(tmp_path, "core/.cache/x.cpp", 'TEST_CASE("x", "[ENG-001]") {}\n')
    write(tmp_path, "tests/__pycache__/x.py", cited_test("ENG-001"))
    write(tmp_path, "tests/node_modules/x.py", cited_test("ENG-001"))
    assert collect(tmp_path).citations == []


def test_python_and_cpp_citations_collected_together(tmp_path: Path) -> None:
    write(tmp_path, "tests/unit/test_x.py", cited_test("ENG-001"))
    write(tmp_path, "core/engine/tests/test_y.cpp", 'TEST_CASE("y", "[ENG-001]") {}\n')
    assert collect(tmp_path).citations == [
        Citation("ENG-001", "core/engine/tests/test_y.cpp", 1),
        Citation("ENG-001", "tests/unit/test_x.py", 4),
    ]


# ===== Matrix =========================================================================================================


def test_matrix_with_no_requirements_holds_only_summary() -> None:
    text = render_matrix([], [])
    assert text == (
        "# Traceability matrix\n\n"
        "Generated by `tools/traceability.py`; do not edit by hand. "
        "Regenerate with `python3 tools/traceability.py --write`.\n\n"
        "## Summary\n\n"
        "| Prefix | Requirements | Covered | Uncovered | Withdrawn |\n"
        "|---|---|---|---|---|\n"
        "| Total | 0 | 0 | 0 | 0 |\n"
    )


def test_matrix_tables_and_summary() -> None:
    requirements = [
        Requirement("KER-001", "docs/spec/ker.md", 11, "The kernel shall tick.", "M1", "unit"),
        Requirement("ENG-002", "docs/spec/eng.md", 12, "Withdrawn: merged.", "M2", "reference"),
        Requirement("ENG-001", "docs/spec/eng.md", 11, "The engine shall keep |v| small.", "M1", "reference"),
        Requirement("ENG-003", "docs/spec/eng.md", 13, "The engine shall stop.", "roadmap", "unit"),
    ]
    citations = [
        Citation("ENG-001", "tests/b.py", 3),
        Citation("ENG-001", "core/a.cpp", 9),
        Citation("ENG-001", "tests/b.py", 3),
        Citation("ENG-002", "tests/c.py", 1),
        Citation("KER-001", "core/k.cpp", 2),
    ]
    text = render_matrix(requirements, citations)
    assert text.endswith(
        "## ENG\n\n"
        "| ID | Requirement | Milestone | Tier | Tests |\n"
        "|---|---|---|---|---|\n"
        "| ENG-001 | The engine shall keep \\|v\\| small. | M1 | reference | core/a.cpp:9, tests/b.py:3 |\n"
        "| ENG-002 | Withdrawn: merged. | M2 | reference | tests/c.py:1 |\n"
        "| ENG-003 | The engine shall stop. | roadmap | unit | none |\n\n"
        "## KER\n\n"
        "| ID | Requirement | Milestone | Tier | Tests |\n"
        "|---|---|---|---|---|\n"
        "| KER-001 | The kernel shall tick. | M1 | unit | core/k.cpp:2 |\n\n"
        "## Summary\n\n"
        "| Prefix | Requirements | Covered | Uncovered | Withdrawn |\n"
        "|---|---|---|---|---|\n"
        "| ENG | 3 | 1 | 1 | 1 |\n"
        "| KER | 1 | 1 | 0 | 0 |\n"
        "| Total | 4 | 2 | 1 | 1 |\n"
    )


def test_matrix_is_order_independent() -> None:
    requirements = [
        Requirement("ENG-002", "docs/spec/eng.md", 12, "b", "M1", "unit"),
        Requirement("ENG-001", "docs/spec/eng.md", 11, "a", "M1", "unit"),
    ]
    citations = [Citation("ENG-002", "tests/z.py", 1), Citation("ENG-001", "tests/a.py", 1)]
    assert render_matrix(requirements, citations) == render_matrix(requirements[::-1], citations[::-1])


def test_matrix_lists_duplicate_id_once_from_first_spec() -> None:
    requirements = [
        Requirement("ENG-001", "docs/spec/ker.md", 11, "second", "M1", "unit"),
        Requirement("ENG-001", "docs/spec/eng.md", 11, "first", "M1", "unit"),
    ]
    text = render_matrix(requirements, [])
    assert "| ENG-001 | first |" in text
    assert "second" not in text
    assert "| Total | 1 | 0 | 1 | 0 |" in text


# ===== CLI ============================================================================================================


def test_main_clean_repository(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001")))
    write(tmp_path, "tests/test_x.py", cited_test("ENG-001"))
    assert main(["--write", "--root", str(tmp_path)]) == 0
    assert "| ENG-001 |" in (tmp_path / "docs/spec/traceability.md").read_text(encoding="utf-8")
    capsys.readouterr()
    assert main(["--root", str(tmp_path)]) == 0
    assert capsys.readouterr().out == ""


def test_main_zero_specs_writes_summary_only_matrix(tmp_path: Path) -> None:
    assert main(["--write", "--root", str(tmp_path)]) == 0
    assert (tmp_path / "docs/spec/traceability.md").read_text(encoding="utf-8") == render_matrix([], [])
    assert main(["--root", str(tmp_path)]) == 0


def test_main_missing_matrix_is_stale(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--root", str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert "docs/spec/traceability.md is stale" in out
    assert "+| Total | 0 | 0 | 0 | 0 |" in out


def test_main_stale_matrix_prints_diff(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001")))
    write(tmp_path, "tests/test_x.py", cited_test("ENG-001"))
    commit_matrix(tmp_path)
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001", "The x shall z.")))
    capsys.readouterr()
    assert main(["--root", str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert "--- docs/spec/traceability.md (committed)" in out
    assert "+++ docs/spec/traceability.md (generated)" in out
    assert "-| ENG-001 | The x shall y. |" in out
    assert "+| ENG-001 | The x shall z. |" in out
    assert (tmp_path / "docs/spec/traceability.md").read_text(encoding="utf-8").count("The x shall y.") == 1


def test_main_hand_edited_matrix_is_stale(tmp_path: Path) -> None:
    commit_matrix(tmp_path)
    path = tmp_path / "docs/spec/traceability.md"
    path.write_text(path.read_text(encoding="utf-8") + "extra\n", encoding="utf-8")
    assert main(["--root", str(tmp_path)]) == 1


def test_main_crlf_matrix_counts_as_current(tmp_path: Path) -> None:
    commit_matrix(tmp_path)
    path = tmp_path / "docs/spec/traceability.md"
    path.write_bytes(path.read_bytes().replace(b"\n", b"\r\n"))
    assert main(["--root", str(tmp_path)]) == 0


def test_main_violations_fail_even_with_current_matrix(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001")))
    assert main(["--write", "--root", str(tmp_path)]) == 1
    assert (tmp_path / "docs/spec/traceability.md").is_file()
    capsys.readouterr()
    assert main(["--root", str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert out == "docs/spec/eng.md:11: requirement ENG-001 has no test citing it\n"


def test_main_write_overwrites_stale_matrix(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/traceability.md", "old\n")
    assert main(["--write", "--root", str(tmp_path)]) == 0
    assert main(["--root", str(tmp_path)]) == 0


def test_main_write_is_idempotent(tmp_path: Path) -> None:
    write(tmp_path, "docs/spec/eng.md", spec(row("ENG-001")))
    write(tmp_path, "tests/test_x.py", cited_test("ENG-001"))
    commit_matrix(tmp_path)
    first = (tmp_path / "docs/spec/traceability.md").read_bytes()
    commit_matrix(tmp_path)
    assert (tmp_path / "docs/spec/traceability.md").read_bytes() == first
    assert b"\r" not in first


def test_main_unknown_argument_is_usage_error(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--bogus"]) == 2
    assert "usage" in capsys.readouterr().err


def test_main_positional_argument_is_usage_error() -> None:
    assert main(["extra"]) == 2


def test_main_root_without_value_is_usage_error() -> None:
    assert main(["--root"]) == 2


def test_main_missing_root_is_usage_error(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--root", str(tmp_path / "missing")]) == 2
    assert "not a directory" in capsys.readouterr().err


def test_main_root_that_is_a_file_is_usage_error(tmp_path: Path) -> None:
    write(tmp_path, "file.txt", "x")
    assert main(["--root", str(tmp_path / "file.txt")]) == 2


def test_main_help_exits_zero(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--help"]) == 0
    assert "--write" in capsys.readouterr().out


def test_script_entry_point(tmp_path: Path) -> None:
    assert main(["--write", "--root", str(tmp_path)]) == 0
    result = subprocess.run(
        [sys.executable, str(TOOL), "--root", str(tmp_path)], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0
    usage = subprocess.run([sys.executable, str(TOOL), "--nope"], capture_output=True, text=True, check=False)
    assert usage.returncode == 2
