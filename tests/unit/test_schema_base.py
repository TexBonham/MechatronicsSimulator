"""The shared strict base for data-file models and the loaders' error formatter (L4, invariant 3)."""

from __future__ import annotations

import pytest
from pydantic import Field, ValidationError, field_validator

from mechsim.design import schema as design_schema
from mechsim.schema_base import StrictModel, format_errors


class Leaf(StrictModel):
    value: int


class Node(StrictModel):
    name: str
    leaf: Leaf
    items: list[Leaf] = Field(default_factory=list)


class Other(StrictModel):
    label: str


class Choice(StrictModel):
    pick: Leaf | Other


class Checked(StrictModel):
    count: int

    @field_validator("count")
    @classmethod
    def _positive(cls, count: int) -> int:
        if count <= 0:
            raise ValueError("count must be positive\ncount must be positive\nsee the spec")
        return count


def _error(data: object) -> ValidationError:
    with pytest.raises(ValidationError) as caught:
        Node.model_validate(data)
    return caught.value


@pytest.mark.req("INV-003")
def test_unknown_key_rejected() -> None:
    error = _error({"name": "a", "leaf": {"value": 1}, "extra": 2})
    assert [item["type"] for item in error.errors()] == ["extra_forbidden"]


@pytest.mark.req("INV-003")
def test_unknown_nested_key_rejected() -> None:
    error = _error({"name": "a", "leaf": {"value": 1, "unit": "m"}})
    assert [item["loc"] for item in error.errors()] == [("leaf", "unit")]


def test_configuration_flags() -> None:
    assert StrictModel.model_config.get("extra") == "forbid"
    assert StrictModel.model_config.get("strict") is True
    assert Leaf.model_config.get("extra") == "forbid"


def test_strict_rejects_coercion() -> None:
    error = _error({"name": "a", "leaf": {"value": "1"}})
    assert [item["type"] for item in error.errors()] == ["int_type"]


def test_valid_data_accepted() -> None:
    node = Node.model_validate({"name": "a", "leaf": {"value": 1}, "items": [{"value": 2}]})
    assert node.leaf.value == 1
    assert node.items[0].value == 2


def test_format_one_error() -> None:
    error = _error({"name": "a", "leaf": {"value": 1}, "extra": 2})
    assert format_errors(error) == ["extra: Extra inputs are not permitted"]


def test_format_several_errors() -> None:
    error = _error({"name": 3, "leaf": {"value": "x"}, "items": [{"value": 1}, {"value": 2, "bad": 0}]})
    assert format_errors(error) == [
        "name: Input should be a valid string",
        "leaf.value: Input should be a valid integer",
        "items.1.bad: Extra inputs are not permitted",
    ]


def test_format_top_level_error_has_no_location() -> None:
    error = _error([1])
    assert format_errors(error) == ["Input should be a valid dictionary or instance of Node"]


def test_format_splits_lines_drops_prefix_and_deduplicates() -> None:
    with pytest.raises(ValidationError) as caught:
        Checked.model_validate({"count": 0})
    assert format_errors(caught.value) == ["count: count must be positive", "count: see the spec"]


def test_format_drops_union_branch_names() -> None:
    with pytest.raises(ValidationError) as caught:
        Choice.model_validate({"pick": {"value": 1, "label": "a"}})
    assert format_errors(caught.value) == [
        "pick.label: Extra inputs are not permitted",
        "pick.value: Extra inputs are not permitted",
    ]


def test_design_aliases_are_public_names() -> None:
    assert design_schema._Strict is StrictModel
    assert design_schema._format_errors is format_errors
