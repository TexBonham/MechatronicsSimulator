"""Run-twice comparison of two archives (REC-022, INV-014): byte for byte, then recorded content."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import h5py  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class Report:
    """`identical`: the files are byte-identical. `content_identical`: every attribute and dataset is equal.
    `first_difference` names the first differing attribute or dataset and index, or None."""

    identical: bool
    content_identical: bool
    first_difference: str | None


def _attr_key(value: Any) -> object:
    if isinstance(value, bytes):
        return ("text", value.decode("utf-8"))
    if isinstance(value, str):
        return ("text", value)
    array = np.asarray(value)
    if array.dtype == object:
        return ("texts", array.shape, tuple(_attr_key(item) for item in array.reshape(-1)))
    return (array.dtype.str, array.shape, array.tobytes())


def _attrs_difference(where: str, a: Any, b: Any) -> str | None:
    for name in sorted(set(a.keys()) | set(b.keys())):
        if name not in b:
            return f"{where} attribute {name!r}: only in the first archive"
        if name not in a:
            return f"{where} attribute {name!r}: only in the second archive"
        if _attr_key(a[name]) != _attr_key(b[name]):
            return f"{where} attribute {name!r}: {a[name]!r} != {b[name]!r}"
    return None


def _values_difference(path: str, a: NDArray[Any], b: NDArray[Any]) -> str | None:
    if a.shape != b.shape:
        return f"dataset {path}: shape {a.shape} != {b.shape}"
    if a.dtype != b.dtype:
        return f"dataset {path}: dtype {a.dtype} != {b.dtype}"
    if a.size == 0:
        return None
    width = a.dtype.itemsize
    left = np.ascontiguousarray(a).reshape(-1).view(np.uint8).reshape(a.size, width)
    right = np.ascontiguousarray(b).reshape(-1).view(np.uint8).reshape(b.size, width)
    differing = np.nonzero(np.any(left != right, axis=1))[0]
    if differing.size == 0:
        return None
    index = tuple(int(part) for part in np.unravel_index(int(differing[0]), a.shape))
    return f"dataset {path}: first difference at index {index}"


def _objects(h5: Any) -> dict[str, Any]:
    found: dict[str, Any] = {}

    def visit(name: str, item: Any) -> None:
        found[name] = item

    h5.visititems(visit)
    return found


def _content_difference(a: Path, b: Path) -> str | None:
    with h5py.File(a, "r") as first, h5py.File(b, "r") as second:
        difference = _attrs_difference("root", first.attrs, second.attrs)
        if difference is not None:
            return difference
        left = _objects(first)
        right = _objects(second)
        for path in sorted(set(left) | set(right)):
            if path not in right:
                return f"{path}: only in the first archive"
            if path not in left:
                return f"{path}: only in the second archive"
            difference = _attrs_difference(path, left[path].attrs, right[path].attrs)
            if difference is not None:
                return difference
            left_is_dataset = isinstance(left[path], h5py.Dataset)
            if left_is_dataset != isinstance(right[path], h5py.Dataset):
                return f"{path}: a dataset in one archive and a group in the other"
            if left_is_dataset:
                difference = _values_difference(path, left[path][...], right[path][...])
                if difference is not None:
                    return difference
    return None


def archives_identical(a: Path, b: Path) -> Report:
    first = a.read_bytes()
    second = b.read_bytes()
    if first == second:
        return Report(True, True, None)
    difference = _content_difference(a, b)
    if difference is not None:
        return Report(False, False, difference)
    common = min(len(first), len(second))
    left = np.frombuffer(first, dtype=np.uint8, count=common)
    right = np.frombuffer(second, dtype=np.uint8, count=common)
    differing = np.nonzero(left != right)[0]
    offset = common
    if differing.size > 0:
        offset = int(differing[0])
    return Report(False, True, f"file bytes differ at offset {offset}; recorded content identical")
