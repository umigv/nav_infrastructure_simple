from __future__ import annotations

from dataclasses import dataclass, field

import pytest
from nav_utils.config import load


class Param:
    def __init__(self, value):
        self.value = value


class MockNode:
    def __init__(self, initial: dict[str, object] | None = None):
        self._store: dict[str, object] = dict(initial or {})
        self.declared: list[tuple[str, object | None]] = []

    def get_name(self) -> str:
        return "MockNode"

    def declare_parameter(self, key: str, default_value=None):
        self.declared.append((key, default_value))
        if key not in self._store:
            self._store[key] = default_value

    def get_parameter(self, key: str) -> Param:
        return Param(self._store.get(key, None))


def test_load_required_param_success():
    @dataclass
    class Config:
        rate: int

    node = MockNode(initial={"rate": 10})
    config = load(node, Config)

    assert ("rate", None) in node.declared
    assert config.rate == 10


def test_load_required_param_missing_raises():
    @dataclass
    class Config:
        rate: int

    node = MockNode(initial={})
    with pytest.raises(RuntimeError, match=r"Required parameter 'rate' not set"):
        load(node, Config)


def test_load_default_is_used_if_missing():
    @dataclass
    class Config:
        rate: int = 20

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.rate == 20


def test_load_default_overridden_if_present():
    @dataclass
    class Config:
        rate: int = 20

    node = MockNode(initial={"rate": 7})
    config = load(node, Config)

    assert config.rate == 7


def test_load_default_factory():
    @dataclass
    class Config:
        ids: list[int] = field(default_factory=lambda: [1, 2, 3])

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.ids == [1, 2, 3]


@dataclass
class Inner:
    gain: float = 0.5
    name: str = "abc"


@dataclass
class Outer:
    inner: Inner
    rate: int = 10


def test_load_nested_dataclass():
    node = MockNode(
        initial={
            "inner.gain": 1.25,
            "rate": 42,
        }
    )

    config = load(node, Outer)

    assert config.rate == 42
    assert config.inner.gain == 1.25
    assert config.inner.name == "abc"
