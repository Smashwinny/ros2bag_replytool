#!/usr/bin/env python3
"""Epoch and immutable display-history primitives for deterministic replay."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Generic, Iterable, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class ReplayEpoch:
    epoch: int
    target_ns: int
    first_pass_complete: bool


class ReplayEpochClock:
    """Creates strictly increasing replay epochs for every timeline jump."""

    def __init__(self) -> None:
        self._epoch = 0

    def next(self, target_ns: int, first_pass_complete: bool) -> ReplayEpoch:
        if target_ns < 0:
            raise ValueError("target_ns must be non-negative")
        self._epoch += 1
        return ReplayEpoch(self._epoch, target_ns, first_pass_complete)

    @property
    def current(self) -> int:
        return self._epoch


class ImmutableTimeline(Generic[T]):
    """Append-only canonical history with disposable cutoff views.

    Items are de-duplicated by an externally supplied stable key. Rendering
    never mutates the canonical history, so a backward seek followed by a
    forward seek restores the exact previously observed prefix.
    """

    def __init__(self, stamp_ns: Callable[[T], int], key: Callable[[T], object]):
        self._stamp_ns = stamp_ns
        self._key = key
        self._items: list[T] = []
        self._keys: set[object] = set()
        self._sealed = False

    def append(self, item: T) -> bool:
        if self._sealed:
            return False
        key = self._key(item)
        if key in self._keys:
            return False
        self._keys.add(key)
        self._items.append(item)
        return True

    def seal(self) -> None:
        self._sealed = True

    def view(self, cutoff_ns: int | None) -> list[T]:
        if cutoff_ns is None:
            return list(self._items)
        return [item for item in self._items
                if self._stamp_ns(item) <= cutoff_ns]

    def replace_for_test(self, items: Iterable[T]) -> None:
        if self._items or self._sealed:
            raise RuntimeError("canonical timeline is already initialized")
        for item in items:
            self.append(item)

    @property
    def sealed(self) -> bool:
        return self._sealed

    @property
    def size(self) -> int:
        return len(self._items)
