# Shared fan-out queue used by the typed drivers to deliver received DALI
# traffic to multiple independent consumers.

from __future__ import annotations

import asyncio
from typing import Any


class DistributorQueue(asyncio.Queue[Any]):
    """An asyncio.Queue that fans one stream of items out to many consumers.

    Create one parent (no ``parent`` argument), then a child per consumer,
    passing the parent. ``distribute()`` on the parent enqueues the item on
    every child, so each consumer can ``await child.get()`` independently.
    Only one level of nesting is supported.
    """

    def __init__(
        self, parent: DistributorQueue | None = None, *, maxsize: int = 0
    ) -> None:
        super().__init__(maxsize=maxsize)
        self._handlers: dict[int, DistributorQueue] = {}
        self._parent = parent
        if self._parent is not None:
            self._parent.add_handler(self)

    @property
    def is_parent(self) -> bool:
        return self._parent is None

    def add_handler(self, handler: DistributorQueue) -> None:
        if not self.is_parent:
            raise RuntimeError(
                "Cannot add handler to DistributorQueue with a parent"
            )
        self._handlers[hash(handler)] = handler

    def del_handler(self, handler: DistributorQueue) -> None:
        self._handlers.pop(hash(handler), None)

    def distribute(self, item: Any) -> None:
        if self._parent is not None:
            self.put_nowait(item)
        for handler in self._handlers.values():
            handler.distribute(item)

    def __del__(self) -> None:
        if self._parent is not None:
            self._parent.del_handler(self)
