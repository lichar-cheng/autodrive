from __future__ import annotations

import asyncio
from collections import defaultdict
from dataclasses import dataclass
from typing import AsyncIterator, Dict, Set


@dataclass
class TopicStats:
    published: int = 0
    dropped: int = 0


class TopicBus:
    def __init__(self, queue_size: int = 200) -> None:
        self._subscribers: Dict[str, Set[asyncio.Queue]] = defaultdict(set)
        self._queue_size = queue_size
        self._stats: Dict[str, TopicStats] = defaultdict(TopicStats)

    async def publish(self, topic: str, message: dict) -> None:
        self._stats[topic].published += 1
        for q in list(self._subscribers[topic]):
            # Never block publishers on slow consumers. Otherwise a single
            # stalled websocket subscriber can backpressure the whole event
            # loop and make the server appear "frozen" (including slow Ctrl+C).
            while True:
                try:
                    q.put_nowait(message)
                    break
                except asyncio.QueueFull:
                    try:
                        q.get_nowait()
                        self._stats[topic].dropped += 1
                    except asyncio.QueueEmpty:
                        # Consumer raced and emptied queue; retry put_nowait.
                        continue

    async def subscribe(self, topic: str) -> AsyncIterator[dict]:
        q: asyncio.Queue = asyncio.Queue(maxsize=self._queue_size)
        self._subscribers[topic].add(q)
        try:
            while True:
                yield await q.get()
        finally:
            self._subscribers[topic].discard(q)

    def list_topics(self) -> list[str]:
        return sorted(self._subscribers.keys())

    def stats(self) -> dict:
        return {
            t: {
                "published": s.published,
                "dropped": s.dropped,
                "subscribers": len(self._subscribers[t]),
            }
            for t, s in self._stats.items()
        }
