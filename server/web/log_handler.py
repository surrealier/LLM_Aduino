"""
In-process log ring buffer for the web dashboard.
Attaches a logging.Handler to the root logger that writes to a deque.
The WebSocket broadcaster reads from this deque for live log streaming.
"""
import asyncio
import collections
import logging
from typing import Deque, Optional

_ring: Deque[str] = collections.deque(maxlen=200)
_loop: Optional[asyncio.AbstractEventLoop] = None
_broadcast_fn = None  # set by app.py after uvicorn starts


class RingBufferHandler(logging.Handler):
    def emit(self, record: logging.LogRecord):
        try:
            line = self.format(record)
            _ring.append(line)
            # Broadcast to WebSocket clients if event loop is available
            if _broadcast_fn is not None and _loop is not None and _loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    _broadcast_fn({"event": "log", "line": line}),
                    _loop,
                )
        except Exception:
            pass


def get_log_ring() -> Deque[str]:
    return _ring


def set_loop(loop: asyncio.AbstractEventLoop):
    global _loop
    _loop = loop


def set_broadcast(fn):
    global _broadcast_fn
    _broadcast_fn = fn


def install(max_lines: int = 200):
    """Attach the ring buffer handler to the root logger. Call after setup_logging()."""
    global _ring
    _ring = collections.deque(maxlen=max_lines)
    handler = RingBufferHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    handler.setLevel(logging.DEBUG)
    logging.getLogger().addHandler(handler)
