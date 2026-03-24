"""
ccoli web dashboard package.
Call start_web_server() from server.py main() to launch the dashboard.
"""
import threading
from typing import Callable

from .app import create_app


def start_web_server(
    agent_fn: Callable,
    robot_fn: Callable,
    mode_fn: Callable,
    dashboard_state_fn: Callable | None = None,
    host: str = "0.0.0.0",
    port: int = 8005,
) -> threading.Thread:
    import uvicorn

    app = create_app(agent_fn, robot_fn, mode_fn, dashboard_state_fn)
    config = uvicorn.Config(
        app,
        host=host,
        port=port,
        log_level="warning",
        loop="asyncio",
    )
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True, name="web-server")
    thread.start()
    return thread
