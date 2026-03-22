import socket
from types import SimpleNamespace

import pytest

from src.connection_manager import (
    AutoConnectionManager,
    ConnectionManager,
    SerialConnectionManager,
    SerialSocketAdapter,
    _normalize_connection_priority,
    _iter_serial_candidates,
    build_connection_manager,
)


def test_connection_manager_start():
    def handler(conn, addr):
        conn.close()

    manager = ConnectionManager("127.0.0.1", 0, handler)
    srv = manager.start()
    host, port = srv.getsockname()
    assert host in ("127.0.0.1", "0.0.0.0")
    assert port > 0
    srv.close()


class _FakeConfig:
    def __init__(self, data):
        self.data = data

    def get(self, *keys, default=None):
        value = self.data
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value


def test_build_connection_manager_wifi():
    cfg = _FakeConfig(
        {
            "server": {"host": "127.0.0.1", "port": 5001},
            "connection": {"mode": "wifi", "socket_timeout": 0.5},
        }
    )
    manager = build_connection_manager(cfg, lambda *_args: None)
    assert isinstance(manager, ConnectionManager)


def test_build_connection_manager_wired():
    cfg = _FakeConfig(
        {
            "server": {"host": "127.0.0.1", "port": 5001},
            "connection": {"mode": "wired", "socket_timeout": 0.5},
        }
    )
    manager = build_connection_manager(cfg, lambda *_args: None)
    assert isinstance(manager, SerialConnectionManager)


def test_build_connection_manager_auto():
    cfg = _FakeConfig(
        {
            "server": {"host": "127.0.0.1", "port": 5001},
            "connection": {"mode": "auto", "socket_timeout": 0.5},
        }
    )
    manager = build_connection_manager(cfg, lambda *_args: None)
    assert isinstance(manager, AutoConnectionManager)


def test_iter_serial_candidates_prefers_matching_usb_device(monkeypatch):
    fake_ports = [
        SimpleNamespace(device="/dev/cu.Bluetooth", description="Bluetooth-Incoming-Port", manufacturer="Apple"),
        SimpleNamespace(device="/dev/cu.SLAB_USBtoUART", description="CP2104 USB to UART Bridge", manufacturer="Silicon Labs", vid=0x10C4, pid=0xEA60),
    ]
    monkeypatch.setattr("src.connection_manager.serial", object())
    monkeypatch.setattr("src.connection_manager.list_ports", SimpleNamespace(comports=lambda: fake_ports))
    candidates = _iter_serial_candidates("auto")
    assert candidates[0].device == "/dev/cu.SLAB_USBtoUART"


class _FakeSerial:
    def __init__(self, reads=None, writes=None):
        self.reads = list(reads or [])
        self.writes = list(writes or [])
        self.timeout = None
        self.write_timeout = None
        self.closed = False
        self.flush_calls = 0

    def read(self, _n):
        if self.reads:
            return self.reads.pop(0)
        return b""

    def write(self, data):
        if self.writes:
            return self.writes.pop(0)
        return len(data)

    def flush(self):
        self.flush_calls += 1

    def close(self):
        self.closed = True


def test_serial_socket_adapter_sendall_flushes_payload():
    fake = _FakeSerial(writes=[2, 3])
    adapter = SerialSocketAdapter(fake, port_name="/dev/cu.test")
    adapter.sendall(b"hello")
    assert fake.flush_calls == 1


def test_serial_socket_adapter_initial_idle_timeout():
    fake = _FakeSerial(reads=[b""])
    adapter = SerialSocketAdapter(fake, port_name="/dev/cu.test", initial_idle_timeout_s=0)
    with pytest.raises(OSError):
        adapter.recv(1)


def test_serial_socket_adapter_uses_relaxed_write_timeout():
    fake = _FakeSerial()
    adapter = SerialSocketAdapter(fake, port_name="/dev/cu.test", timeout=0.5, write_timeout=5.0)
    assert fake.timeout == 0.5
    assert fake.write_timeout == 5.0


def test_normalize_connection_priority_preserves_order_and_backfills():
    assert _normalize_connection_priority("wifi > wired") == ["wifi", "wired"]
    assert _normalize_connection_priority(["wired"]) == ["wired", "wifi"]


def test_auto_connection_manager_uses_live_priority_order(monkeypatch):
    manager = AutoConnectionManager(
        "127.0.0.1",
        5001,
        lambda *_args: None,
        connection_priority_provider=lambda: ["wifi", "wired"],
    )
    calls = []
    expected = (object(), ("127.0.0.1", 5001))

    monkeypatch.setattr(manager.serial_manager, "try_accept", lambda: calls.append("wired") or None)
    monkeypatch.setattr(manager, "_try_accept_tcp", lambda: calls.append("wifi") or expected)

    assert manager._accept_next_candidate() == expected
    assert calls == ["wifi"]
