"""
TCP 서버 연결 관리 모듈
- ESP32 클라이언트와의 TCP 소켓 연결을 관리
- 연결 수락, 핸들러 호출, 에러 처리 담당
"""
from __future__ import annotations

import logging
import select
import socket
import time
from types import SimpleNamespace
from typing import Callable, Optional, Tuple

try:
    import serial
    from serial import SerialException, SerialTimeoutException
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised via runtime environment
    serial = None
    SerialException = OSError
    SerialTimeoutException = TimeoutError
    list_ports = None

log = logging.getLogger(__name__)

_SERIAL_HINTS = (
    "m5",
    "atom",
    "echo",
    "esp32",
    "espressif",
    "cp210",
    "silicon labs",
    "usb serial",
    "usb-serial",
    "wch",
    "ch340",
    "slab",
)

_DEFAULT_CONNECTION_PRIORITY = ("wired", "wifi")


def _normalize_connection_priority(value) -> list[str]:
    allowed = {"wired", "wifi"}
    if isinstance(value, str):
        raw_items = [item.strip().lower() for item in value.replace(">", ",").split(",") if item.strip()]
    elif isinstance(value, (list, tuple)):
        raw_items = [str(item).strip().lower() for item in value if str(item).strip()]
    else:
        raw_items = []

    normalized: list[str] = []
    for item in raw_items:
        if item in allowed and item not in normalized:
            normalized.append(item)

    for item in _DEFAULT_CONNECTION_PRIORITY:
        if item not in normalized:
            normalized.append(item)
    return normalized


class SerialSocketAdapter:
    """Expose a pyserial connection through the subset of the socket API we use."""

    def __init__(
        self,
        serial_conn,
        port_name: str,
        timeout: float = 0.5,
        write_timeout: float = 5.0,
        initial_idle_timeout_s: float = 5.0,
    ):
        self.serial = serial_conn
        self.port_name = port_name
        self.timeout = timeout
        self.write_timeout = max(float(write_timeout), float(timeout))
        self._prebuffer = bytearray()
        self._saw_inbound_data = False
        self._initial_idle_deadline = time.monotonic() + max(0.5, float(initial_idle_timeout_s))
        self.settimeout(timeout)

    def settimeout(self, timeout: float):
        self.timeout = timeout
        self.serial.timeout = timeout
        self.serial.write_timeout = max(self.write_timeout, timeout)

    def setsockopt(self, *_args, **_kwargs):
        return None

    def prime(self, data: bytes):
        if data:
            self._prebuffer.extend(data)

    def recv(self, n: int) -> bytes:
        if n <= 0:
            return b""

        chunk = bytearray()
        if self._prebuffer:
            take = min(n, len(self._prebuffer))
            chunk.extend(self._prebuffer[:take])
            del self._prebuffer[:take]
            if len(chunk) >= n:
                self._saw_inbound_data = True
                return bytes(chunk)

        remaining = n - len(chunk)
        try:
            incoming = self.serial.read(remaining)
        except SerialException as exc:
            raise OSError(f"serial read failed on {self.port_name}: {exc}") from exc

        if incoming:
            chunk.extend(incoming)
            self._saw_inbound_data = True
            return bytes(chunk)

        if chunk:
            self._saw_inbound_data = True
            return bytes(chunk)

        if not self._saw_inbound_data and time.monotonic() >= self._initial_idle_deadline:
            raise OSError(f"serial connection idle during initial handshake: {self.port_name}")

        raise socket.timeout("serial read timeout")

    def sendall(self, data: bytes):
        view = memoryview(data)
        while view:
            try:
                written = self.serial.write(view)
            except (SerialException, SerialTimeoutException) as exc:
                raise OSError(f"serial write failed on {self.port_name}: {exc}") from exc
            if not written:
                raise OSError(f"serial write returned 0 on {self.port_name}")
            view = view[written:]
        try:
            self.serial.flush()
        except (SerialException, SerialTimeoutException) as exc:
            raise OSError(f"serial flush failed on {self.port_name}: {exc}") from exc

    def close(self):
        try:
            self.serial.close()
        except Exception:
            pass


def _serial_candidate_score(port_info) -> int:
    score = 0
    text = " ".join(
        str(getattr(port_info, attr, "") or "")
        for attr in ("device", "description", "manufacturer", "product", "hwid")
    ).lower()
    for hint in _SERIAL_HINTS:
        if hint in text:
            score += 2

    vid = getattr(port_info, "vid", None)
    pid = getattr(port_info, "pid", None)
    if (vid, pid) == (0x10C4, 0xEA60):  # Silicon Labs CP210x
        score += 6
    if vid == 0x303A:  # Espressif USB/JTAG/Serial
        score += 6
    if (vid, pid) == (0x1A86, 0x7523):  # CH340
        score += 4
    return score


def _iter_serial_candidates(port_name: str):
    if serial is None or list_ports is None:
        return []

    requested = (port_name or "auto").strip()
    if requested and requested.lower() != "auto":
        return [SimpleNamespace(device=requested, description="configured serial port")]

    ports = list(list_ports.comports())
    if not ports:
        return []

    scored = sorted(
        ((_serial_candidate_score(port_info), port_info) for port_info in ports),
        key=lambda item: (-item[0], getattr(item[1], "device", "")),
    )
    preferred = [port_info for score, port_info in scored if score > 0]
    if preferred:
        return preferred
    if len(ports) == 1:
        return ports
    return []
class ConnectionManager:
    """TCP 서버 연결 관리자 클래스"""
    
    def __init__(
        self,
        host: str,
        port: int,
        handler: Callable[[socket.socket, Tuple[str, int]], None],
        backlog: int = 5,
        accept_backoff: float = 1.0,
    ):
        # 서버 설정 초기화
        self.host = host
        self.port = port
        self.handler = handler
        self.backlog = backlog
        self.accept_backoff = accept_backoff
        self.server_socket = None

    def start(self):
        # TCP 서버 소켓 생성 및 바인딩
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(self.backlog)
        self.server_socket = srv
        log.info("Server listening on %s:%s", self.host, self.port)
        return srv

    def accept_loop(self):
        # 클라이언트 연결 수락 루프
        if self.server_socket is None:
            self.start()

        while True:
            log.info("Ready for next connection...")
            try:
                # 클라이언트 연결 대기 및 수락
                conn, addr = self.server_socket.accept()
            except KeyboardInterrupt:
                break
            except Exception as exc:
                log.error("Accept failed: %s", exc)
                time.sleep(self.accept_backoff)
                continue

            try:
                # 연결 핸들러 호출
                self.handler(conn, addr)
            except Exception as exc:
                log.exception("Connection handler error: %s", exc)
            finally:
                # 연결 정리
                try:
                    conn.close()
                except Exception:
                    pass
                log.info("Disconnected: %s", addr)
                time.sleep(0.1)


class SerialConnectionManager:
    """USB serial 연결 관리자."""

    def __init__(
        self,
        handler: Callable[[SerialSocketAdapter, Tuple[str, str]], None],
        port: str = "auto",
        baudrate: int = 115200,
        scan_interval: float = 1.0,
        initial_idle_timeout_s: float = 5.0,
    ):
        self.handler = handler
        self.port = port
        self.baudrate = baudrate
        self.scan_interval = scan_interval
        self.initial_idle_timeout_s = initial_idle_timeout_s
        self._retry_after: dict[str, float] = {}

    def _open(self, port_name: str) -> SerialSocketAdapter:
        if serial is None:
            raise RuntimeError("pyserial is required for wired USB mode")

        ser = serial.Serial(
            port=port_name,
            baudrate=self.baudrate,
            timeout=0.5,
            write_timeout=0.5,
        )
        try:
            if hasattr(ser, "dtr"):
                ser.dtr = False
            if hasattr(ser, "rts"):
                ser.rts = False
        except Exception:
            pass

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        return SerialSocketAdapter(
            ser,
            port_name=port_name,
            timeout=0.5,
            write_timeout=5.0,
            initial_idle_timeout_s=self.initial_idle_timeout_s,
        )

    def try_accept(self) -> Optional[tuple[SerialSocketAdapter, Tuple[str, str]]]:
        now = time.monotonic()
        for port_info in _iter_serial_candidates(self.port):
            port_name = getattr(port_info, "device", str(port_info))
            if self._retry_after.get(port_name, 0.0) > now:
                continue
            try:
                conn = self._open(port_name)
                log.info("Wired device candidate detected on %s (%s)", port_name, getattr(port_info, "description", "unknown"))
                return conn, ("serial", port_name)
            except Exception as exc:
                self._retry_after[port_name] = now + self.scan_interval
                log.warning("Failed to open serial port %s: %s", port_name, exc)
        return None

    def accept_loop(self):
        if serial is None:
            raise RuntimeError("pyserial is required for wired USB mode")

        waiting_logged = False
        while True:
            if not waiting_logged:
                log.info("Waiting for wired USB device...")
                waiting_logged = True
            result = self.try_accept()
            if result is None:
                time.sleep(self.scan_interval)
                continue

            conn, addr = result
            waiting_logged = False
            try:
                self.handler(conn, addr)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                log.exception("Wired connection handler error: %s", exc)
            finally:
                conn.close()
                log.info("Disconnected: %s", addr)
                time.sleep(0.1)


class AutoConnectionManager:
    """Serial auto-detect를 우선 시도하고, 없으면 TCP도 동시에 수락한다."""

    def __init__(
        self,
        host: str,
        port: int,
        handler: Callable,
        backlog: int = 5,
        accept_backoff: float = 1.0,
        serial_port: str = "auto",
        serial_baudrate: int = 115200,
        serial_scan_interval: float = 1.0,
        serial_initial_idle_timeout_s: float = 5.0,
        connection_priority_provider: Callable[[], list[str]] | None = None,
        poll_interval: float = 0.1,
    ):
        self.tcp_manager = ConnectionManager(host, port, handler, backlog=backlog, accept_backoff=accept_backoff)
        self.serial_manager = SerialConnectionManager(
            handler,
            port=serial_port,
            baudrate=serial_baudrate,
            scan_interval=serial_scan_interval,
            initial_idle_timeout_s=serial_initial_idle_timeout_s,
        )
        self.accept_backoff = accept_backoff
        self.connection_priority_provider = connection_priority_provider or (lambda: list(_DEFAULT_CONNECTION_PRIORITY))
        self.poll_interval = max(0.02, float(poll_interval))

    def start(self):
        srv = self.tcp_manager.start()
        srv.setblocking(False)
        return srv

    def _current_connection_priority(self) -> list[str]:
        return _normalize_connection_priority(self.connection_priority_provider())

    def _try_accept_tcp(self):
        server_socket = self.tcp_manager.server_socket
        if server_socket is None:
            return None

        ready, _, _ = select.select([server_socket], [], [], 0.0)
        if not ready:
            return None
        return server_socket.accept()

    def _accept_next_candidate(self):
        for transport in self._current_connection_priority():
            if transport == "wired":
                serial_result = self.serial_manager.try_accept()
                if serial_result is not None:
                    return serial_result
                continue
            if transport == "wifi":
                tcp_result = self._try_accept_tcp()
                if tcp_result is not None:
                    return tcp_result
        return None

    def accept_loop(self):
        if self.tcp_manager.server_socket is None:
            self.start()

        waiting_logged = False
        last_priority = None
        while True:
            current_priority = self._current_connection_priority()
            if not waiting_logged or current_priority != last_priority:
                label = " > ".join(item.upper() for item in current_priority)
                log.info("Ready for next connection (auto priority: %s)...", label)
                waiting_logged = True
                last_priority = current_priority
            try:
                accepted = self._accept_next_candidate()
                if accepted is None:
                    time.sleep(self.poll_interval)
                    continue

                conn, addr = accepted
            except BlockingIOError:
                time.sleep(self.poll_interval)
                continue
            except KeyboardInterrupt:
                break
            except Exception as exc:
                log.error("Accept failed: %s", exc)
                time.sleep(self.accept_backoff)
                continue

            try:
                waiting_logged = False
                self.tcp_manager.handler(conn, addr)
            except Exception as exc:
                log.exception("Connection handler error: %s", exc)
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
                log.info("Disconnected: %s", addr)
                time.sleep(0.1)


def build_connection_manager(
    config,
    handler: Callable,
    connection_priority_provider: Callable[[], list[str]] | None = None,
):
    mode = (config.get("connection", "mode", default="auto") or "auto").lower()
    host = config.get("server", "host")
    port = config.get("server", "port")
    socket_timeout = float(config.get("connection", "socket_timeout", default=0.5))
    serial_port = config.get("connection", "serial_port", default="auto")
    serial_baudrate = int(config.get("connection", "serial_baudrate", default=115200))
    serial_scan_interval = float(config.get("connection", "serial_scan_interval", default=1.0))
    serial_initial_idle_timeout_s = float(
        config.get("connection", "serial_initial_idle_timeout_s", default=max(5.0, socket_timeout * 10))
    )
    connection_priority = config.get("connection", "priority", default=list(_DEFAULT_CONNECTION_PRIORITY))

    if mode == "wired":
        return SerialConnectionManager(
            handler,
            port=serial_port,
            baudrate=serial_baudrate,
            scan_interval=serial_scan_interval,
            initial_idle_timeout_s=serial_initial_idle_timeout_s,
        )
    if mode == "wifi":
        return ConnectionManager(host, port, handler)
    return AutoConnectionManager(
        host=host,
        port=port,
        handler=handler,
        serial_port=serial_port,
        serial_baudrate=serial_baudrate,
        serial_scan_interval=serial_scan_interval,
        serial_initial_idle_timeout_s=serial_initial_idle_timeout_s,
        connection_priority_provider=connection_priority_provider
        or (lambda: _normalize_connection_priority(connection_priority)),
    )
