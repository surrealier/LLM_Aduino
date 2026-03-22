"""
USB 시리얼 연결 관리 모듈
- ESP32가 USB로 연결됐을 때 시리얼 포트를 자동 감지하여 접속
- socket 호환 인터페이스(SerialSocket)로 래핑하여 기존 handler 재사용
- TCP ConnectionManager와 병렬로 daemon thread에서 실행
"""
import logging
import socket
import time
from typing import Callable, Optional

log = logging.getLogger(__name__)

# ESP32/M5Stack에서 흔히 쓰는 USB-시리얼 칩 키워드
_ESP32_VID_PIDS = {
    (0x10C4, 0xEA60),  # CP2104 (M5Stack)
    (0x1A86, 0x7523),  # CH340
    (0x1A86, 0x55D4),  # CH9102
    (0x0403, 0x6001),  # FTDI FT232
    (0x239A, None),    # Adafruit / ESP32-S2/S3 native USB
    (0x303A, None),    # Espressif native USB
}

_ESP32_KEYWORDS = ["m5stack", "cp210", "ch340", "ch341", "ch9102",
                   "esp32", "esp32s", "ftdi", "uart", "usb serial"]


def _find_esp32_port() -> Optional[str]:
    """연결된 ESP32/M5Stack 시리얼 포트를 자동 감지."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None

    for port_info in list_ports.comports():
        desc = (port_info.description or "").lower()
        mfg  = (port_info.manufacturer or "").lower()
        name = (port_info.name or "").lower()

        # VID/PID 매칭
        vid = port_info.vid
        pid = port_info.pid
        if vid is not None:
            for (v, p) in _ESP32_VID_PIDS:
                if v == vid and (p is None or p == pid):
                    log.info("Serial: found ESP32 by VID/PID %04X:%04X at %s",
                             vid, pid or 0, port_info.device)
                    return port_info.device

        # 이름/설명 키워드 매칭
        combined = f"{desc} {mfg} {name}"
        for kw in _ESP32_KEYWORDS:
            if kw in combined:
                log.info("Serial: found ESP32 by keyword '%s' at %s", kw, port_info.device)
                return port_info.device

    return None


class SerialSocket:
    """
    serial.Serial을 socket.socket 호환 인터페이스로 래핑.
    recv_exact / send_packet 등 기존 프로토콜 코드를 그대로 재사용.
    """

    def __init__(self, ser):
        self._ser = ser

    # ── socket 호환 메서드 ──

    def recv(self, n: int) -> bytes:
        data = self._ser.read(n)
        if not data:
            raise socket.timeout("serial read timeout")
        return data

    def send(self, data: bytes) -> int:
        return self._ser.write(data)

    def settimeout(self, t: Optional[float]):
        self._ser.timeout = t if t is not None else 1.0

    def setsockopt(self, *args):
        pass  # no-op (TCP 전용)

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    def fileno(self):
        try:
            return self._ser.fileno()
        except Exception:
            return -1

    def __repr__(self):
        return f"<SerialSocket port={self._ser.port}>"


class SerialConnectionManager:
    """
    USB 시리얼 포트를 감지하고 연결을 수락하여 handler를 호출.
    TCP ConnectionManager와 동일한 handler 시그니처 사용:
        handler(conn: SerialSocket, addr: tuple)
    """

    def __init__(
        self,
        handler: Callable,
        baudrate: int = 921600,
        poll_interval: float = 2.0,
    ):
        self.handler = handler
        self.baudrate = baudrate
        self.poll_interval = poll_interval

    def accept_loop(self):
        """
        포트를 주기적으로 스캔하고, 연결되면 handler를 호출.
        handler 종료 후 다시 스캔 대기.
        """
        import serial

        log.info("Serial: scanning for ESP32 (baud=%d)...", self.baudrate)

        while True:
            port = _find_esp32_port()
            if not port:
                time.sleep(self.poll_interval)
                continue

            log.info("Serial: connecting to %s", port)
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=0.5,
                )
            except Exception as exc:
                log.warning("Serial: failed to open %s: %s", port, exc)
                time.sleep(self.poll_interval)
                continue

            sock = SerialSocket(ser)
            addr = ("serial", port)
            log.info("Serial: connected at %s", port)
            try:
                self.handler(sock, addr)
            except Exception as exc:
                log.exception("Serial: handler error: %s", exc)
            finally:
                sock.close()
                log.info("Serial: disconnected from %s", port)

            time.sleep(1.0)  # 재연결 전 잠시 대기
