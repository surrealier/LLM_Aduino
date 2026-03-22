import socket
import struct
import threading
import time
from unittest import mock

import numpy as np

from src import protocol


def read_packet(sock):
    ptype = sock.recv(1)
    if not ptype:
        return None, None
    length = sock.recv(2)
    plen = int.from_bytes(length, "little")
    payload = sock.recv(plen) if plen else b""
    return ptype[0], payload


def test_send_packet_basic():
    s1, s2 = socket.socketpair()
    try:
        payload = b"abc"
        ok = protocol.send_packet(s1, protocol.PTYPE_CMD, payload)
        assert ok
        ptype, recv_payload = read_packet(s2)
        assert ptype == protocol.PTYPE_CMD
        assert recv_payload == payload
    finally:
        s1.close()
        s2.close()


def test_send_packet_audio_even_bytes():
    s1, s2 = socket.socketpair()
    try:
        payload = b"\x01\x02\x03"  # odd length, last byte should be dropped
        ok = protocol.send_packet(s1, protocol.PTYPE_AUDIO_OUT, payload, audio_chunk=4, audio_sleep_s=0)
        assert ok
        ptype, recv_payload = read_packet(s2)
        assert ptype == protocol.PTYPE_AUDIO_OUT
        assert len(recv_payload) == 2
    finally:
        s1.close()
        s2.close()


def test_send_packet_audio_allows_one_byte_samples():
    s1, s2 = socket.socketpair()
    try:
        payload = b"\x81\x7f\x00"
        ok = protocol.send_packet(
            s1,
            protocol.PTYPE_AUDIO_OUT,
            payload,
            audio_chunk=4,
            audio_sleep_s=0,
            audio_bytes_per_sample=1,
        )
        assert ok
        ptype, recv_payload = read_packet(s2)
        assert ptype == protocol.PTYPE_AUDIO_OUT
        assert recv_payload == payload
    finally:
        s1.close()
        s2.close()


def test_send_audio_splits_default_packets_to_2kb_or_less():
    s1, s2 = socket.socketpair()
    try:
        payload = b"\x00\x01" * 2500
        ok = protocol.send_audio(s1, payload, audio_sleep_s=0)
        assert ok

        packets = []
        total = 0
        while total < len(payload):
            ptype, recv_payload = read_packet(s2)
            packets.append((ptype, recv_payload))
            total += len(recv_payload)

        assert total == len(payload)
        assert all(ptype == protocol.PTYPE_AUDIO_OUT for ptype, _payload in packets)
        assert all(len(recv_payload) <= 2048 for _ptype, recv_payload in packets)
    finally:
        s1.close()
        s2.close()


def test_send_audio_uses_smaller_chunks_for_serial_links():
    fake_conn = mock.Mock()
    fake_conn.serial = object()
    fake_conn.port_name = "/dev/cu.test"

    with mock.patch.object(protocol, "send_packet", return_value=True) as send_packet:
        ok = protocol.send_audio(fake_conn, b"\x00\x01" * 1000)

    assert ok is True
    assert send_packet.call_args.kwargs["audio_chunk"] == 512
    assert send_packet.call_args.kwargs["audio_sample_rate"] == 8000
    assert send_packet.call_args.kwargs["audio_bytes_per_sample"] == 1
    assert len(send_packet.call_args.args[2]) == 500


def test_encode_serial_audio_payload_downsamples_to_wired_codec():
    pcm16 = b"\x00\x80\x00\x80\x00\x00\xff\x7f"
    encoded = protocol._encode_serial_audio_payload(pcm16)
    assert len(encoded) == 2
    decoded = protocol._decode_serial_audio_payload(encoded)
    restored = np.frombuffer(decoded, dtype=np.int16)
    assert restored.shape == (4,)
    assert restored[0] < -1000
    assert restored[1] < -1000
    assert restored[2] > 1000
    assert restored[3] > 1000


def test_recv_packet_decodes_serial_audio_payload_back_to_pcm16():
    class _FakeSerialConn:
        def __init__(self, data: bytes):
            self._data = bytearray(data)
            self.serial = object()
            self.port_name = "/dev/cu.test"

        def recv(self, n: int) -> bytes:
            if not self._data:
                return b""
            take = min(n, len(self._data))
            chunk = bytes(self._data[:take])
            del self._data[:take]
            return chunk

    pcm = (np.array([-30000, -30000, 12000, 12000], dtype=np.int16)).tobytes()
    encoded = protocol._encode_serial_audio_payload(pcm)
    packet_bytes = struct.pack("<BH", protocol.PTYPE_AUDIO, len(encoded)) + encoded

    packet = protocol.recv_packet(_FakeSerialConn(packet_bytes), header_timeouts=1, payload_timeouts=1)

    assert packet is not None
    ptype, payload = packet
    assert ptype == protocol.PTYPE_AUDIO
    restored = np.frombuffer(payload, dtype=np.int16)
    assert restored.shape == (4,)
    assert restored[0] < -1000
    assert restored[1] < -1000
    assert restored[2] > 1000
    assert restored[3] > 1000


def test_recv_packet_skips_garbage_before_ping():
    s1, s2 = socket.socketpair()
    try:
        s2.settimeout(0.1)
        s1.sendall(b"boot\r\n" + struct.pack("<BH", protocol.PTYPE_PING, 0))

        packet = protocol.recv_packet(s2, header_timeouts=2, payload_timeouts=1)

        assert packet == (protocol.PTYPE_PING, b"")
    finally:
        s1.close()
        s2.close()


def test_recv_packet_resyncs_after_incomplete_false_header():
    s1, s2 = socket.socketpair()
    sender = None
    try:
        s2.settimeout(0.05)

        def _send_script():
            s1.sendall(struct.pack("<BH", protocol.PTYPE_AUDIO, 6) + b"\x01\x02")
            time.sleep(0.08)
            s1.sendall(struct.pack("<BH", protocol.PTYPE_PING, 0))

        sender = threading.Thread(target=_send_script, daemon=True)
        sender.start()

        packet = protocol.recv_packet(s2, header_timeouts=4, payload_timeouts=1)

        assert packet == (protocol.PTYPE_PING, b"")
    finally:
        if sender is not None:
            sender.join(timeout=1)
        s1.close()
        s2.close()
