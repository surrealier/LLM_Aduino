"""Tests for server.py orchestration functions (pure + mock-based)."""
import builtins
import signal
import sys
import unittest.mock as mock

import yaml


# server.py has heavy top-level imports (STTEngine → faster_whisper).
# We pre-mock the unavailable modules so we can import the pure helpers.
_HEAVY = [
    "faster_whisper",
    "faster_whisper.transcribe",
]
for _m in _HEAVY:
    if _m not in sys.modules:
        sys.modules[_m] = mock.MagicMock()

import server as srv  # noqa: E402  (after mocking)
from src.connection_manager import AutoConnectionManager, ConnectionManager, SerialConnectionManager  # noqa: E402


# ── _normalize_start_command (pure) ──────────────────────────

def test_normalize_string():
    assert srv._normalize_start_command("ollama serve") == ["ollama", "serve"]


def test_normalize_list():
    assert srv._normalize_start_command(["a", "b"]) == ["a", "b"]


def test_normalize_none():
    assert srv._normalize_start_command(None) is None


def test_normalize_empty():
    assert srv._normalize_start_command("") is None


# ── _ollama_health_check ─────────────────────────────────────

def test_health_check_success():
    resp = mock.MagicMock()
    resp.status = 200
    cm = mock.MagicMock()
    cm.__enter__ = mock.Mock(return_value=resp)
    cm.__exit__ = mock.Mock(return_value=False)
    with mock.patch("server.urllib.request.urlopen", return_value=cm):
        assert srv._ollama_health_check("http://localhost:11434") is True


def test_health_check_failure():
    with mock.patch("server.urllib.request.urlopen", side_effect=OSError):
        assert srv._ollama_health_check("http://localhost:11434") is False


# ── ensure_ollama_running ────────────────────────────────────

def test_ensure_already_up():
    with mock.patch("server._ollama_health_check", return_value=True):
        assert srv.ensure_ollama_running("http://localhost:11434", {}) is True


def test_ensure_auto_start_disabled():
    with mock.patch("server._ollama_health_check", return_value=False):
        assert srv.ensure_ollama_running("http://localhost:11434", {"auto_start": False}) is False


def test_start_web_dashboard_returns_local_and_lan_urls():
    with mock.patch.object(srv, "_discover_local_ip_addresses", return_value=["192.168.0.24"]):
        urls = srv._start_web_dashboard(
            {
                "enabled": True,
                "host": "0.0.0.0",
                "port": 8005,
                "auth_token": "",
                "log_tail_lines": 50,
            },
            agent_fn=lambda: None,
            robot_fn=lambda: None,
            mode_fn=lambda: "agent",
            start_web_server_fn=lambda **kwargs: mock.sentinel.thread,
            configure_auth_fn=lambda token: None,
            install_log_handler_fn=lambda max_lines: None,
        )

    assert urls == [
        "http://localhost:8005",
        "http://192.168.0.24:8005",
    ]


def test_start_web_dashboard_skips_optional_dependency_error():
    def _missing(**kwargs):
        raise ModuleNotFoundError("No module named 'uvicorn'", name="uvicorn")

    urls = srv._start_web_dashboard(
        {
            "enabled": True,
            "host": "127.0.0.1",
            "port": 8005,
            "auth_token": "",
            "log_tail_lines": 50,
        },
        agent_fn=lambda: None,
        robot_fn=lambda: None,
        mode_fn=lambda: "agent",
        start_web_server_fn=_missing,
        configure_auth_fn=lambda token: None,
        install_log_handler_fn=lambda max_lines: None,
    )

    assert urls == []


# ── load_commands_config ─────────────────────────────────────

def test_load_commands_valid(tmp_path):
    p = tmp_path / "cmds.yaml"
    p.write_text(yaml.dump({"commands": [{"name": "nod"}]}))
    srv.load_commands_config(str(p))
    assert srv.ACTIONS_CONFIG == [{"name": "nod"}]


def test_load_commands_missing():
    srv.load_commands_config("/nonexistent/path.yaml")
    assert srv.ACTIONS_CONFIG == []


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


def test_build_connection_manager_modes():
    base = {"server": {"host": "127.0.0.1", "port": 5001}, "connection": {"socket_timeout": 0.5}}
    wifi_cfg = _FakeConfig({**base, "connection": {"mode": "wifi", "socket_timeout": 0.5}})
    wired_cfg = _FakeConfig({**base, "connection": {"mode": "wired", "socket_timeout": 0.5}})
    auto_cfg = _FakeConfig({**base, "connection": {"mode": "auto", "socket_timeout": 0.5}})

    assert isinstance(srv.build_connection_manager(wifi_cfg, lambda *_: None), ConnectionManager)
    assert isinstance(srv.build_connection_manager(wired_cfg, lambda *_: None), SerialConnectionManager)
    assert isinstance(srv.build_connection_manager(auto_cfg, lambda *_: None), AutoConnectionManager)


def test_send_connection_greeting_sends_audio_once():
    class _FakeAgent:
        def __init__(self):
            self.greeting_calls = 0
            self.tts_calls = 0

        def generate_connection_greeting(self):
            self.greeting_calls += 1
            return "콜리 연결됐어요! 아직 일하고 계세요?"

        def text_to_audio(self, text, trim_pad_ms=140.0):
            self.tts_calls += 1
            assert "콜리 연결됐어요!" in text
            assert trim_pad_ms == srv.CONNECTION_GREETING_TTS_PAD_MS
            return b"\x00\x01" * 32

    state = {"connection_greeting_sent": False}
    agent = _FakeAgent()

    with (
        mock.patch.object(srv, "current_mode", "agent"),
        mock.patch.object(srv, "send_action", return_value=True) as send_action,
        mock.patch.object(srv, "send_audio", return_value=True) as send_audio,
    ):
        assert srv._send_connection_greeting(mock.sentinel.conn, mock.sentinel.lock, agent, state) is True
        assert srv._send_connection_greeting(mock.sentinel.conn, mock.sentinel.lock, agent, state) is False

    send_audio.assert_called_once()
    send_action.assert_called_once_with(mock.sentinel.conn, {"action": "MIC_LOCK"}, mock.sentinel.lock)
    assert state["connection_greeting_sent"] is True
    assert agent.greeting_calls == 1
    assert agent.tts_calls == 1


def test_send_connection_greeting_skips_when_input_stream_active():
    class _FakeAgent:
        def generate_connection_greeting(self):
            raise AssertionError("should not generate greeting while input is active")

        def text_to_audio(self, _text, trim_pad_ms=140.0):
            raise AssertionError("should not synthesize greeting while input is active")

    gate = srv.InputGate()
    assert gate.start_stream() is True

    with (
        mock.patch.object(srv, "current_mode", "agent"),
        mock.patch.object(srv, "send_action") as send_action,
        mock.patch.object(srv, "send_audio") as send_audio,
    ):
        assert srv._send_connection_greeting(
            mock.sentinel.conn,
            mock.sentinel.lock,
            _FakeAgent(),
            {"connection_greeting_sent": False},
            gate,
        ) is False

    send_action.assert_not_called()
    send_audio.assert_not_called()


def test_send_connection_greeting_unlocks_when_audio_send_fails():
    class _FakeAgent:
        def generate_connection_greeting(self):
            return "콜리 연결됐어요!"

        def text_to_audio(self, text, trim_pad_ms=140.0):
            assert "콜리 연결됐어요!" in text
            return b"\x00\x01" * 32

    calls = []

    def _record_send_action(conn, action_dict, lock=None):
        calls.append(action_dict)
        return True

    with (
        mock.patch.object(srv, "current_mode", "agent"),
        mock.patch.object(srv, "send_action", side_effect=_record_send_action),
        mock.patch.object(srv, "send_audio", return_value=False),
    ):
        assert srv._send_connection_greeting(
            mock.sentinel.conn,
            mock.sentinel.lock,
            _FakeAgent(),
            {"connection_greeting_sent": False},
        ) is False

    assert calls == [{"action": "MIC_LOCK"}, {"action": "MIC_UNLOCK"}]


def test_send_tts_chunks_locks_once_and_sends_all_audio():
    calls = []

    def _record_send_action(conn, action_dict, lock=None):
        calls.append(action_dict)
        return True

    with (
        mock.patch.object(srv, "send_action", side_effect=_record_send_action),
        mock.patch.object(srv, "send_audio", return_value=True) as send_audio,
    ):
        ok = srv._send_tts_chunks(
            mock.sentinel.conn,
            mock.sentinel.lock,
            [b"\x00\x01" * 8, b"\x00\x01" * 4],
        )

    assert ok is True
    assert calls == [{"action": "MIC_LOCK"}]
    assert send_audio.call_count == 2


def test_send_tts_chunks_unlocks_when_audio_send_fails():
    calls = []

    def _record_send_action(conn, action_dict, lock=None):
        calls.append(action_dict)
        return True

    with (
        mock.patch.object(srv, "send_action", side_effect=_record_send_action),
        mock.patch.object(srv, "send_audio", side_effect=[True, False]),
    ):
        ok = srv._send_tts_chunks(
            mock.sentinel.conn,
            mock.sentinel.lock,
            [b"\x00\x01" * 8, b"\x00\x01" * 4],
        )

    assert ok is False
    assert calls == [{"action": "MIC_LOCK"}, {"action": "MIC_UNLOCK"}]


def test_build_tts_audio_payloads_merges_multiple_chunks():
    calls = []

    class _FakeAgent:
        def prepare_tts_chunks(self, text, max_chunks=3):
            assert text == "응답 본문"
            assert max_chunks == 3
            return ["첫 번째 문장", "두 번째 문장"]

        def text_to_audio(self, text, trim_pad_ms=140.0):
            calls.append((text, trim_pad_ms))
            return text.encode("utf-8")

        def merge_audio_chunks(self, chunks, sr=16000, crossfade_ms=12.0):
            assert sr == 16000
            assert crossfade_ms == 12.0
            assert chunks == ["첫 번째 문장".encode("utf-8"), "두 번째 문장".encode("utf-8")]
            return b"merged-audio"

    payloads = srv._build_tts_audio_payloads(_FakeAgent(), "응답 본문", max_chunks=3)

    assert payloads == [b"merged-audio"]
    assert calls == [("첫 번째 문장", srv.TTS_CHUNK_EDGE_PAD_MS), ("두 번째 문장", srv.TTS_CHUNK_EDGE_PAD_MS)]


def test_build_tts_audio_payloads_retries_single_pass_after_chunk_failure():
    calls = []

    class _FakeAgent:
        def prepare_tts_chunks(self, text, max_chunks=3):
            assert text == "응답 본문"
            return ["앞부분", "뒷부분"]

        def text_to_audio(self, text, trim_pad_ms=140.0):
            calls.append((text, trim_pad_ms))
            if text == "앞부분":
                return b"front"
            if text == "뒷부분":
                return b""
            if text == "앞부분 뒷부분":
                return b"fallback"
            raise AssertionError(text)

        def merge_audio_chunks(self, chunks, sr=16000, crossfade_ms=12.0):
            raise AssertionError("merge should not run when chunk fallback is needed")

    payloads = srv._build_tts_audio_payloads(_FakeAgent(), "응답 본문", max_chunks=3)

    assert payloads == [b"fallback"]
    assert calls == [("앞부분", srv.TTS_CHUNK_EDGE_PAD_MS), ("뒷부분", srv.TTS_CHUNK_EDGE_PAD_MS), ("앞부분 뒷부분", srv.TTS_FALLBACK_PAD_MS)]


def test_warm_up_runtime_assets_loads_stt_and_tts():
    class _FakeSTT:
        def __init__(self):
            self.model = None
            self.ensure_calls = 0

        def ensure_model(self):
            self.ensure_calls += 1
            self.model = object()

    class _FakeAgent:
        def __init__(self):
            self.calls = []

        def text_to_audio(self, text, trim_pad_ms=140.0):
            self.calls.append((text, trim_pad_ms))
            return b"\x00\x01" * 16

    stt = _FakeSTT()
    agent = _FakeAgent()

    status = srv._warm_up_runtime_assets(stt, agent)

    assert status == {"stt_ready": True, "tts_ready": True}
    assert stt.ensure_calls == 1
    assert agent.calls == [("준비됐어요.", 0.0)]


def test_prime_connection_sends_initial_pong_for_serial():
    with mock.patch.object(srv, "send_pong", return_value=True) as send_pong:
        assert srv._prime_connection(mock.sentinel.conn, ("serial", "/dev/cu.usbserial-test"), mock.sentinel.lock) is True

    send_pong.assert_called_once_with(mock.sentinel.conn, mock.sentinel.lock)


def test_prime_connection_skips_non_serial_addr():
    with mock.patch.object(srv, "send_pong") as send_pong:
        assert srv._prime_connection(mock.sentinel.conn, ("127.0.0.1", 5001), mock.sentinel.lock) is False

    send_pong.assert_not_called()


def test_start_connection_greeting_spawns_background_thread():
    fake_thread = mock.Mock()
    gate = mock.sentinel.input_gate

    with mock.patch.object(srv.threading, "Thread", return_value=fake_thread) as thread_cls:
        result = srv._start_connection_greeting(
            mock.sentinel.conn,
            mock.sentinel.lock,
            mock.sentinel.agent,
            {"connection_greeting_sent": False},
            gate,
        )

    assert result is fake_thread
    fake_thread.start.assert_called_once_with()
    thread_cls.assert_called_once()
    _, kwargs = thread_cls.call_args
    assert kwargs["target"] is srv._send_connection_greeting
    assert kwargs["args"] == (
        mock.sentinel.conn,
        mock.sentinel.lock,
        mock.sentinel.agent,
        {"connection_greeting_sent": False},
        gate,
    )
    assert kwargs["daemon"] is True
    assert kwargs["name"] == "connection-greeting"


def test_build_interrupt_handler_prints_stats_and_raises():
    perf = mock.Mock()
    handler = srv._build_interrupt_handler(perf)

    with mock.patch.object(builtins, "__import__") as import_mock:
        fake_logger = mock.Mock()
        import_mock.return_value.getLogger.return_value = fake_logger
        try:
            handler(signal.SIGINT, None)
        except KeyboardInterrupt:
            pass
        else:
            assert False, "KeyboardInterrupt expected"

    perf.print_stats.assert_called_once()
    fake_logger.info.assert_called()
