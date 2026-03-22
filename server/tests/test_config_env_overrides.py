from config_loader import Config


def test_env_overrides_support_stt_tts_and_memory(monkeypatch):
    monkeypatch.setenv("STT_DEVICE", "cuda")
    monkeypatch.setenv("STT_LANGUAGE", "ko")
    monkeypatch.setenv("STT_MODEL_SIZE", "medium")
    monkeypatch.setenv("TTS_VOICE", "ko-KR-SunHiNeural")
    monkeypatch.setenv("MEMORY_DIR", "../.codex/personal_memory")
    monkeypatch.setenv("MEMORY_REFRESH_INTERVAL", "7")

    cfg = Config(config_file='missing.yaml')

    assert cfg.get("stt", "device") == "cuda"
    assert cfg.get("stt", "language") == "ko"
    assert cfg.get("stt", "model_size") == "medium"
    assert cfg.get("tts", "voice") == "ko-KR-SunHiNeural"
    assert cfg.get("memory", "memory_dir") == "../.codex/personal_memory"
    assert cfg.get("memory", "refresh_interval") == 7


def test_stt_device_override_takes_precedence_over_legacy_device(monkeypatch):
    monkeypatch.setenv("DEVICE", "cpu")
    monkeypatch.setenv("STT_DEVICE", "cuda")

    cfg = Config(config_file='missing.yaml')

    assert cfg.get("stt", "device") == "cuda"