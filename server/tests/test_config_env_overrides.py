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


def test_runtime_priority_env_overrides_are_normalized(monkeypatch):
    monkeypatch.setenv("LLM_PRIORITY", "api > ollama_cpu")
    monkeypatch.setenv("LLM_API_PRIORITY", "claude > gemini")
    monkeypatch.setenv("CONNECTION_PRIORITY", "wifi > wired")
    monkeypatch.setenv("PROCESSOR_PRIORITY", "cpu > gpu")
    monkeypatch.setenv("OLLAMA_MODEL", "qwen2.5:1.5b")
    monkeypatch.setenv("GEMINI_MODEL", "gemini-2.5-flash")

    cfg = Config(config_file="missing.yaml")

    assert cfg.get("llm", "priority") == ["api", "ollama_cpu", "ollama", "other"]
    assert cfg.get("llm", "api_priority") == ["claude", "gemini", "chatgpt"]
    assert cfg.get("connection", "priority") == ["wifi", "wired"]
    assert cfg.get("runtime", "processor_priority") == ["cpu", "gpu"]
    assert cfg.get("llm", "ollama_model") == "qwen2.5:1.5b"
    assert cfg.get("llm", "api_models", "gemini") == "gemini-2.5-flash"


def test_telegram_env_overrides_are_loaded(monkeypatch):
    monkeypatch.setenv("TELEGRAM_ENABLED", "true")
    monkeypatch.setenv("TELEGRAM_BOT_TOKEN", "123456:abc")
    monkeypatch.setenv("TELEGRAM_ALLOWED_CHAT_IDS", "42,-100987654321")
    monkeypatch.setenv("TELEGRAM_MIN_INTERVAL_SEC", "1.5")
    monkeypatch.setenv("TELEGRAM_POLL_INTERVAL_SEC", "2.0")
    monkeypatch.setenv("TELEGRAM_LONG_POLL_TIMEOUT_SEC", "15.0")

    cfg = Config(config_file="missing.yaml")

    assert cfg.get("telegram", "enabled") is True
    assert cfg.get("telegram", "bot_token") == "123456:abc"
    assert cfg.get("telegram", "allowed_chat_ids") == ["42", "-100987654321"]
    assert cfg.get("telegram", "min_interval_sec") == 1.5
    assert cfg.get("telegram", "poll_interval_sec") == 2.0
    assert cfg.get("telegram", "long_poll_timeout_sec") == 15.0
