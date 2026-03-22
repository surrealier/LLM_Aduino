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
