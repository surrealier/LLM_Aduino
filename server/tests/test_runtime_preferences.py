from src.runtime_preferences import HardwareProfile, RuntimePreferences


def _preferences(*, platform="darwin", accelerators=None, tts_backend="edge_tts"):
    return RuntimePreferences(
        llm_priority=["ollama", "api", "ollama_cpu", "other"],
        api_priority=["gemini", "claude", "chatgpt"],
        connection_priority=["wired", "wifi"],
        processor_priority=["gpu", "cpu"],
        llm_models={
            "ollama": "qwen2.5:0.5b",
            "gemini": "gemini-2.5-flash",
            "claude": "claude-3-5-haiku-latest",
            "chatgpt": "gpt-4o-mini",
        },
        hardware=HardwareProfile(accelerators=accelerators or ["mps"], platform=platform),
        tts_backend=tts_backend,
    )


def test_runtime_preferences_keep_stt_on_cpu_when_only_mps_is_detected():
    prefs = _preferences(accelerators=["mps"])

    assert prefs.resolved_stt_devices() == ["cpu"]


def test_runtime_preferences_report_mac_mps_limits_for_current_audio_stack():
    prefs = _preferences(accelerators=["mps"])

    notes = prefs.audio_runtime_notes()

    assert any("MPS" in note and "STT" in note for note in notes)
    assert any("Edge TTS" in note and "MPS" in note for note in notes)


def test_runtime_preferences_status_snapshot_exposes_audio_capability_flags():
    prefs = _preferences(accelerators=["mps"])

    runtime = prefs.to_dict()

    assert runtime["hardware"]["mps_available"] is True
    assert runtime["stt_supported_devices"] == ["cuda", "cpu"]
    assert runtime["stt_mps_supported"] is False
    assert runtime["tts_mps_supported"] is False
