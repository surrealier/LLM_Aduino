from src.runtime_controller import RuntimeController


class _FakeConfig:
    def __init__(self, data):
        self.config = data
        self.save_calls = 0

    def save(self, config_file=None):
        self.save_calls += 1

    def get(self, *keys, default=None):
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value

    def get_llm_config(self):
        return self.config.get("llm", {})


class _FakeLLM:
    def __init__(self):
        self.reloads = []

    def reload_runtime_preferences(self, preferences, llm_config):
        self.reloads.append((preferences, llm_config))

    def describe_runtime(self):
        return {
            "active_provider": "ollama",
            "active_model": "qwen2.5:0.5b",
            "active_bucket": "ollama",
            "active_processor": "gpu",
        }


class _FakeSTT:
    def __init__(self):
        self.device_priority = ["cuda", "cpu"]
        self.device_in_use = "cuda"
        self.updates = []

    def set_device_priority(self, devices):
        self.device_priority = list(devices)
        self.updates.append(list(devices))


def _make_config():
    return _FakeConfig(
        {
            "llm": {
                "provider": "gemini",
                "model": "gemini-2.5-flash",
                "base_url": "",
                "priority": ["ollama", "api", "ollama_cpu", "other"],
                "api_priority": ["gemini", "claude", "chatgpt"],
                "api_models": {
                    "gemini": "gemini-2.5-flash",
                    "claude": "claude-3-5-haiku-latest",
                    "chatgpt": "gpt-4o-mini",
                },
            },
            "connection": {
                "mode": "auto",
                "priority": ["wired", "wifi"],
            },
            "runtime": {
                "processor_priority": ["gpu", "cpu"],
            },
            "tts": {
                "backend": "edge_tts",
            },
        }
    )


def test_runtime_controller_reports_status_in_korean():
    controller = RuntimeController(_make_config())

    message = controller.handle_text_command("우선순위 상태")

    assert message is not None
    assert "현재 우선순위 상태예요." in message
    assert "모델: Ollama(Local) > API > Ollama(Local CPU) > Other" in message
    assert "연결: Wired > WiFi" in message


def test_runtime_controller_updates_priority_and_refreshes_bound_runtime():
    config = _make_config()
    controller = RuntimeController(config)
    fake_llm = _FakeLLM()
    fake_stt = _FakeSTT()
    controller.bind(llm_client=fake_llm, stt_engine=fake_stt)

    response = controller.handle_text_command(
        "모델 우선순위 api > ollama cpu > other 연결 우선순위 wifi > wired 프로세서 우선순위 cpu > gpu"
    )

    assert response is not None
    assert "우선순위를 반영했어요." in response
    assert config.config["llm"]["priority"] == ["api", "ollama_cpu", "other", "ollama"]
    assert config.config["connection"]["priority"] == ["wifi", "wired"]
    assert config.config["runtime"]["processor_priority"] == ["cpu", "gpu"]
    assert fake_stt.device_priority[0] == "cpu"
    assert fake_llm.reloads
    assert config.save_calls == 1


def test_runtime_controller_ignores_non_priority_commands():
    controller = RuntimeController(_make_config())

    assert controller.handle_text_command("@@voice-id status") is None
