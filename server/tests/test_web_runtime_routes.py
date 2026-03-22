from fastapi.testclient import TestClient

from web.app import create_app
from web.auth import configure


class _FakeEmotion:
    current_emotion = "neutral"
    emotion_history = ["neutral"]


class _FakeProactive:
    def get_stats(self):
        return {"count": 1}


class _FakeInfoServices:
    def get_active_timers(self):
        return []

    def get_active_alarms(self):
        return []


class _FakeRuntimeController:
    def handle_text_command(self, text):
        if text == "우선순위 상태":
            return "현재 우선순위 상태예요."
        return None

    def status_snapshot(self):
        return {
            "llm_priority": ["ollama", "api", "ollama_cpu", "other"],
            "connection_priority": ["wired", "wifi"],
            "processor_priority": ["gpu", "cpu"],
        }


class _FakeAgent:
    def __init__(self):
        self.runtime_controller = _FakeRuntimeController()
        self.emotion_system = _FakeEmotion()
        self.proactive = _FakeProactive()
        self.info_services = _FakeInfoServices()
        self.conversation_count = 3
        self.generate_calls = []

    def generate_response(self, text, speaker_id="web_user"):
        self.generate_calls.append((text, speaker_id))
        return "일반 응답", "none"


def _client():
    configure("")
    agent = _FakeAgent()
    app = create_app(lambda: agent, lambda: None, lambda: "agent")
    return TestClient(app), agent


def test_api_chat_handles_runtime_priority_command():
    client, agent = _client()

    response = client.post("/api/chat/", json={"text": "우선순위 상태"})

    assert response.status_code == 200
    assert response.json()["intent"] == "runtime_config"
    assert "현재 우선순위 상태예요." in response.json()["response"]
    assert agent.generate_calls == []


def test_api_status_exposes_runtime_snapshot():
    client, _agent = _client()

    response = client.get("/api/status")

    assert response.status_code == 200
    payload = response.json()
    assert payload["runtime"]["llm_priority"] == ["ollama", "api", "ollama_cpu", "other"]
    assert payload["runtime"]["connection_priority"] == ["wired", "wifi"]
