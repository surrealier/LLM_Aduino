from src.agent_mode import AgentMode


class _FakeLLM:
    def __init__(self, response):
        self.response = response
        self.messages = None

    def chat(self, messages, **kwargs):
        self.messages = messages
        return self.response


class _FakeEmotion:
    def set_body_state(self, **_kwargs):
        pass

    def analyze_emotion(self, _text, speaker_id="default"):
        return "neutral"


class _FakeProactive:
    def __init__(self):
        self.updated = 0
        self.sleep_mode = False
        self.sleep_until = None

    def update_interaction(self):
        self.updated += 1


class _FakeScheduler:
    def process_schedule_request(self, _text):
        return None


class _FakeMemory:
    def __init__(self):
        self.after_turn_called = 0

    def build_system_prompt(self):
        return "base system prompt"

    def after_turn(self, _history):
        self.after_turn_called += 1


class _FakeIntegrations:
    def execute(self, *_args, **_kwargs):
        return None


class _FakeInfo:
    def process_info_request(self, _text):
        return None


def _make_agent(fake_llm):
    agent = AgentMode.__new__(AgentMode)
    agent.llm = fake_llm
    agent.tts_voice = "ko-KR-SunHiNeural"
    agent.conversation_history = []
    agent.user_histories = {}
    agent.max_history = 20
    agent.conversation_count = 0
    agent.proactive = _FakeProactive()
    agent.emotion_system = _FakeEmotion()
    agent.scheduler = _FakeScheduler()
    agent.memory = _FakeMemory()
    agent.integrations = _FakeIntegrations()
    agent.info_services = _FakeInfo()
    return agent


def test_scenario_user_question_to_llm_answer_with_integration_context():
    llm = _FakeLLM("[INTENT:none] Python is a general-purpose programming language.")
    agent = _make_agent(llm)
    agent._resolve_info_data = lambda _text: {"type": "search", "items": ["python"]}

    response, intent = agent.generate_response("search python", speaker_id="alice")

    assert response == "Python is a general-purpose programming language."
    assert intent == "none"
    assert agent.proactive.updated == 1
    assert agent.memory.after_turn_called == 1
    assert "python" in llm.messages[0]["content"]
    assert len(agent.user_histories["alice"]) == 2


def test_scenario_integration_error_returns_user_guidance_message():
    llm = _FakeLLM("unused")
    agent = _make_agent(llm)
    agent._resolve_info_data = lambda _text: {
        "type": "integration_error",
        "message": "Search API key is missing.",
    }

    response, intent = agent.generate_response("search")

    assert response == "Search API key is missing."
    assert intent == "none"


def test_integration_context_is_not_overwritten_by_local_scheduler():
    class _LocalScheduler:
        def process_schedule_request(self, _text):
            return "local schedule"

    llm = _FakeLLM("[INTENT:none] ok")
    agent = _make_agent(llm)
    agent.scheduler = _LocalScheduler()
    agent._resolve_info_data = lambda _text: {"type": "calendar", "events": [{"title": "google event"}]}

    response, _ = agent.generate_response("calendar")

    assert response == "ok"
    system_prompt = llm.messages[0]["content"]
    assert "google event" in system_prompt
    assert "local schedule" not in system_prompt
