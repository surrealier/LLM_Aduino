from datetime import datetime

import numpy as np

from src.agent_mode import AgentMode


def _make_agent():
    agent = AgentMode.__new__(AgentMode)
    agent._get_assistant_settings = lambda: ("아이", "cheerful")
    agent.conversation_history = []
    return agent


def test_sanitize_response_removes_intro_and_emoji():
    agent = _make_agent()
    response = "안녕하세요! 저는 콜리입니다! 반가워요 😊"
    assert agent._sanitize_response(response) == "반가워요"


def test_split_text_for_tts_long_text_to_two_or_three_chunks():
    agent = _make_agent()
    response = (
        "오늘 일정을 확인해 보니 오후 세 시 회의가 있고, "
        "저녁 여섯 시에는 운동 약속이 있어요. 준비할 게 있으면 미리 알려드릴게요."
    )
    chunks = agent.split_text_for_tts(response, max_chunks=3)
    assert 2 <= len(chunks) <= 3
    assert "".join(chunks).replace(" ", "") == response.replace(" ", "")


def test_prepare_tts_chunks_sanitizes_text():
    agent = _make_agent()
    response = "저는 콜리입니다! 오늘은 날씨가 좋아요 😊 산책 어떠세요?"
    chunks = agent.prepare_tts_chunks(response, max_chunks=3)
    assert chunks
    assert all("콜리입니다" not in chunk for chunk in chunks)
    assert all("😊" not in chunk for chunk in chunks)


def test_merge_audio_chunks_applies_crossfade():
    chunk1 = (np.ones(1600, dtype=np.int16) * 1200).tobytes()
    chunk2 = (np.ones(1600, dtype=np.int16) * -1200).tobytes()

    merged = AgentMode.merge_audio_chunks(
        [chunk1, chunk2],
        sr=16000,
        crossfade_ms=10.0,
    )

    # 10ms @ 16kHz = 160 samples crossfade overlap
    expected_samples = 1600 + 1600 - 160
    assert len(merged) == expected_samples * 2


def test_crossfade_audio_boundaries_keeps_chunked_structure():
    chunk1 = (np.ones(1600, dtype=np.int16) * 1200).tobytes()
    chunk2 = (np.ones(1600, dtype=np.int16) * -1200).tobytes()
    chunk3 = (np.ones(1600, dtype=np.int16) * 800).tobytes()

    crossed = AgentMode.crossfade_audio_boundaries(
        [chunk1, chunk2, chunk3],
        sr=16000,
        crossfade_ms=10.0,
    )

    assert len(crossed) == 3
    # boundary 2개 × 160 samples가 전체에서 줄어든다.
    total_samples = sum(len(c) for c in crossed) // 2
    assert total_samples == (1600 * 3 - 160 * 2)


def test_history_for_user_separated():
    agent = AgentMode.__new__(AgentMode)
    agent.conversation_history = []
    agent.user_histories = {}
    h1 = agent._history_for_user("alice")
    h2 = agent._history_for_user("bob")
    h1.append({"role": "user", "content": "a"})
    assert h1 is not h2
    assert len(h2) == 0


def test_generate_connection_greeting_uses_llm_context_and_prefix():
    class _FakeMemory:
        def build_system_prompt(self):
            return "기억된 메모"

    class _FakeLLM:
        def __init__(self):
            self.messages = None

        def chat(self, messages, **kwargs):
            self.messages = messages
            return "아직 일하고 계세요?"

    agent = _make_agent()
    agent.memory = _FakeMemory()
    agent.llm = _FakeLLM()
    agent.conversation_history = [{"role": "user", "content": "오늘 회의 많아서 바빠"}]

    greeting = agent.generate_connection_greeting()

    assert greeting == "콜리 연결됐어요! 아직 일하고 계세요?"
    assert "실시간 최근 대화" in agent.llm.messages[1]["content"]
    assert agent.llm.messages


def test_generate_connection_greeting_falls_back_when_llm_truncates():
    class _FakeMemory:
        def build_system_prompt(self):
            return "기억된 메모"

    class _FakeLLM:
        last_response_truncated = False

        def chat(self, messages, **kwargs):
            self.last_response_truncated = True
            return "연결"

    agent = _make_agent()
    agent.memory = _FakeMemory()
    agent.llm = _FakeLLM()

    greeting = agent.generate_connection_greeting(now=datetime(2026, 3, 17, 8, 0))

    assert greeting == "콜리 연결됐어요! 잠 잘 주무셨어요?"


def test_generate_connection_greeting_falls_back_by_time():
    agent = _make_agent()
    agent.llm = None

    greeting = agent.generate_connection_greeting(now=datetime(2026, 3, 17, 8, 0))

    assert greeting == "콜리 연결됐어요! 잠 잘 주무셨어요?"
