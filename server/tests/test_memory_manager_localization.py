from src.memory_manager import MemoryManager, _KOREAN_WEEKDAYS


class _FakeLLM:
    def chat(self, *_args, **_kwargs):
        return ""


def test_build_system_prompt_uses_korean_weekday(tmp_path):
    memory_dir = tmp_path / "memory"
    mm = MemoryManager(_FakeLLM(), memory_dir=str(memory_dir), refresh_interval=1)

    prompt = mm.build_system_prompt()

    assert any(day in prompt for day in _KOREAN_WEEKDAYS)
