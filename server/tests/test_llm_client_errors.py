from src.llm_client import LLMClient


def test_missing_external_api_key_sets_error_state():
    client = LLMClient("", "gemini-1.5-flash", provider="gemini", api_key="")

    result = client.chat([{"role": "user", "content": "hello"}])

    assert result == ""
    assert client.last_error_code == "missing_api_key"
    assert "GEMINI_API_KEY" in client.last_error