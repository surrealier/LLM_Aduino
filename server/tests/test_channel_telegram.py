import pytest

from src.channels.telegram import (
    TelegramBotAdapter,
    TelegramBotClient,
    TelegramChannelService,
    TelegramPollingWorker,
    TelegramUpdate,
)


class _DummyAdapter:
    def __init__(self, ok=True):
        self.ok = ok
        self.sent = []

    def send_text(self, chat_id: str, text: str) -> bool:
        self.sent.append((chat_id, text))
        return self.ok


@pytest.mark.telegram
@pytest.mark.channel
def test_telegram_channel_accepts_and_sends():
    adapter = _DummyAdapter(ok=True)
    svc = TelegramChannelService(adapter=adapter, allowed_chat_ids={"42"}, min_interval_sec=0.0)

    ok, response = svc.handle_message("42", "안녕", lambda text: f"echo:{text}")

    assert ok is True
    assert response == "echo:안녕"
    assert adapter.sent == [("42", "echo:안녕")]


@pytest.mark.telegram
@pytest.mark.channel
def test_telegram_channel_rejects_unauthorized_chat():
    adapter = _DummyAdapter(ok=True)
    svc = TelegramChannelService(adapter=adapter, allowed_chat_ids={"42"})

    ok, message = svc.handle_message("99", "안녕", lambda text: text)

    assert ok is False
    assert "인증" in message


class _FakeResponse:
    def __init__(self, payload):
        self.payload = payload

    def raise_for_status(self):
        return None

    def json(self):
        return self.payload


class _FakeSession:
    def __init__(self, *, get_payload=None, post_payload=None):
        self.get_payload = get_payload or {"ok": True, "result": []}
        self.post_payload = post_payload or {"ok": True}
        self.calls = []

    def get(self, url, params=None, timeout=None):
        self.calls.append(("get", url, params, timeout))
        return _FakeResponse(self.get_payload)

    def post(self, url, json=None, timeout=None):
        self.calls.append(("post", url, json, timeout))
        return _FakeResponse(self.post_payload)

    def close(self):
        return None


class _FakeTelegramClient:
    def __init__(self, updates=None):
        self.updates = list(updates or [])
        self.sent = []

    def get_updates(self, offset=None, timeout_sec=20.0):
        _ = timeout_sec
        items = [item for item in self.updates if offset is None or item.update_id >= offset]
        self.updates = []
        return items

    def send_message(self, chat_id: str, text: str) -> bool:
        self.sent.append((chat_id, text))
        return True

    def close(self):
        return None


@pytest.mark.telegram
@pytest.mark.channel
def test_telegram_bot_client_parses_text_updates():
    session = _FakeSession(
        get_payload={
            "ok": True,
            "result": [
                {"update_id": 7, "message": {"chat": {"id": 42}, "text": "안녕"}},
                {"update_id": 8, "message": {"chat": {"id": 42}}},
            ],
        }
    )
    client = TelegramBotClient(bot_token="123:abc", session=session)

    updates = client.get_updates(offset=5, timeout_sec=10)

    assert updates == [TelegramUpdate(update_id=7, chat_id="42", text="안녕")]
    assert session.calls[0][0] == "get"
    assert session.calls[0][2]["offset"] == 5


@pytest.mark.telegram
@pytest.mark.channel
def test_telegram_bot_adapter_splits_long_messages():
    fake_client = _FakeTelegramClient()
    adapter = TelegramBotAdapter(fake_client)

    ok = adapter.send_text("42", ("안녕 " * 1500).strip())

    assert ok is True
    assert len(fake_client.sent) >= 2
    assert all(len(text) <= 4096 for _chat_id, text in fake_client.sent)


@pytest.mark.telegram
@pytest.mark.channel
def test_telegram_polling_worker_routes_reply_to_agent():
    fake_client = _FakeTelegramClient(updates=[TelegramUpdate(update_id=7, chat_id="42", text="안녕")])
    adapter = TelegramBotAdapter(fake_client)
    svc = TelegramChannelService(adapter=adapter, allowed_chat_ids={"42"}, min_interval_sec=0.0)
    worker = TelegramPollingWorker(
        client=fake_client,
        channel_service=svc,
        llm_respond=lambda chat_id, text: f"echo:{chat_id}:{text}",
        long_poll_timeout_sec=0.0,
    )

    processed = worker.poll_once()

    assert processed == 1
    assert fake_client.sent == [("42", "echo:42:안녕")]
    assert worker._next_offset == 8


@pytest.mark.telegram
@pytest.mark.channel
def test_telegram_polling_worker_replies_with_rejection_reason():
    fake_client = _FakeTelegramClient(updates=[TelegramUpdate(update_id=3, chat_id="99", text="안녕")])
    adapter = TelegramBotAdapter(fake_client)
    svc = TelegramChannelService(adapter=adapter, allowed_chat_ids={"42"}, min_interval_sec=0.0)
    worker = TelegramPollingWorker(
        client=fake_client,
        channel_service=svc,
        llm_respond=lambda chat_id, text: f"echo:{chat_id}:{text}",
        long_poll_timeout_sec=0.0,
    )

    worker.poll_once()

    assert fake_client.sent == [("99", "인증되지 않은 채널이에요. 운영자에게 chat id 등록을 요청해 주세요.")]
