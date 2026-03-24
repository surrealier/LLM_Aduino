from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from threading import Event, Thread
from typing import Callable

import requests

from .base import ChannelAdapter

log = logging.getLogger(__name__)

TELEGRAM_API_BASE_URL = "https://api.telegram.org"
_TELEGRAM_MESSAGE_LIMIT = 4096


@dataclass(frozen=True)
class TelegramUpdate:
    update_id: int
    chat_id: str
    text: str


@dataclass
class TelegramBotClient:
    bot_token: str
    base_url: str = TELEGRAM_API_BASE_URL
    session: requests.sessions.Session | None = None

    def __post_init__(self) -> None:
        self.bot_token = (self.bot_token or "").strip()
        self.base_url = (self.base_url or TELEGRAM_API_BASE_URL).rstrip("/")
        if self.session is None:
            self.session = requests.Session()

    def close(self) -> None:
        if self.session is not None and hasattr(self.session, "close"):
            self.session.close()

    def get_updates(self, offset: int | None = None, timeout_sec: float = 20.0) -> list[TelegramUpdate]:
        params: dict[str, int] = {"timeout": max(0, int(timeout_sec))}
        if offset is not None:
            params["offset"] = int(offset)

        response = self.session.get(
            self._api_url("getUpdates"),
            params=params,
            timeout=max(5.0, float(timeout_sec) + 5.0),
        )
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", False):
            raise RuntimeError("Telegram getUpdates returned ok=false")

        updates: list[TelegramUpdate] = []
        for item in payload.get("result", []):
            parsed = self._parse_update(item)
            if parsed is not None:
                updates.append(parsed)
        return updates

    def send_message(self, chat_id: str, text: str) -> bool:
        response = self.session.post(
            self._api_url("sendMessage"),
            json={"chat_id": chat_id, "text": text},
            timeout=10.0,
        )
        response.raise_for_status()
        payload = response.json()
        return bool(payload.get("ok", False))

    def _api_url(self, method: str) -> str:
        return f"{self.base_url}/bot{self.bot_token}/{method}"

    @staticmethod
    def _parse_update(payload: dict) -> TelegramUpdate | None:
        message = payload.get("message") or payload.get("edited_message") or {}
        chat = message.get("chat") or {}
        chat_id = chat.get("id")
        text = (message.get("text") or message.get("caption") or "").strip()
        if chat_id is None or not text:
            return None
        return TelegramUpdate(
            update_id=int(payload.get("update_id", 0)),
            chat_id=str(chat_id),
            text=text,
        )


@dataclass
class TelegramBotAdapter:
    client: TelegramBotClient

    def send_text(self, chat_id: str, text: str) -> bool:
        content = (text or "").strip()
        if not content:
            return False

        try:
            for chunk in self._split_message(content):
                if not self.client.send_message(chat_id, chunk):
                    return False
            return True
        except requests.RequestException as exc:
            log.warning("Telegram send_text request failed for chat_id=%s: %s", chat_id, exc)
            return False
        except Exception as exc:
            log.warning("Telegram send_text failed for chat_id=%s: %s", chat_id, exc)
            return False

    @staticmethod
    def _split_message(text: str) -> list[str]:
        content = (text or "").strip()
        if len(content) <= _TELEGRAM_MESSAGE_LIMIT:
            return [content]

        chunks: list[str] = []
        remaining = content
        while len(remaining) > _TELEGRAM_MESSAGE_LIMIT:
            split_at = remaining.rfind("\n", 0, _TELEGRAM_MESSAGE_LIMIT)
            if split_at < 0:
                split_at = remaining.rfind(" ", 0, _TELEGRAM_MESSAGE_LIMIT)
            if split_at < 1:
                split_at = _TELEGRAM_MESSAGE_LIMIT
            chunks.append(remaining[:split_at].strip())
            remaining = remaining[split_at:].strip()

        if remaining:
            chunks.append(remaining)
        return chunks


@dataclass
class TelegramChannelService:
    adapter: ChannelAdapter
    allowed_chat_ids: set[str] = field(default_factory=set)
    min_interval_sec: float = 0.5
    _last_sent_at: dict[str, float] = field(default_factory=dict)

    def can_accept(self, chat_id: str) -> tuple[bool, str]:
        if self.allowed_chat_ids and chat_id not in self.allowed_chat_ids:
            return False, "인증되지 않은 채널이에요. 운영자에게 chat id 등록을 요청해 주세요."

        now = time.time()
        last = self._last_sent_at.get(chat_id, 0.0)
        if now - last < self.min_interval_sec:
            return False, "요청이 너무 빨라요. 잠시 후 다시 시도해 주세요."
        return True, ""

    def validate_message(self, chat_id: str, text: str) -> tuple[bool, str]:
        allowed, reason = self.can_accept(chat_id)
        if not allowed:
            return False, reason

        if not text.strip():
            return False, "비어있는 메시지는 처리할 수 없어요."
        return True, ""

    def deliver_message(self, chat_id: str, text: str, llm_respond: Callable[[str], str]) -> tuple[bool, str]:
        response = " ".join((llm_respond(text) or "").split()).strip()
        if not response:
            return False, "응답을 만들지 못했어요."
        if not self.adapter.send_text(chat_id, response):
            return False, "메시지 전송에 실패했어요."

        self._last_sent_at[chat_id] = time.time()
        return True, response

    def handle_message(self, chat_id: str, text: str, llm_respond: Callable[[str], str]) -> tuple[bool, str]:
        valid, reason = self.validate_message(chat_id, text)
        if not valid:
            return False, reason
        return self.deliver_message(chat_id, text, llm_respond)


@dataclass
class TelegramPollingWorker:
    client: TelegramBotClient
    channel_service: TelegramChannelService
    llm_respond: Callable[[str, str], str]
    poll_interval_sec: float = 1.0
    long_poll_timeout_sec: float = 20.0
    stop_event: Event = field(default_factory=Event)
    _next_offset: int | None = field(default=None, init=False)
    _thread: Thread | None = field(default=None, init=False)

    def start(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            return False

        self.stop_event.clear()
        self._thread = Thread(target=self.run_forever, name="telegram-poller", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self.stop_event.set()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def close(self) -> None:
        self.client.close()

    def poll_once(self) -> int:
        updates = self.client.get_updates(offset=self._next_offset, timeout_sec=self.long_poll_timeout_sec)
        processed = 0
        for update in updates:
            self._next_offset = update.update_id + 1
            self._handle_update(update)
            processed += 1
        return processed

    def run_forever(self) -> None:
        while not self.stop_event.is_set():
            try:
                processed = self.poll_once()
                if processed == 0 and self.long_poll_timeout_sec <= 0:
                    self.stop_event.wait(self.poll_interval_sec)
            except requests.RequestException as exc:
                log.warning("Telegram polling request failed: %s", exc)
                self.stop_event.wait(self.poll_interval_sec)
            except Exception as exc:
                log.exception("Telegram polling loop crashed: %s", exc)
                self.stop_event.wait(self.poll_interval_sec)

    def _handle_update(self, update: TelegramUpdate) -> None:
        valid, reason = self.channel_service.validate_message(update.chat_id, update.text)
        if not valid:
            log.info("Telegram update rejected for chat_id=%s: %s", update.chat_id, reason)
            if reason:
                self.channel_service.adapter.send_text(update.chat_id, reason)
            return

        ok, result = self.channel_service.deliver_message(
            update.chat_id,
            update.text,
            lambda text: self.llm_respond(update.chat_id, text),
        )
        if not ok:
            log.warning("Telegram reply failed for chat_id=%s: %s", update.chat_id, result)
