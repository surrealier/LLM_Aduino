"""Multi-provider LLM client (Ollama / Gemini / Claude / ChatGPT)."""
import json
import logging
from typing import Optional, Union

import requests

from .runtime_preferences import RuntimePreferences


log = logging.getLogger(__name__)
ThinkType = Optional[Union[bool, str]]


class LLMClient:
    def __init__(
        self,
        base_url: str,
        model: str,
        default_think: ThinkType = False,
        provider: str = "ollama",
        api_key: str = "",
        retry_max_tokens: int = 1024,
    ):
        self.provider = (provider or "ollama").strip().lower()
        self.api_key = api_key or ""
        resolved_base_url = (base_url or "http://localhost:11434").strip()
        self.base_url = resolved_base_url.rstrip("/")
        self.model = model
        self.default_think = default_think
        self.retry_max_tokens = max(1, int(retry_max_tokens or 1024))
        self.url = f"{self.base_url}/api/chat"
        self.url_generate = f"{self.base_url}/api/generate"
        self.last_error_code = None
        self.last_error = ""
        self.last_finish_reason = ""
        self.last_response_truncated = False

    def _clear_error_state(self):
        self.last_error_code = None
        self.last_error = ""
        self.last_finish_reason = ""
        self.last_response_truncated = False

    def _remember_error(self, code: str, message: str):
        self.last_error_code = code
        self.last_error = message

    @staticmethod
    def _is_length_finish_reason(reason: str | None) -> bool:
        normalized = (reason or "").strip().lower()
        return normalized in {"length", "max_tokens", "max_output_tokens", "max_tokens_reached"}

    def _remember_finish_reason(self, reason: str | None):
        self.last_finish_reason = (reason or "").strip()
        self.last_response_truncated = self._is_length_finish_reason(self.last_finish_reason)

    def _retry_token_budget(self, max_tokens: int) -> int:
        retry_tokens = max(int(max_tokens) * 2, 384)
        return max(int(max_tokens), min(retry_tokens, self.retry_max_tokens))

    def chat(
        self,
        messages: list,
        temperature: float = 0.8,
        max_tokens: int = 512,
        think: ThinkType = None,
    ) -> str:
        """LLM chat request. messages format: [{"role": ..., "content": ...}, ...]."""
        self._clear_error_state()
        if self.provider != "ollama":
            return self._chat_external(messages, temperature, max_tokens)
        try:
            if think is None:
                think = self.default_think

            content, done_reason, thinking = self._chat_once(
                messages,
                temperature,
                max_tokens,
                think=think,
            )

            # 모델이 길이 제한으로 끊긴 경우 한 번 더 크게 재시도
            if content.strip() and done_reason == "length":
                retry_tokens = self._retry_token_budget(max_tokens)
                retry_think = False if think else think
                log.warning(
                    "Ollama response hit token limit (num_predict=%d). retrying once with %d (think=%s).",
                    max_tokens,
                    retry_tokens,
                    retry_think,
                )
                retry_content, retry_done_reason, retry_thinking = self._chat_once(
                    messages,
                    temperature,
                    retry_tokens,
                    think=retry_think,
                )
                if retry_content.strip():
                    content = retry_content
                    done_reason = retry_done_reason
                    thinking = retry_thinking
            self._remember_finish_reason(done_reason)

            if not content.strip():
                if thinking.strip() and think:
                    # thinking은 생성됐지만 최종 content가 비는 경우, think=false로 1회 재시도
                    retry_tokens = min(max(max_tokens, 384), self.retry_max_tokens)
                    log.warning(
                        "Ollama returned empty content (thinking_len=%d). retrying once with think=false.",
                        len(thinking),
                    )
                    retry_content, retry_done_reason, _ = self._chat_once(
                        messages,
                        temperature,
                        retry_tokens,
                        think=False,
                    )
                    if retry_content.strip():
                        self._remember_finish_reason(retry_done_reason)
                        log.info(
                            "Ollama empty content recovered via think=false (done_reason=%s, len=%d)",
                            retry_done_reason,
                            len(retry_content.strip()),
                        )
                        return retry_content.strip()

                log.warning("Ollama returned empty content.")
                fallback = self._generate_fallback(messages, temperature, max_tokens)
                if fallback.strip():
                    log.info("Ollama chat empty -> generate fallback ok (len=%d)", len(fallback.strip()))
                    return fallback.strip()
            return content.strip()
        except Exception as exc:
            self._remember_error("provider_error", str(exc))
            log.warning("Ollama API error: %s", exc)
            return ""

    def _chat_external(self, messages: list, temperature: float, max_tokens: int) -> str:
        try:
            if self.provider == "gemini":
                text = self._chat_gemini(messages, temperature, max_tokens)
            elif self.provider == "claude":
                text = self._chat_claude(messages, temperature, max_tokens)
            elif self.provider == "chatgpt":
                text = self._chat_openai(messages, temperature, max_tokens)

            if self.provider in {"gemini", "claude", "chatgpt"}:
                if self.last_response_truncated:
                    retry_tokens = self._retry_token_budget(max_tokens)
                    log.warning(
                        "%s response hit token limit (max_tokens=%d). retrying once with %d.",
                        self.provider,
                        max_tokens,
                        retry_tokens,
                    )
                    retry_text = {
                        "gemini": self._chat_gemini,
                        "claude": self._chat_claude,
                        "chatgpt": self._chat_openai,
                    }[self.provider](messages, temperature, retry_tokens)
                    if retry_text.strip():
                        text = retry_text
                return text
            self._remember_error("unsupported_provider", f"Unsupported LLM provider: {self.provider}")
            log.error("Unsupported LLM provider: %s", self.provider)
        except Exception as exc:
            error_message = str(exc)
            error_code = "missing_api_key" if "API_KEY is missing" in error_message else "provider_error"
            self._remember_error(error_code, error_message)
            log.error("%s API error: %s", self.provider, exc)
        return ""

    def _chat_openai(self, messages: list, temperature: float, max_tokens: int) -> str:
        if not self.api_key:
            raise RuntimeError("OPENAI_API_KEY is missing")
        response = requests.post(
            "https://api.openai.com/v1/chat/completions",
            headers={"Authorization": f"Bearer {self.api_key}"},
            json={
                "model": self.model,
                "messages": messages,
                "temperature": temperature,
                "max_tokens": max_tokens,
            },
            timeout=(5, 120),
        )
        response.raise_for_status()
        data = response.json()
        choices = data.get("choices") or []
        if not choices:
            return ""
        self._remember_finish_reason(choices[0].get("finish_reason"))
        message = choices[0].get("message") or {}
        return (message.get("content") or "").strip()

    def _chat_claude(self, messages: list, temperature: float, max_tokens: int) -> str:
        if not self.api_key:
            raise RuntimeError("ANTHROPIC_API_KEY is missing")
        system_prompt = ""
        non_system = []
        for msg in messages:
            if msg.get("role") == "system":
                system_prompt += (msg.get("content") or "") + "\n"
            else:
                non_system.append(msg)

        response = requests.post(
            "https://api.anthropic.com/v1/messages",
            headers={
                "x-api-key": self.api_key,
                "anthropic-version": "2023-06-01",
                "content-type": "application/json",
            },
            json={
                "model": self.model,
                "system": system_prompt.strip(),
                "messages": non_system,
                "temperature": temperature,
                "max_tokens": max_tokens,
            },
            timeout=(5, 120),
        )
        response.raise_for_status()
        data = response.json()
        self._remember_finish_reason(data.get("stop_reason"))
        content = data.get("content") or []
        text_parts = [part.get("text", "") for part in content if isinstance(part, dict)]
        return "".join(text_parts).strip()

    def _chat_gemini(self, messages: list, temperature: float, max_tokens: int) -> str:
        text, finish_reason = self._chat_gemini_once(messages, temperature, max_tokens)
        if text and finish_reason == "MAX_TOKENS":
            log.warning("Gemini response truncated (finishReason=MAX_TOKENS, max_tokens=%d)", max_tokens)
        return text

    def _chat_gemini_once(self, messages: list, temperature: float, max_tokens: int) -> tuple[str, str]:
        if not self.api_key:
            raise RuntimeError("GEMINI_API_KEY is missing")

        contents = []
        for msg in messages:
            role = msg.get("role", "user")
            mapped_role = "model" if role == "assistant" else "user"
            if role == "system":
                mapped_role = "user"
            contents.append({"role": mapped_role, "parts": [{"text": msg.get("content", "")}]} )

        response = requests.post(
            f"https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent?key={self.api_key}",
            json={
                "contents": contents,
                "generationConfig": {
                    "temperature": temperature,
                    "maxOutputTokens": max_tokens,
                },
            },
            timeout=(5, 120),
        )
        response.raise_for_status()
        data = response.json()
        candidates = data.get("candidates") or []
        if not candidates:
            return "", ""
        candidate = candidates[0]
        finish_reason = (candidate.get("finishReason") or "").upper()
        self._remember_finish_reason(finish_reason)
        candidate_content = (candidate.get("content") or {}).get("parts") or []
        text = "".join(part.get("text", "") for part in candidate_content if isinstance(part, dict)).strip()
        return text, finish_reason

    def _chat_once(
        self,
        messages: list,
        temperature: float,
        max_tokens: int,
        think: ThinkType = True,
    ) -> tuple[str, str, str]:
        """
        /api/chat 스트리밍 응답을 조합해 최종 텍스트를 반환.
        Returns:
            (content, done_reason, thinking)
        """
        payload = {
            "model": self.model,
            "messages": messages,
            "stream": True,
            "options": {"temperature": temperature, "num_predict": max_tokens},
        }
        if think is not None:
            payload["think"] = think

        chunks = []
        thinking_chunks = []
        done_reason = ""

        with requests.post(
            self.url,
            json=payload,
            timeout=(5, 180),
            stream=True,
        ) as resp:
            resp.raise_for_status()
            for raw_line in resp.iter_lines(decode_unicode=True):
                if not raw_line:
                    continue
                line = raw_line.strip()
                if not line:
                    continue

                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    log.debug("Skipping non-JSON Ollama stream chunk: %r", line[:200])
                    continue

                if not isinstance(data, dict):
                    continue
                if data.get("error"):
                    raise RuntimeError(data.get("error"))

                piece = ""
                think_piece = ""
                msg = data.get("message")
                if isinstance(msg, dict):
                    piece = msg.get("content") or ""
                    think_piece = msg.get("thinking") or ""
                elif "response" in data:
                    piece = data.get("response") or ""
                    think_piece = data.get("thinking") or ""

                if piece:
                    chunks.append(piece)
                if think_piece:
                    thinking_chunks.append(think_piece)

                if data.get("done") is True:
                    done_reason = (data.get("done_reason") or "").strip().lower()
                    break

        return "".join(chunks).strip(), done_reason, "".join(thinking_chunks).strip()

    def _generate_fallback(self, messages: list, temperature: float, max_tokens: int) -> str:
        """Fallback to /api/generate when /api/chat returns empty content."""
        try:
            prompt = self._messages_to_prompt(messages)
            resp = requests.post(self.url_generate, json={
                "model": self.model,
                "prompt": prompt,
                "stream": False,
                "think": False,
                "options": {"temperature": temperature, "num_predict": max_tokens},
            }, timeout=(5, 180))
            resp.raise_for_status()
            data = resp.json()
            if isinstance(data, dict):
                return (data.get("response") or "").strip()
        except Exception as exc:
            log.warning("Ollama generate fallback error: %s", exc)
        return ""

    @staticmethod
    def _messages_to_prompt(messages: list) -> str:
        """Simple chat-to-prompt converter for /api/generate."""
        lines = []
        for msg in messages:
            role = msg.get("role")
            content = msg.get("content", "")
            if role == "system":
                lines.append(f"SYSTEM: {content}")
            elif role == "user":
                lines.append(f"USER: {content}")
            elif role == "assistant":
                lines.append(f"ASSISTANT: {content}")
        lines.append("ASSISTANT:")
        return "\n".join(lines)


class PriorityLLMClient:
    def __init__(self, llm_config: dict, preferences: RuntimePreferences):
        self.default_think = llm_config.get("think", False)
        self.last_error_code = None
        self.last_error = ""
        self.last_finish_reason = ""
        self.last_response_truncated = False
        self.provider = (llm_config.get("provider", "ollama") or "ollama").strip().lower()
        self.base_url = llm_config.get("base_url", "http://localhost:11434").rstrip("/")
        self.model = llm_config.get("model", "")
        self.active_candidate = None
        self._clients: dict[tuple[str, str, str], LLMClient] = {}
        self.reload_runtime_preferences(preferences, llm_config)

    def reload_runtime_preferences(self, preferences: RuntimePreferences, llm_config: dict):
        self.preferences = preferences
        self.llm_config = dict(llm_config or {})
        self.default_think = self.llm_config.get("think", False)
        self.retry_max_tokens = int(self.llm_config.get("retry_max_tokens", 1024) or 1024)
        resolved_base_url = (self.llm_config.get("base_url") or "http://localhost:11434").strip()
        self.base_url = resolved_base_url.rstrip("/")
        self.provider = (self.llm_config.get("provider", "ollama") or "ollama").strip().lower()
        self.model = self.llm_config.get("model", "") or preferences.llm_models.get("ollama", "")

    def _clear_error_state(self):
        self.last_error_code = None
        self.last_error = ""

    def _remember_error(self, code: str | None, message: str):
        self.last_error_code = code or "provider_error"
        self.last_error = message

    def _api_key_for_provider(self, provider: str) -> str:
        mapping = {
            "gemini": self.llm_config.get("gemini_api_key", ""),
            "claude": self.llm_config.get("anthropic_api_key", ""),
            "chatgpt": self.llm_config.get("openai_api_key", ""),
        }
        return mapping.get(provider, "") or ""

    def _client_for_candidate(self, provider: str, model: str, api_key: str = "") -> LLMClient:
        key = (provider, model, api_key)
        client = self._clients.get(key)
        if client is None:
            client = LLMClient(
                base_url=self.base_url,
                model=model,
                default_think=self.default_think,
                provider=provider,
                api_key=api_key,
                retry_max_tokens=self.retry_max_tokens,
            )
            self._clients[key] = client
        else:
            client.retry_max_tokens = self.retry_max_tokens
        return client

    def _build_candidates(self) -> list[dict]:
        candidates: list[dict] = []
        for bucket in self.preferences.llm_priority:
            if bucket == "ollama":
                if not self.preferences.gpu_bucket_enabled():
                    continue
                candidates.append(
                    {
                        "bucket": bucket,
                        "provider": "ollama",
                        "model": self.preferences.llm_models["ollama"],
                        "api_key": "",
                        "processor": "gpu",
                    }
                )
                continue

            if bucket == "api":
                for provider in self.preferences.api_priority:
                    api_key = self._api_key_for_provider(provider)
                    if not api_key:
                        continue
                    candidates.append(
                        {
                            "bucket": bucket,
                            "provider": provider,
                            "model": self.preferences.llm_models[provider],
                            "api_key": api_key,
                            "processor": "remote",
                        }
                    )
                continue

            if bucket == "ollama_cpu":
                candidates.append(
                    {
                        "bucket": bucket,
                        "provider": "ollama",
                        "model": self.preferences.llm_models["ollama"],
                        "api_key": "",
                        "processor": "cpu",
                    }
                )
                continue

            if bucket == "other":
                legacy_provider = self.provider
                if legacy_provider == "ollama":
                    candidates.append(
                        {
                            "bucket": bucket,
                            "provider": "ollama",
                            "model": self.preferences.llm_models["ollama"],
                            "api_key": "",
                            "processor": "gpu" if self.preferences.gpu_bucket_enabled() else "cpu",
                        }
                    )
                elif legacy_provider in self.preferences.llm_models:
                    api_key = self._api_key_for_provider(legacy_provider)
                    if api_key:
                        candidates.append(
                            {
                                "bucket": bucket,
                                "provider": legacy_provider,
                                "model": self.preferences.llm_models[legacy_provider],
                                "api_key": api_key,
                                "processor": "remote",
                            }
                        )

        deduped: list[dict] = []
        seen: set[tuple[str, str, str]] = set()
        for candidate in candidates:
            signature = (
                candidate["bucket"],
                candidate["provider"],
                candidate["model"],
            )
            if signature in seen:
                continue
            seen.add(signature)
            deduped.append(candidate)
        return deduped

    def chat(
        self,
        messages: list,
        temperature: float = 0.8,
        max_tokens: int = 512,
        think: ThinkType = None,
    ) -> str:
        self._clear_error_state()
        candidates = self._build_candidates()
        if not candidates:
            self._remember_error("unsupported_provider", "No runnable LLM candidate is configured")
            return ""

        errors = []
        for candidate in candidates:
            client = self._client_for_candidate(
                candidate["provider"],
                candidate["model"],
                candidate["api_key"],
            )
            response = client.chat(
                messages,
                temperature=temperature,
                max_tokens=max_tokens,
                think=think,
            )
            if response.strip():
                self.provider = client.provider
                self.model = client.model
                self.base_url = client.base_url
                self.active_candidate = {
                    "bucket": candidate["bucket"],
                    "provider": client.provider,
                    "model": client.model,
                    "processor": candidate["processor"],
                }
                self.last_error_code = None
                self.last_error = ""
                self.last_finish_reason = getattr(client, "last_finish_reason", "")
                self.last_response_truncated = bool(getattr(client, "last_response_truncated", False))
                return response.strip()

            errors.append(
                f"{candidate['bucket']}:{candidate['provider']}({client.last_error_code or 'empty'})"
            )
            if client.last_error_code:
                self.last_error_code = client.last_error_code
                self.last_error = client.last_error
            else:
                self._remember_error("provider_error", f"{candidate['provider']} returned empty response")

        if errors and not self.last_error:
            self._remember_error("provider_error", ", ".join(errors))
        return ""

    def describe_runtime(self) -> dict:
        return {
            "configured_provider": (self.llm_config.get("provider", "ollama") or "ollama").strip().lower(),
            "active_provider": (self.active_candidate or {}).get("provider"),
            "active_model": (self.active_candidate or {}).get("model"),
            "active_bucket": (self.active_candidate or {}).get("bucket"),
            "active_processor": (self.active_candidate or {}).get("processor"),
            "priority": list(self.preferences.llm_priority),
            "api_priority": list(self.preferences.api_priority),
            "base_url": self.base_url,
            "last_error_code": self.last_error_code,
            "last_finish_reason": self.last_finish_reason,
            "last_response_truncated": self.last_response_truncated,
        }
