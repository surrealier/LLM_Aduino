from __future__ import annotations

from datetime import datetime, timedelta, timezone
import re
import time
from typing import Any, Dict, Optional

from .base import BaseIntegration, IntegrationErrorCode, IntegrationResult


TOKEN_URL = "https://oauth2.googleapis.com/token"
CALENDAR_API_BASE = "https://www.googleapis.com/calendar/v3"


class GoogleCalendarIntegration(BaseIntegration):
    name = "calendar-google"

    def __init__(
        self,
        client_id: str = "",
        client_secret: str = "",
        refresh_token: str = "",
        calendar_id: str = "primary",
        time_zone: str = "Asia/Seoul",
        request_timeout: float = 8.0,
    ):
        self.client_id = client_id or ""
        self.client_secret = client_secret or ""
        self.refresh_token = refresh_token or ""
        self.calendar_id = (calendar_id or "primary").strip() or "primary"
        self.time_zone = (time_zone or "Asia/Seoul").strip() or "Asia/Seoul"
        self.request_timeout = request_timeout
        self._access_token = ""
        self._access_token_expires_at = 0.0

    def is_configured(self) -> bool:
        return all(bool(x.strip()) for x in (self.client_id, self.client_secret, self.refresh_token))

    def health_check(self) -> IntegrationResult:
        if not self.is_configured():
            return self._missing_credentials()
        return self.execute("calendar.list", {"max_results": 1, "days": 1})

    def execute(self, intent: str, params: Optional[Dict[str, Any]] = None) -> IntegrationResult:
        if not self.is_configured():
            return self._missing_credentials()

        params = params or {}
        if intent == "calendar.list":
            return self._list_events(params)
        if intent == "calendar.create":
            return self._create_event(params)
        if intent == "calendar.update":
            return self._update_event(params)
        if intent == "calendar.delete":
            return self._delete_event(params)
        return IntegrationResult.failure(
            IntegrationErrorCode.UNKNOWN,
            "Unsupported Google Calendar request.",
            {"intent": intent},
        )

    def _missing_credentials(self) -> IntegrationResult:
        return IntegrationResult.failure(
            IntegrationErrorCode.AUTH_MISSING_KEY,
            "Google Calendar OAuth settings are missing. Run `ccoli config integration set calendar-google ...`.",
        )

    def _list_events(self, params: Dict[str, Any]) -> IntegrationResult:
        now = datetime.now(timezone.utc)
        days = self._coerce_int(params.get("days"), 7, minimum=1, maximum=60)
        max_results = self._coerce_int(params.get("max_results") or params.get("maxResults"), 5, minimum=1, maximum=20)
        query = {
            "timeMin": params.get("time_min") or params.get("timeMin") or now.isoformat(),
            "timeMax": params.get("time_max") or params.get("timeMax") or (now + timedelta(days=days)).isoformat(),
            "singleEvents": "true",
            "orderBy": "startTime",
            "maxResults": max_results,
        }
        result = self._request("GET", self._events_path(params), params=query)
        if not result.ok:
            return result
        payload = result.data or {}
        events = [self._normalize_event(item) for item in payload.get("items", []) if isinstance(item, dict)]
        return IntegrationResult.success(
            {
                "type": "calendar",
                "action": "list",
                "calendar_id": self._calendar_id(params),
                "events": events,
            }
        )

    def _create_event(self, params: Dict[str, Any]) -> IntegrationResult:
        summary = (params.get("summary") or params.get("title") or "").strip()
        start = params.get("start") or params.get("start_time") or params.get("datetime")
        end = params.get("end") or params.get("end_time")
        if not summary or not start or not end:
            return IntegrationResult.failure(
                IntegrationErrorCode.HTTP_4XX,
                "Calendar event creation requires title, start, and end.",
                {"missing": self._missing_event_fields(summary, start, end)},
            )

        body = {
            "summary": summary,
            "start": self._event_time(start, params),
            "end": self._event_time(end, params),
        }
        for key in ("description", "location"):
            if params.get(key):
                body[key] = params[key]

        result = self._request("POST", self._events_path(params), json=body)
        if not result.ok:
            return result
        return IntegrationResult.success(
            {
                "type": "calendar",
                "action": "create",
                "event": self._normalize_event(result.data or {}),
            }
        )

    def _update_event(self, params: Dict[str, Any]) -> IntegrationResult:
        event_id = (params.get("event_id") or params.get("id") or "").strip()
        if not event_id:
            return IntegrationResult.failure(
                IntegrationErrorCode.HTTP_4XX,
                "Calendar event update requires event_id.",
            )

        body: Dict[str, Any] = {}
        if params.get("summary") or params.get("title"):
            body["summary"] = params.get("summary") or params.get("title")
        for key in ("description", "location"):
            if key in params:
                body[key] = params[key]
        if params.get("start") or params.get("start_time") or params.get("datetime"):
            body["start"] = self._event_time(
                params.get("start") or params.get("start_time") or params.get("datetime"),
                params,
            )
        if params.get("end") or params.get("end_time"):
            body["end"] = self._event_time(params.get("end") or params.get("end_time"), params)
        if not body:
            return IntegrationResult.failure(
                IntegrationErrorCode.HTTP_4XX,
                "Calendar event update has no fields to change.",
            )

        result = self._request("PATCH", f"{self._events_path(params)}/{event_id}", json=body)
        if not result.ok:
            return result
        return IntegrationResult.success(
            {
                "type": "calendar",
                "action": "update",
                "event": self._normalize_event(result.data or {}),
            }
        )

    def _delete_event(self, params: Dict[str, Any]) -> IntegrationResult:
        event_id = (params.get("event_id") or params.get("id") or "").strip()
        if not event_id:
            return IntegrationResult.failure(
                IntegrationErrorCode.HTTP_4XX,
                "Calendar event deletion requires event_id.",
            )

        result = self._request("DELETE", f"{self._events_path(params)}/{event_id}")
        if not result.ok:
            return result
        return IntegrationResult.success(
            {
                "type": "calendar",
                "action": "delete",
                "calendar_id": self._calendar_id(params),
                "event_id": event_id,
            }
        )

    def _request(
        self,
        method: str,
        path: str,
        params: Optional[Dict[str, Any]] = None,
        json: Optional[Dict[str, Any]] = None,
    ) -> IntegrationResult:
        token_result = self._get_access_token()
        if not token_result.ok:
            return token_result

        try:
            import requests

            response = requests.request(
                method,
                f"{CALENDAR_API_BASE}{path}",
                headers={"Authorization": f"Bearer {self._access_token}"},
                params=params,
                json=json,
                timeout=self.request_timeout,
            )
        except requests.Timeout:
            return IntegrationResult.failure(
                IntegrationErrorCode.TIMEOUT,
                "Google Calendar request timed out.",
            )
        except Exception as exc:
            return IntegrationResult.failure(
                IntegrationErrorCode.PROVIDER_UNAVAILABLE,
                "Could not connect to Google Calendar.",
                {"error": str(exc)},
            )

        return self._handle_response(response)

    def _get_access_token(self) -> IntegrationResult:
        if self._access_token and time.time() < self._access_token_expires_at - 30:
            return IntegrationResult.success({"type": "google_oauth", "cached": True})

        try:
            import requests

            response = requests.post(
                TOKEN_URL,
                data={
                    "client_id": self.client_id,
                    "client_secret": self.client_secret,
                    "refresh_token": self.refresh_token,
                    "grant_type": "refresh_token",
                },
                timeout=self.request_timeout,
            )
        except requests.Timeout:
            return IntegrationResult.failure(
                IntegrationErrorCode.TIMEOUT,
                "Google OAuth token refresh timed out.",
            )
        except Exception as exc:
            return IntegrationResult.failure(
                IntegrationErrorCode.PROVIDER_UNAVAILABLE,
                "Could not refresh Google OAuth token.",
                {"error": str(exc)},
            )

        if response.status_code in {400, 401}:
            return IntegrationResult.failure(
                IntegrationErrorCode.AUTH_INVALID_KEY,
                "Google Calendar OAuth credentials are invalid or the refresh token expired.",
                self._google_error_debug(response),
            )
        if response.status_code == 429:
            return IntegrationResult.failure(
                IntegrationErrorCode.RATE_LIMITED,
                "Google OAuth rate limit exceeded.",
                {"status_code": response.status_code},
            )
        if response.status_code >= 500:
            return IntegrationResult.failure(
                IntegrationErrorCode.HTTP_5XX,
                "Google OAuth service is temporarily unavailable.",
                {"status_code": response.status_code},
            )
        if response.status_code >= 400:
            return IntegrationResult.failure(
                IntegrationErrorCode.HTTP_4XX,
                "Google OAuth token refresh was rejected.",
                self._google_error_debug(response),
            )

        payload = self._safe_json(response)
        token = str(payload.get("access_token", "") or "")
        if not token:
            return IntegrationResult.failure(
                IntegrationErrorCode.AUTH_INVALID_KEY,
                "Google OAuth token response did not include an access token.",
                {"status_code": response.status_code},
            )
        self._access_token = token
        self._access_token_expires_at = time.time() + float(payload.get("expires_in", 3600) or 3600)
        return IntegrationResult.success({"type": "google_oauth", "cached": False})

    def _handle_response(self, response: Any) -> IntegrationResult:
        if response.status_code in {200, 201, 204}:
            return IntegrationResult.success({} if response.status_code == 204 else self._safe_json(response))
        if response.status_code in {401, 403}:
            return IntegrationResult.failure(
                IntegrationErrorCode.AUTH_INVALID_KEY,
                "Google Calendar credentials are invalid or do not have calendar access.",
                self._google_error_debug(response),
            )
        if response.status_code == 429:
            return IntegrationResult.failure(
                IntegrationErrorCode.RATE_LIMITED,
                "Google Calendar API rate limit exceeded.",
                {"status_code": response.status_code},
            )
        if 400 <= response.status_code < 500:
            return IntegrationResult.failure(
                IntegrationErrorCode.HTTP_4XX,
                "Google Calendar rejected the request.",
                self._google_error_debug(response),
            )
        return IntegrationResult.failure(
            IntegrationErrorCode.HTTP_5XX,
            "Google Calendar service is temporarily unavailable.",
            {"status_code": response.status_code},
        )

    def _events_path(self, params: Dict[str, Any]) -> str:
        return f"/calendars/{self._calendar_id(params)}/events"

    def _calendar_id(self, params: Dict[str, Any]) -> str:
        return str(params.get("calendar_id") or params.get("calendarId") or self.calendar_id or "primary")

    def _event_time(self, value: Any, params: Dict[str, Any]) -> Dict[str, str]:
        if isinstance(value, dict):
            return value
        text = str(value or "").strip()
        if re.fullmatch(r"\d{4}-\d{2}-\d{2}", text):
            return {"date": text}
        time_zone = str(params.get("time_zone") or params.get("timeZone") or self.time_zone)
        return {"dateTime": text, "timeZone": time_zone}

    @staticmethod
    def _normalize_event(item: Dict[str, Any]) -> Dict[str, Any]:
        start = item.get("start", {}) if isinstance(item.get("start"), dict) else {}
        end = item.get("end", {}) if isinstance(item.get("end"), dict) else {}
        return {
            "id": item.get("id", ""),
            "title": item.get("summary", ""),
            "start": start.get("dateTime") or start.get("date", ""),
            "end": end.get("dateTime") or end.get("date", ""),
            "location": item.get("location", ""),
            "description": item.get("description", ""),
            "status": item.get("status", ""),
            "html_link": item.get("htmlLink", ""),
        }

    @staticmethod
    def _safe_json(response: Any) -> Dict[str, Any]:
        try:
            payload = response.json()
            return payload if isinstance(payload, dict) else {}
        except Exception:
            return {}

    @classmethod
    def _google_error_debug(cls, response: Any) -> Dict[str, Any]:
        payload = cls._safe_json(response)
        error = payload.get("error", {}) if isinstance(payload.get("error"), dict) else {}
        return {
            "status_code": getattr(response, "status_code", None),
            "reason": error.get("status") or error.get("reason") or error.get("message", ""),
        }

    @staticmethod
    def _coerce_int(value: Any, default: int, minimum: int, maximum: int) -> int:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            parsed = default
        return min(max(parsed, minimum), maximum)

    @staticmethod
    def _missing_event_fields(summary: str, start: Any, end: Any) -> list[str]:
        missing = []
        if not summary:
            missing.append("title")
        if not start:
            missing.append("start")
        if not end:
            missing.append("end")
        return missing
