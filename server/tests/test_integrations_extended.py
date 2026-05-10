from src.integrations import (
    GoogleCalendarIntegration,
    MapsIntegration,
    NotifyIntegration,
    SearchIntegration,
)


def test_search_integration_requires_query_and_key():
    search = SearchIntegration(api_key="")
    assert search.execute("search.query", {"query": "news"}).ok is False


def test_calendar_integration_requires_oauth_settings():
    calendar = GoogleCalendarIntegration("", "", "")
    result = calendar.execute("calendar.list")
    assert result.ok is False
    assert result.error.code.value == "AUTH_MISSING_KEY"


def test_calendar_integration_lists_and_creates_events(monkeypatch):
    class _Resp:
        def __init__(self, status_code, payload):
            self.status_code = status_code
            self._payload = payload

        def json(self):
            return self._payload

    calls = []

    def _fake_post(url, data=None, timeout=None):
        assert url == "https://oauth2.googleapis.com/token"
        assert data["client_id"] == "id"
        assert data["client_secret"] == "secret"
        assert data["refresh_token"] == "refresh"
        assert data["grant_type"] == "refresh_token"
        return _Resp(200, {"access_token": "access-token", "expires_in": 3600})

    def _fake_request(method, url, headers=None, params=None, json=None, timeout=None):
        calls.append((method, url, params, json))
        assert headers == {"Authorization": "Bearer access-token"}
        if method == "GET":
            return _Resp(
                200,
                {
                    "items": [
                        {
                            "id": "evt-1",
                            "summary": "meeting",
                            "start": {"dateTime": "2026-05-11T10:00:00+09:00"},
                            "end": {"dateTime": "2026-05-11T10:30:00+09:00"},
                            "htmlLink": "https://calendar.google.com/event?eid=evt-1",
                        }
                    ]
                },
            )
        return _Resp(
            200,
            {
                "id": "evt-2",
                "summary": json["summary"],
                "start": json["start"],
                "end": json["end"],
            },
        )

    import requests

    monkeypatch.setattr(requests, "post", _fake_post)
    monkeypatch.setattr(requests, "request", _fake_request)

    calendar = GoogleCalendarIntegration("id", "secret", "refresh")
    listed = calendar.execute("calendar.list", {"max_results": 2})
    assert listed.ok is True
    assert listed.data["events"][0]["title"] == "meeting"

    created = calendar.execute(
        "calendar.create",
        {
            "title": "meeting",
            "start": "2026-05-11T10:00:00",
            "end": "2026-05-11T10:30:00",
        },
    )
    assert created.ok is True
    assert created.data["event"]["id"] == "evt-2"
    assert calls[0][0] == "GET"
    assert calls[1][0] == "POST"


def test_notify_integration_send_validation():
    notify = NotifyIntegration("token")
    assert notify.execute("notify.send", {"channel": "", "text": "hi"}).ok is False
    assert notify.execute("notify.send", {"channel": "#general", "text": "hi"}).ok is True


def test_maps_integration_validation():
    maps = MapsIntegration("key")
    assert maps.execute("maps.route", {"origin": "", "destination": "Gangnam"}).ok is False
    result = maps.execute("maps.route", {"origin": "Seoul", "destination": "Gangnam"})
    assert result.ok is True
    assert "Seoul" in result.data["summary"]
