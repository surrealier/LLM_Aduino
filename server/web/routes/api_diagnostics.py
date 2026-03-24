from __future__ import annotations

from datetime import datetime, timezone
from typing import Literal

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from config_loader import get_config

from ..app import get_agent, get_dashboard_state, get_mode
from ..auth import require_auth

router = APIRouter(prefix="/api/diagnostics", dependencies=[Depends(require_auth)])


class DiagnosticsCheckRequest(BaseModel):
    target: Literal["stt", "connection", "llm"]


def _checked_at() -> str:
    return datetime.now(timezone.utc).isoformat()


def _runtime_snapshot() -> dict:
    agent = get_agent()
    runtime_controller = getattr(agent, "runtime_controller", None)
    if runtime_controller is None:
        return {}
    return runtime_controller.status_snapshot()


def _stt_summary(config, runtime: dict) -> dict:
    stt_runtime = runtime.get("stt", {}) if isinstance(runtime.get("stt"), dict) else {}
    return {
        "model_size": config.get("stt", "model_size", default="medium"),
        "configured_device": config.get("stt", "device", default="cpu"),
        "resolved_devices": list(stt_runtime.get("configured_devices", [])),
        "device_in_use": stt_runtime.get("device_in_use"),
        "language": config.get("stt", "language", default="ko"),
        "notes": list(runtime.get("audio_runtime_notes", [])),
    }


def _connection_summary(config, runtime: dict, dashboard_state: dict) -> dict:
    connection_state = (
        dashboard_state.get("connection", {})
        if isinstance(dashboard_state.get("connection"), dict)
        else {}
    )
    return {
        "mode": config.get("connection", "mode", default="auto"),
        "priority": list(runtime.get("connection_priority", [])),
        "connected": bool(connection_state.get("connected", False)),
        "current_transport": connection_state.get("current_transport"),
        "current_endpoint": connection_state.get("current_endpoint"),
        "last_transport": connection_state.get("last_transport"),
        "last_endpoint": connection_state.get("last_endpoint"),
        "last_connected_at": connection_state.get("last_connected_at"),
        "last_disconnected_at": connection_state.get("last_disconnected_at"),
    }


def _llm_summary(config, runtime: dict) -> dict:
    llm_runtime = runtime.get("llm", {}) if isinstance(runtime.get("llm"), dict) else {}
    llm_cfg = config.get_llm_config()
    return {
        "configured_provider": llm_cfg.get("provider", "ollama"),
        "configured_model": llm_cfg.get("model", ""),
        "priority": list(runtime.get("llm_priority", [])),
        "api_priority": list(runtime.get("api_priority", [])),
        "active_provider": llm_runtime.get("active_provider"),
        "active_model": llm_runtime.get("active_model"),
        "active_bucket": llm_runtime.get("active_bucket"),
        "active_processor": llm_runtime.get("active_processor"),
        "last_error_code": llm_runtime.get("last_error_code"),
    }


def _diagnostics_payload() -> dict:
    config = get_config()
    runtime = _runtime_snapshot()
    dashboard_state = get_dashboard_state()
    warmup = dashboard_state.get("warmup", {}) if isinstance(dashboard_state.get("warmup"), dict) else {}
    last_checks = (
        dashboard_state.get("last_checks", {})
        if isinstance(dashboard_state.get("last_checks"), dict)
        else {}
    )

    return {
        "mode": get_mode(),
        "runtime": {
            "warmup": warmup,
            "llm": _llm_summary(config, runtime),
        },
        "stt": _stt_summary(config, runtime),
        "connection": _connection_summary(config, runtime, dashboard_state),
        "last_checks": last_checks,
    }


def _build_check_result(target: str, diagnostics: dict) -> dict:
    checked_at = _checked_at()

    if target == "stt":
        stt = diagnostics["stt"]
        ok = bool(stt.get("configured_device")) and bool(stt.get("resolved_devices"))
        active = stt.get("device_in_use")
        summary = (
            f"STT active on {active}"
            if active
            else f"STT configured for {stt.get('configured_device')}"
        )
        return {
            "target": target,
            "ok": ok,
            "summary": summary,
            "details": stt,
            "checked_at": checked_at,
        }

    if target == "connection":
        connection = diagnostics["connection"]
        ok = bool(connection.get("connected"))
        transport = connection.get("current_transport") or connection.get("last_transport")
        endpoint = connection.get("current_endpoint") or connection.get("last_endpoint")
        summary = (
            f"Connected via {transport} ({endpoint})"
            if ok and transport and endpoint
            else "No active device connection"
        )
        return {
            "target": target,
            "ok": ok,
            "summary": summary,
            "details": connection,
            "checked_at": checked_at,
        }

    llm = diagnostics["runtime"]["llm"]
    last_error = llm.get("last_error_code")
    ok = not bool(last_error)
    active_provider = llm.get("active_provider") or llm.get("configured_provider")
    active_model = llm.get("active_model") or llm.get("configured_model")
    summary = (
        f"LLM ready: {active_provider} / {active_model}"
        if ok and active_provider
        else f"LLM reported an error: {last_error or 'unknown'}"
    )
    return {
        "target": target,
        "ok": ok,
        "summary": summary,
        "details": llm,
        "checked_at": checked_at,
    }


@router.get("/")
def get_diagnostics():
    return _diagnostics_payload()


@router.post("/check")
def run_diagnostics_check(body: DiagnosticsCheckRequest):
    diagnostics = _diagnostics_payload()
    result = _build_check_result(body.target, diagnostics)
    dashboard_state = get_dashboard_state()
    last_checks = dashboard_state.setdefault("last_checks", {})
    last_checks[body.target] = result
    return result
