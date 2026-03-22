from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from ..app import get_agent
from ..auth import require_auth

router = APIRouter(prefix="/api/integrations", dependencies=[Depends(require_auth)])


@router.get("/")
def list_integrations():
    agent = get_agent()
    return {"integrations": agent.integrations.list()}


@router.get("/{name}/health")
def health_check(name: str):
    agent = get_agent()
    result = agent.integrations.health_check(name)
    if result is None:
        raise HTTPException(status_code=404, detail=f"Integration not found: {name}")
    return {
        "ok": result.ok,
        "data": result.data,
        "error": vars(result.error) if result.error else None,
    }


class EnableBody(BaseModel):
    enabled: bool


@router.post("/{name}/enabled")
def set_enabled(name: str, body: EnableBody):
    agent = get_agent()
    ok = agent.integrations.set_enabled(name, body.enabled)
    if not ok:
        raise HTTPException(status_code=404, detail=f"Integration not found: {name}")
    return {"ok": True}
