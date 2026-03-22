from fastapi import APIRouter, Depends, Query
from ..app import get_agent
from ..auth import require_auth

router = APIRouter(prefix="/api/conversation", dependencies=[Depends(require_auth)])


@router.get("/")
def get_conversation(
    limit: int = Query(50, ge=1, le=500),
    speaker_id: str = Query(None),
):
    agent = get_agent()
    if speaker_id:
        history = list(agent.user_histories.get(speaker_id, []))
    else:
        history = list(agent.conversation_history)
    return {"history": history[-limit:], "total": len(history)}


@router.get("/speakers")
def list_speakers():
    agent = get_agent()
    return {"speakers": list(agent.user_histories.keys())}


@router.delete("/")
def clear_conversation():
    agent = get_agent()
    agent.conversation_history.clear()
    return {"ok": True}
