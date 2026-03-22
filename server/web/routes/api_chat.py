import asyncio

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from ..app import get_agent, broadcast, _loop
from ..auth import require_auth

router = APIRouter(prefix="/api/chat", dependencies=[Depends(require_auth)])


class ChatMessage(BaseModel):
    text: str
    speaker_id: str = "web_user"


@router.post("/")
def chat(body: ChatMessage):
    agent = get_agent()
    response, intent = agent.generate_response(body.text, speaker_id=body.speaker_id)
    emotion = agent.emotion_system.current_emotion

    # Broadcast chat event to WebSocket clients
    from ..app import _loop as loop
    if loop is not None and loop.is_running():
        asyncio.run_coroutine_threadsafe(
            broadcast({
                "event": "chat_response",
                "text": body.text,
                "response": response,
                "intent": intent,
                "emotion": emotion,
                "speaker_id": body.speaker_id,
            }),
            loop,
        )

    return {
        "response": response,
        "intent": intent,
        "emotion": emotion,
    }
