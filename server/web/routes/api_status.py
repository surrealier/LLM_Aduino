from fastapi import APIRouter, Depends
from ..app import get_agent, get_mode
from ..auth import require_auth

router = APIRouter(prefix="/api", dependencies=[Depends(require_auth)])


@router.get("/status")
def get_status():
    try:
        agent = get_agent()
        emotion = agent.emotion_system.current_emotion
        emotion_history = list(agent.emotion_system.emotion_history)
        proactive = agent.proactive.get_stats()
        conv_count = agent.conversation_count
        timers = agent.info_services.get_active_timers()
        alarms = agent.info_services.get_active_alarms()
    except Exception:
        emotion = "neutral"
        emotion_history = []
        proactive = {}
        conv_count = 0
        timers = []
        alarms = []

    return {
        "mode": get_mode(),
        "emotion": emotion,
        "emotion_history": emotion_history,
        "conversation_count": conv_count,
        "proactive": proactive,
        "active_timers": timers,
        "active_alarms": alarms,
    }
