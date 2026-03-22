from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, Query
from pydantic import BaseModel

from ..app import get_agent
from ..auth import require_auth

router = APIRouter(prefix="/api/schedules", dependencies=[Depends(require_auth)])


class ScheduleCreate(BaseModel):
    title: str
    datetime: str       # ISO 8601 string
    description: str = ""
    reminder_before: int = 0  # minutes


@router.get("/")
def list_schedules():
    agent = get_agent()
    return {"schedules": agent.scheduler.schedules}


@router.get("/upcoming")
def upcoming_schedules(hours: int = Query(24, ge=1)):
    agent = get_agent()
    return {"schedules": agent.scheduler.get_upcoming_schedules(hours)}


@router.post("/")
def create_schedule(body: ScheduleCreate):
    agent = get_agent()
    try:
        dt = datetime.fromisoformat(body.datetime)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid datetime format (use ISO 8601)")
    msg = agent.scheduler.add_schedule(
        body.title, dt, body.description, body.reminder_before
    )
    return {"ok": True, "message": msg}


@router.delete("/{schedule_id}")
def delete_schedule(schedule_id: int):
    agent = get_agent()
    msg = agent.scheduler.delete_schedule(schedule_id)
    return {"ok": True, "message": msg}


@router.post("/{schedule_id}/complete")
def complete_schedule(schedule_id: int):
    agent = get_agent()
    msg = agent.scheduler.complete_schedule(schedule_id)
    return {"ok": True, "message": msg}
