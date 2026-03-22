from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from ..app import get_agent
from ..auth import require_auth

router = APIRouter(prefix="/api/memory", dependencies=[Depends(require_auth)])

VALID_FILES = (
    "Soul.md",
    "User.md",
    "Shortterm_Memory.md",
    "Longterm_Memory.md",
    "Relation.md",
)


@router.get("/")
def list_memory():
    agent = get_agent()
    return {
        "files": [
            {"filename": f, "size": len(agent.memory._cache.get(f, ""))}
            for f in VALID_FILES
        ]
    }


@router.get("/{filename}")
def read_memory(filename: str):
    if filename not in VALID_FILES:
        raise HTTPException(status_code=404, detail=f"Unknown memory file: {filename}")
    agent = get_agent()
    return {"filename": filename, "content": agent.memory._cache.get(filename, "")}


class MemoryUpdate(BaseModel):
    content: str


@router.put("/{filename}")
def update_memory(filename: str, body: MemoryUpdate):
    if filename not in VALID_FILES:
        raise HTTPException(status_code=404, detail=f"Unknown memory file: {filename}")
    agent = get_agent()
    agent.memory._save(filename, body.content)
    return {"ok": True}
