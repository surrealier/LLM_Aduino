from fastapi import APIRouter, Depends, Query
from ..auth import require_auth
from ..log_handler import get_log_ring

router = APIRouter(prefix="/api/logs", dependencies=[Depends(require_auth)])


@router.get("/")
def get_logs(tail: int = Query(100, ge=1, le=500)):
    lines = list(get_log_ring())
    return {"lines": lines[-tail:], "total": len(lines)}
