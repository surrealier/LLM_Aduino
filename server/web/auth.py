"""
Optional token-based authentication for the web dashboard.
Set web.auth_token in config.yaml to enable. Leave blank to disable.
Clients pass the token via the X-Auth-Token header.
"""
from fastapi import HTTPException, Security, status
from fastapi.security.api_key import APIKeyHeader

_API_KEY_HEADER = APIKeyHeader(name="X-Auth-Token", auto_error=False)

_token: str = ""


def configure(token: str):
    global _token
    _token = (token or "").strip()


async def require_auth(key: str = Security(_API_KEY_HEADER)):
    if not _token:
        return  # auth disabled
    if key != _token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or missing X-Auth-Token header",
        )
