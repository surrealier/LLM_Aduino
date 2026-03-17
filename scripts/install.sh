#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${CCOLI_GIT_URL:-https://github.com/surrealier/ccoli.git}"
REPO_REF="${CCOLI_GIT_REF:-main}"
PYTHON_BIN="${CCOLI_PYTHON:-python3}"
INSTALL_SOURCE="${CCOLI_INSTALL_SOURCE:-auto}"
RUN_SETUP="${CCOLI_RUN_SETUP:-0}"
DRY_RUN="${CCOLI_DRY_RUN:-0}"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "error: python3 is required to install ccoli" >&2
  exit 1
fi

USER_FLAG=()
if [[ -z "${VIRTUAL_ENV:-}" && -z "${CONDA_PREFIX:-}" ]]; then
  USER_FLAG=(--user)
fi

USER_BIN="$("$PYTHON_BIN" - <<'PY'
import site
print(site.USER_BASE + "/bin")
PY
)"

SCRIPT_DIR=""
if [[ -n "${BASH_SOURCE[0]:-}" && -f "${BASH_SOURCE[0]}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

LOCAL_SOURCE=""
if [[ -n "${CCOLI_LOCAL_SOURCE:-}" ]]; then
  LOCAL_SOURCE="${CCOLI_LOCAL_SOURCE}"
elif [[ -n "$SCRIPT_DIR" && -f "$SCRIPT_DIR/../pyproject.toml" ]]; then
  LOCAL_SOURCE="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

INSTALL_CMD=("$PYTHON_BIN" -m pip install)
if [[ ${#USER_FLAG[@]} -gt 0 ]]; then
  INSTALL_CMD+=("${USER_FLAG[@]}")
fi
if [[ "$INSTALL_SOURCE" == "local" || ( "$INSTALL_SOURCE" == "auto" && -n "$LOCAL_SOURCE" ) ]]; then
  if [[ -z "$LOCAL_SOURCE" ]]; then
    echo "error: local install requested but no local source path was found" >&2
    exit 1
  fi
  INSTALL_CMD+=(-e "$LOCAL_SOURCE")
else
  INSTALL_CMD+=(--upgrade "git+${REPO_URL}@${REPO_REF}")
fi

SETUP_CMD=("ccoli" "setup" "$@")
PYTHON_SETUP_CMD=("$PYTHON_BIN" -m ccoli setup "$@")

if [[ "$DRY_RUN" == "1" ]]; then
  echo "INSTALL_CMD=${INSTALL_CMD[*]}"
  echo "NEXT_CMD=${SETUP_CMD[*]}"
  echo "FALLBACK_NEXT_CMD=${PYTHON_SETUP_CMD[*]}"
  echo "USER_BIN=${USER_BIN}"
  exit 0
fi

echo "running: ${INSTALL_CMD[*]}"
"${INSTALL_CMD[@]}"

if [[ ${#USER_FLAG[@]} -gt 0 && ":$PATH:" != *":${USER_BIN}:"* ]]; then
  echo "note: ccoli was installed into ${USER_BIN}, which is not on PATH."
  echo "add it for this shell with:"
  echo "  export PATH=\"${USER_BIN}:\$PATH\""
fi

if [[ "$RUN_SETUP" == "1" ]]; then
  if command -v ccoli >/dev/null 2>&1; then
    if [[ -r /dev/tty ]]; then
      exec "${SETUP_CMD[@]}" </dev/tty
    fi
    exec "${SETUP_CMD[@]}"
  fi

  if [[ -r /dev/tty ]]; then
    exec "${PYTHON_SETUP_CMD[@]}" </dev/tty
  fi
  exec "${PYTHON_SETUP_CMD[@]}"
fi

echo "bootstrap complete."
if command -v ccoli >/dev/null 2>&1; then
  echo "next: ${SETUP_CMD[*]}"
else
  echo "next: ${PYTHON_SETUP_CMD[*]}"
  echo "after PATH is updated, you can also run: ${SETUP_CMD[*]}"
fi
