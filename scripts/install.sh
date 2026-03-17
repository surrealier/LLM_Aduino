#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${CCOLI_GIT_URL:-https://github.com/surrealier/ccoli.git}"
REPO_REF="${CCOLI_GIT_REF:-main}"
PYTHON_BIN="${CCOLI_PYTHON:-python3}"
INSTALL_SOURCE="${CCOLI_INSTALL_SOURCE:-auto}"
SKIP_SETUP="${CCOLI_SKIP_SETUP:-0}"
DRY_RUN="${CCOLI_DRY_RUN:-0}"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "error: python3 is required to install ccoli" >&2
  exit 1
fi

USER_FLAG=()
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  USER_FLAG=(--user)
fi

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

INSTALL_CMD=("$PYTHON_BIN" -m pip install "${USER_FLAG[@]}")
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
  echo "SETUP_CMD=${SETUP_CMD[*]}"
  echo "PYTHON_SETUP_CMD=${PYTHON_SETUP_CMD[*]}"
  exit 0
fi

echo "running: ${INSTALL_CMD[*]}"
"${INSTALL_CMD[@]}"

if [[ "$SKIP_SETUP" == "1" ]]; then
  echo "bootstrap complete. next: ccoli setup"
  exit 0
fi

if command -v ccoli >/dev/null 2>&1; then
  exec "${SETUP_CMD[@]}"
fi

exec "${PYTHON_SETUP_CMD[@]}"
