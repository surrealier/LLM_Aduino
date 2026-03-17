"""Tests for the curl/bootstrap shell installer."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "install.sh"


def _run_bootstrap(env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    merged = os.environ.copy()
    merged.update(env)
    return subprocess.run(
        ["bash", str(SCRIPT)],
        cwd=str(ROOT),
        env=merged,
        check=False,
        capture_output=True,
        text=True,
    )


def test_bootstrap_dry_run_prefers_local_repo_in_auto_mode():
    result = _run_bootstrap(
        {
            "CCOLI_DRY_RUN": "1",
            "CCOLI_INSTALL_SOURCE": "auto",
            "CCOLI_PYTHON": sys.executable,
        }
    )

    assert result.returncode == 0
    assert f"INSTALL_CMD={sys.executable} -m pip install --user -e {ROOT}" in result.stdout
    assert "SETUP_CMD=ccoli setup" in result.stdout


def test_bootstrap_dry_run_can_force_remote_git_install():
    result = _run_bootstrap(
        {
            "CCOLI_DRY_RUN": "1",
            "CCOLI_INSTALL_SOURCE": "remote",
            "CCOLI_GIT_URL": "https://example.com/demo.git",
            "CCOLI_GIT_REF": "stable",
            "CCOLI_PYTHON": sys.executable,
        }
    )

    assert result.returncode == 0
    assert "git+https://example.com/demo.git@stable" in result.stdout
    assert f"PYTHON_SETUP_CMD={sys.executable} -m ccoli setup" in result.stdout
