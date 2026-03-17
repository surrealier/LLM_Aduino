#!/usr/bin/env python3
"""Bootstrap the lightweight CLI, then launch the interactive setup wizard."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    install_cmd = [sys.executable, "-m", "pip", "install", "-e", "."]
    setup_cmd = [sys.executable, "-m", "ccoli", "setup", *sys.argv[1:]]

    print(f"running: {' '.join(install_cmd)}")
    install = subprocess.run(install_cmd, cwd=str(root), check=False)
    if install.returncode != 0:
        return install.returncode

    return subprocess.run(setup_cmd, cwd=str(root), check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
