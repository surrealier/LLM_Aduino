#!/usr/bin/env python3
"""Bootstrap the lightweight CLI and print the next setup command."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    install_cmd = [sys.executable, "-m", "pip", "install", "-e", "."]

    print(f"running: {' '.join(install_cmd)}")
    install = subprocess.run(install_cmd, cwd=str(root), check=False)
    if install.returncode != 0:
        return install.returncode

    print("bootstrap complete.")
    print("next: python3 -m ccoli setup")
    print("after the CLI is on PATH, you can also run: ccoli setup")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
