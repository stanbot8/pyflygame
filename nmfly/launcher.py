"""Launch flygame C++ sim or FWMC brain viewer.

Usage:
    python -m nmfly.launcher
    python -m nmfly --backend launch
"""

import logging
import os
import subprocess
import sys

from .flygame import find_flygame_exe

log = logging.getLogger("nmfly.launcher")

VIEWER_EXE_PATHS = [
    "../fwmc/build/viewer/Release/fwmc-viewer.exe",
    "../fwmc/build/viewer/Debug/fwmc-viewer.exe",
    "../fwmc-build/viewer/Release/fwmc-viewer.exe",
    "../fwmc-build/viewer/Debug/fwmc-viewer.exe",
]


def _find_exe(paths: list[str]) -> str:
    for p in paths:
        if os.path.isfile(p):
            return os.path.abspath(p)
    return ""


def launch(exe: str = "", headless: bool = False,
           extra_args: list[str] | None = None):
    """Launch nmfly-sim (flygame C++ executable).

    If exe is not provided, searches common build paths and PATH.
    """
    if not exe:
        exe = find_flygame_exe() or ""
    if not exe:
        log.error("Cannot find nmfly-sim. Build flygame or pass --exe path.")
        sys.exit(1)

    cmd = [exe]
    if headless:
        cmd.append("--headless")
    if extra_args:
        cmd.extend(extra_args)

    log.info("Starting: %s", " ".join(cmd))
    proc = subprocess.Popen(cmd)

    try:
        proc.wait()
        log.info("flygame exited (code %d)", proc.returncode)
    except KeyboardInterrupt:
        log.info("Interrupted")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def launch_viewer(viewer_exe: str = ""):
    """Launch the FWMC brain viewer."""
    if not viewer_exe:
        viewer_exe = _find_exe(VIEWER_EXE_PATHS)
    if not viewer_exe:
        log.error("Cannot find fwmc-viewer.exe. Pass --viewer-exe path.")
        sys.exit(1)

    log.info("Starting viewer: %s", viewer_exe)
    proc = subprocess.Popen([viewer_exe])

    try:
        proc.wait()
        log.info("Viewer exited (code %d)", proc.returncode)
    except KeyboardInterrupt:
        log.info("Interrupted")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Launch flygame C++ simulator"
    )
    parser.add_argument("--exe", default="",
                        help="Path to nmfly-sim executable")
    parser.add_argument("--headless", action="store_true",
                        help="Run without viewer")
    parser.add_argument("-v", "--verbose", action="store_true")
    args, extra = parser.parse_known_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    launch(exe=args.exe, headless=args.headless, extra_args=extra)


if __name__ == "__main__":
    main()
