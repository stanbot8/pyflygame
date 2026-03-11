"""Launch FWMC brain viewer (with integrated body sim).

Usage:
    python -m nmfly.launcher
    python -m nmfly --backend launch
"""

import logging
import os
import subprocess
import sys

log = logging.getLogger("nmfly.launcher")

VIEWER_EXE_PATHS = [
    "../fwmc/build/viewer/Release/fwmc-viewer.exe",
    "../fwmc/build/viewer/Debug/fwmc-viewer.exe",
    "d:/fwmc-build/viewer/Release/fwmc-viewer.exe",
    "d:/fwmc-build/viewer/Debug/fwmc-viewer.exe",
]


def _find_exe(paths: list[str]) -> str:
    for p in paths:
        if os.path.isfile(p):
            return os.path.abspath(p)
    return ""


def launch(viewer_exe: str = ""):
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
        description="Launch FWMC brain viewer with integrated body sim"
    )
    parser.add_argument("--viewer-exe", default="",
                        help="Path to fwmc-viewer.exe")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    launch(viewer_exe=args.viewer_exe)


if __name__ == "__main__":
    main()
