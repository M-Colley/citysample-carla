#!/usr/bin/env python3
"""
doctor.py - prerequisite check for running Epic's City Sample as a CARLA server.

    python tools/doctor.py              # part B: build and host the whole city
    python tools/doctor.py --part-a     # part A only: export the road network

Part A needs far less: UE 5.8, the City Sample, Python, and a CARLA client to
test the .xodr against. Part B additionally needs a source build of CARLA
against its UE 5.8 fork, which is where the disk and toolchain requirements
come from.

Exit status is 1 if anything required is missing, so this is usable in CI.
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys

# --- disk requirements, from CARLA's own Docs/build_windows_ue5.md -----------
#   "this takes more than 1 extra hour of build and a 225Gb of disk space"
UE_BUILD_GB = 225          # CARLA's UE 5.8 fork, source build
CARLA_BUILD_GB = 60        # CARLA itself + content + intermediates (estimate)
CITY_SAMPLE_GB = 100       # City Sample project + derived data (estimate)
TOTAL_GB = UE_BUILD_GB + CARLA_BUILD_GB + CITY_SAMPLE_GB
# Part A does not build CARLA or its engine fork.
PART_A_GB = CITY_SAMPLE_GB

OK, WARN, BAD = "OK", "WARN", "MISSING"
_results: list[tuple[str, str, str]] = []


def record(status: str, name: str, detail: str = "") -> None:
    _results.append((status, name, detail))


def run(cmd: list[str], timeout: int = 15) -> str | None:
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    return (out.stdout or out.returncode and "" or out.stdout).strip()


def check_tool(name: str, cmd: list[str] | None = None, required: bool = True) -> str | None:
    path = shutil.which(name)
    if not path:
        record(BAD if required else WARN, name, "not on PATH")
        return None
    ver = run(cmd) if cmd else None
    first = ver.splitlines()[0] if ver else path
    record(OK, name, first)
    return path


def check_disk(part_a: bool = False) -> None:
    for drive in (["C:\\", "D:\\"] if platform.system() == "Windows" else ["/"]):
        try:
            total, used, free = shutil.disk_usage(drive)
        except OSError:
            continue
        free_gb = free / 2**30
        # Part A never builds the engine or CARLA - it only needs room beside an
        # existing City Sample project - so holding it to the part B figure
        # reports a blocker that is not one.
        need = PART_A_GB if part_a else TOTAL_GB
        floor = 0 if part_a else UE_BUILD_GB
        # A build that only just fits will fail partway through: cooking and the
        # Zen store need slack on top of the finished tree. Want 15% headroom.
        if free_gb >= need * 1.15:
            status, note = OK, ""
        elif free_gb >= need:
            status = WARN
            note = f" -- only {free_gb - need:,.0f} GiB spare; cooking needs slack"
        elif free_gb >= floor:
            status = WARN
            note = " -- enough for the engine, not for engine + CARLA + City Sample"
        else:
            status, note = BAD, f" -- below the {need} GiB this needs"
        record(status, f"disk {drive}",
               f"{free_gb:,.0f} GiB free (need ~{need} GiB{'' if part_a else ' total'}){note}")


def check_gpu() -> None:
    out = run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version",
               "--format=csv,noheader"])
    if not out:
        record(BAD, "nvidia-smi", "no NVIDIA GPU visible")
        return
    record(OK, "GPU", out)
    try:
        mib = int(out.split(",")[1].strip().split()[0])
    except (IndexError, ValueError):
        return
    gb = mib / 1024
    # City Sample at Epic quality with Nanite and Lumen is the demanding part;
    # CARLA's sensors then render additional views on top of it.
    if gb < 8:
        record(WARN, "VRAM", f"{gb:.1f} GB - City Sample at Epic quality wants 8 GB+; "
                             f"use -Quality Medium")
    else:
        record(OK, "VRAM", f"{gb:.1f} GB")


def check_windows(part_a: bool = False) -> None:
    record(OK, "side", "Windows host - builds and runs the CARLA server")
    check_disk(part_a)
    check_gpu()
    check_tool("git", ["git", "--version"])
    check_tool("cmake", ["cmake", "--version"])   # CARLA needs >= 3.27.2
    check_tool("ninja", ["ninja", "--version"])
    check_tool("python", ["python", "--version"])

    # Visual Studio 2022 with the C++ toolset.
    vswhere = os.path.join(
        os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
        "Microsoft Visual Studio", "Installer", "vswhere.exe",
    )
    if not os.path.exists(vswhere):
        record(BAD, "Visual Studio 2022", "vswhere.exe not found")
    else:
        got = run([vswhere, "-latest", "-products", "*", "-requires",
                   "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                   "-format", "value", "-property", "installationPath"])
        record(OK if got else BAD, "Visual Studio 2022 C++ toolset",
               got or "installed, but the C++ x64 toolset component is missing")

    # The CARLA UE fork is gated: you must link GitHub <-> Epic Games.
    ue = os.environ.get("CARLA_UNREAL_ENGINE_PATH")
    if ue and os.path.isdir(ue):
        record(OK, "CARLA_UNREAL_ENGINE_PATH", ue)
    else:
        record(
            WARN, "CARLA_UNREAL_ENGINE_PATH",
            "unset. CarlaSetup.bat will clone + build CarlaUnreal/UnrealEngine "
            "(branch ue58-dev-carla): ~225 GB and >1 h. Requires a GitHub account "
            "linked to Epic Games - https://www.unrealengine.com/en-US/ue-on-github",
        )

    # The Python client the tools use.
    try:
        import carla  # noqa: F401
        record(OK, "carla python module", getattr(carla, "__version__", "unknown"))
    except ImportError:
        record(
            WARN, "carla python module",
            "not importable. Part A can use any released client (pip install carla); "
            "part B needs the 0.10.0 wheel built from your own tree, at "
            "Build/PythonAPI/dist/carla-0.10.0-*.whl",
        )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--part-a", action="store_true",
                    help="only check what part A needs (road-network export)")
    args = ap.parse_args()

    scope = "  [part A only]" if args.part_a else ""
    print()
    print("  doctor - Epic City Sample as a CARLA 0.10.0 (UE 5.8) server" + scope)
    print(f"  {platform.system()} {platform.release()}  python {platform.python_version()}\n")

    if platform.system() != "Windows":
        record(WARN, "platform",
               f"{platform.system()} - this pipeline is Windows-only "
               f"(UE build scripts, PowerShell tooling)")
    check_windows(part_a=args.part_a)

    width = max(len(n) for _, n, _ in _results) + 2
    for status, name, detail in _results:
        mark = {OK: "  ok  ", WARN: " warn ", BAD: " MISS "}[status]
        print(f"[{mark}] {name:<{width}} {detail}")

    bad = sum(1 for s, _, _ in _results if s == BAD)
    warn = sum(1 for s, _, _ in _results if s == WARN)
    print(f"\n  {len(_results) - bad - warn} ok, {warn} warnings, {bad} missing\n")

    if args.part_a:
        print("  (part A only. Drop --part-a to check the part B build too.)\n")

    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
