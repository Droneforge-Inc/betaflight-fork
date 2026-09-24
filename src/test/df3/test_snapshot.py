#!/usr/bin/env python3
"""Exercise the actual FC snapshot producer and MTF adapter with UBSan."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[3]
with tempfile.TemporaryDirectory(prefix="bf-df3-snapshot-") as directory:
    binary = str(Path(directory) / "test")
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu99", "-g", "-O1", "-fsanitize=undefined", "-fno-sanitize-recover=all",
        "-ffunction-sections", "-fdata-sections", "-DUNIT_TEST", "-DUSE_DF3",
        "-DUSE_RANGEFINDER_OPTFLOW_MTF", "-DUSE_OPTICALFLOW", "-DFLASH_SIZE=128",
        "-Isrc/test/unit", "-Isrc/main", "-Isrc/main/target",
        "-Wl,-dead_strip" if sys.platform == "darwin" else "-Wl,--gc-sections",
        "src/test/df3/snapshot_test.c", "src/main/flight/df3/df3_mlrs.c",
        "src/main/flight/df3/df3_state.c", "src/main/flight/df3/df3_reference.c",
        "src/main/flight/df3/df3_control.c", "src/main/fc/rc_modes.c", "src/main/common/bitarray.c",
        "-o", binary,
    ]
    subprocess.run(command, cwd=root, check=True, timeout=60)
    subprocess.run([binary], check=True, timeout=10)
