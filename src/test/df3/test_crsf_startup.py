#!/usr/bin/env python3
"""Run the actual CRSF startup task against captured UART output, with UBSan."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[3]
with tempfile.TemporaryDirectory(prefix="bf-crsf-startup-") as directory:
    binary = str(Path(directory) / "test")
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu99", "-g", "-O0", "-fsanitize=undefined", "-fno-sanitize-recover=all",
        "-ffunction-sections", "-fdata-sections",
        "-DUNIT_TEST", "-DUSE_DF3", "-DUSE_CRSF_V3", "-DUSE_CRSF_CMS_TELEMETRY",
        "-DUSE_MSP_OVER_TELEMETRY", "-DFLASH_SIZE=128",
        '-D__TARGET__="TEST"', '-D__REVISION__="revision"',
        "-Isrc/test/unit", "-Isrc/main", "-Isrc/main/target",
        "-Wl,-dead_strip" if sys.platform == "darwin" else "-Wl,--gc-sections",
        "src/main/telemetry/crsf.c", "src/main/common/crc.c", "src/main/common/streambuf.c",
        "src/test/df3/crsf_startup_test.c", "-o", binary,
    ]
    subprocess.run(command, cwd=root, check=True, timeout=60)
    subprocess.run([binary], check=True, timeout=10)
