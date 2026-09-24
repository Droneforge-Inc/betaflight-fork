#!/usr/bin/env python3
"""Test actual STM32 persistent initialization and G4 DF3 epoch allocation."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
with tempfile.TemporaryDirectory(prefix="bf-df3-epoch-") as directory:
    binary = str(Path(directory) / "test")
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu99", "-g", "-O1", "-fsanitize=undefined", "-fno-sanitize-recover=all",
        "-Isrc/test/df3/epoch_stubs", "-Isrc/main",
        "src/main/drivers/stm32/persistent.c", "src/main/flight/df3/df3_epoch.c",
        "src/test/df3/epoch_test.c", "-o", binary,
    ]
    subprocess.run(command, cwd=root, check=True, timeout=60)
    subprocess.run([binary], check=True, timeout=10)
