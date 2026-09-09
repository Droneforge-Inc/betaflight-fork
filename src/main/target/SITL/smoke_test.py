#!/usr/bin/env python3
"""Exercise standalone firmware I/O. No vehicle or motor physics is simulated."""
import argparse
import math
import socket
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path

def request(sock, command, payload=b""):
    checksum = len(payload) ^ command
    for value in payload:
        checksum ^= value
    sock.sendall(b"$M<" + bytes([len(payload), command]) + payload + bytes([checksum]))
    def read(n):
        data = b""
        while len(data) < n:
            part = sock.recv(n - len(data))
            if not part:
                raise RuntimeError("MSP connection closed")
            data += part
        return data
    while True:
        header = read(3)
        if header != b"$M>":
            raise AssertionError(f"Unexpected MSP header: {header!r}")
        size, received_command = read(2)
        data = read(size)
        received_checksum = read(1)[0]
        check = size ^ received_command
        for value in data:
            check ^= value
        assert check == received_checksum, "Bad MSP checksum"
        if received_command == command:
            return data

def run(binary, directory, save):
    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    output.bind(("127.0.0.1", 9002))
    output.settimeout(5)
    stop = threading.Event()
    feed_error = []
    rates = [0.0, 0.0, 0.0]
    def feed():
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        start = time.monotonic()
        try:
            while not stop.is_set():
                t = time.monotonic() - start
                # Historical SITL wire convention: level specific force is -g Z.
                packet = struct.pack("<18d", t, *rates, 0, 0, -9.80665,
                                     1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 101325)
                udp.sendto(packet, ("127.0.0.1", 9003))
                channels = [1500, 1500, 1000, 1500] + [1000] * 12
                udp.sendto(struct.pack("<d16H", t, *channels), ("127.0.0.1", 9004))
                stop.wait(0.0002)
        except Exception as error:
            feed_error.append(error)
        finally:
            udp.close()
    with (directory / ("first.log" if save else "restart.log")).open("w") as log:
        process = subprocess.Popen([str(binary)], cwd=directory, stdout=log, stderr=log)
        worker = threading.Thread(target=feed)
        worker.start()
        try:
            deadline = time.monotonic() + 8
            while True:
                if process.poll() is not None:
                    raise RuntimeError(f"SITL exited {process.returncode}")
                try:
                    client = socket.create_connection(("127.0.0.1", 5761), timeout=0.2)
                    break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.05)
            with client:
                client.settimeout(5)
                version = request(client, 3)
                assert len(version) == 3
                motors = struct.unpack("<4f", output.recv(100))
                assert all(math.isfinite(x) and abs(x) < 0.001 for x in motors), motors
                # Allow normal firmware sensor startup/calibration.
                time.sleep(4)
                imu = struct.unpack("<9h", request(client, 102))
                assert abs(imu[2] - 512) < 20 or abs(imu[2] - 256) < 20, imu
                attitude = struct.unpack("<3h", request(client, 108))
                assert abs(attitude[0]) < 20 and abs(attitude[1]) < 20, attitude
                rates[:] = [0.1, 0.2, 0.3]
                time.sleep(0.5)
                moving_imu = struct.unpack("<9h", request(client, 102))
                assert moving_imu[3] > 0 and moving_imu[4] < 0 and moving_imu[5] < 0, moving_imu
                rates[:] = [0.0, 0.0, 0.0]
                # Disarmed MSP motor test checks the output backend, not arming/PID fidelity.
                request(client, 214, struct.pack("<8H", 1200, 1300, 1400, 1500, 1000, 1000, 1000, 1000))
                time.sleep(0.1)
                output.setblocking(False)
                latest = None
                while True:
                    try:
                        latest = struct.unpack("<4f", output.recv(100))
                    except BlockingIOError:
                        break
                output.settimeout(2)
                latest = struct.unpack("<4f", output.recv(100))
                expected = (0.3, 0.4, 0.5, 0.2)  # historical Gazebo remapping
                assert latest is not None and all(abs(a-b) < 0.002 for a,b in zip(latest, expected)), latest
                request(client, 214, struct.pack("<8H", *([1000] * 8)))
                marker = b"DF_SIM_SMOKE"
                if save:
                    request(client, 11, marker)  # MSP_SET_NAME
                    request(client, 250)        # MSP_EEPROM_WRITE
                assert request(client, 10).rstrip(b"\0") == marker
                assert (directory / "eeprom.bin").stat().st_size == 32768
                assert not feed_error, feed_error
                print(f"PASS: version={tuple(version)}, motors={motors}, imu={imu[:6]}, attitude={attitude}, EEPROM={'saved' if save else 'reloaded'}")
        finally:
            stop.set()
            worker.join(timeout=2)
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            output.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("binary", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="df-sitl-smoke-") as temporary:
        folder = Path(temporary)
        try:
            run(args.binary.resolve(), folder, True)
            run(args.binary.resolve(), folder, False)
        except Exception:
            for log in folder.glob("*.log"):
                print(log.read_text()[-5000:])
            raise
