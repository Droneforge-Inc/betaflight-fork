#!/usr/bin/env python3

import argparse
import struct
import sys

import serial
from serial.tools import list_ports


MSP_BOARD_INFO = 4
MSP_UID = 160
SIGNATURE_LENGTH = 32


def msp_v1_request(cmd):
    return bytes((ord("$"), ord("M"), ord("<"), 0, cmd, cmd))


def read_exact(port, size):
    data = bytearray()
    while len(data) < size:
        chunk = port.read(size - len(data))
        if not chunk:
            raise TimeoutError("timed out waiting for MSP response")
        data.extend(chunk)
    return bytes(data)


def read_msp_v1_response(port, expected_cmd):
    while True:
        if read_exact(port, 1) != b"$":
            continue

        header = read_exact(port, 2)
        if header[0:1] != b"M":
            continue

        direction = header[1:2]
        if direction not in (b">", b"!"):
            continue

        size, cmd = struct.unpack("<BB", read_exact(port, 2))
        payload = read_exact(port, size)
        checksum = read_exact(port, 1)[0]

        calc = size ^ cmd
        for byte in payload:
            calc ^= byte

        if calc != checksum:
            raise ValueError("invalid MSP checksum")

        if cmd != expected_cmd:
            continue

        if direction == b"!":
            raise ValueError(f"MSP command {cmd} returned an error")

        return payload


def parse_board_info(payload):
    offset = 0

    def read_u8():
        nonlocal offset
        value = payload[offset]
        offset += 1
        return value

    def read_u16():
        nonlocal offset
        value = struct.unpack_from("<H", payload, offset)[0]
        offset += 2
        return value

    def read_u32():
        nonlocal offset
        value = struct.unpack_from("<I", payload, offset)[0]
        offset += 4
        return value

    def read_bytes(length):
        nonlocal offset
        value = payload[offset:offset + length]
        offset += length
        return value

    def read_string():
        return read_bytes(read_u8()).decode("ascii", errors="replace")

    info = {
        "board_identifier": read_bytes(4).decode("ascii", errors="replace"),
        "hardware_revision": read_u16(),
        "fc_type": read_u8(),
        "target_capabilities": read_u8(),
        "target_name": read_string(),
        "board_name": read_string(),
        "manufacturer_id": read_string(),
        "signature": read_bytes(SIGNATURE_LENGTH).hex(),
        "mcu_type_id": read_u8(),
    }

    if offset < len(payload):
        info["configuration_state"] = read_u8()
    if offset + 2 <= len(payload):
        info["gyro_sample_rate_hz"] = read_u16()
    if offset + 4 <= len(payload):
        info["configuration_problems"] = read_u32()
    if offset < len(payload):
        info["spi_device_count"] = read_u8()
    if offset < len(payload):
        info["i2c_device_count"] = read_u8()

    return info


def parse_uid(payload):
    uid0, uid1, uid2 = struct.unpack("<III", payload)
    return {
        "msp_uid": f"{uid0:08x}{uid1:08x}{uid2:08x}",
        "usb_serial_from_uid": f"{(uid0 + uid2) & 0xffffffff:08x}{uid1:08x}"[:12],
    }


def find_usb_metadata(device):
    for port in list_ports.comports():
        if port.device == device:
            return {
                "usb_serial_number": port.serial_number,
                "usb_location": port.location,
                "usb_vid": None if port.vid is None else f"{port.vid:04x}",
                "usb_pid": None if port.pid is None else f"{port.pid:04x}",
                "usb_manufacturer": port.manufacturer,
                "usb_product": port.product,
            }
    return {}


def main():
    parser = argparse.ArgumentParser(description="Read MSP_BOARD_INFO from a Betaflight flight controller")
    parser.add_argument("port", help="Serial port, for example /dev/tty.usbmodem1234 or COM3")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument("--timeout", type=float, default=2.0, help="Read timeout in seconds (default: 2.0)")
    args = parser.parse_args()

    info = find_usb_metadata(args.port)

    with serial.Serial(args.port, args.baudrate, timeout=args.timeout, write_timeout=args.timeout) as port:
        port.reset_input_buffer()
        port.reset_output_buffer()
        port.write(msp_v1_request(MSP_BOARD_INFO))
        port.flush()
        info.update(parse_board_info(read_msp_v1_response(port, MSP_BOARD_INFO)))

        port.write(msp_v1_request(MSP_UID))
        port.flush()
        info.update(parse_uid(read_msp_v1_response(port, MSP_UID)))

    for key, value in info.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
