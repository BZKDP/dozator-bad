#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Continuous back-and-forth MG90S test for the Gate12 ESP32 firmware."""

from __future__ import annotations

import argparse
import time

import serial


def read_available(ser: serial.Serial, seconds: float = 0.25) -> list[str]:
    lines: list[str] = []
    end = time.time() + seconds
    while time.time() < end:
        raw = ser.readline()
        if raw:
            lines.append(raw.decode("utf-8", errors="replace").strip())
    return lines


def send(ser: serial.Serial, line: str, pause: float = 0.1) -> None:
    ser.write((line + "\n").encode("ascii"))
    ser.flush()
    print(f"> {line}", flush=True)
    time.sleep(pause)
    for response in read_available(ser, 0.2):
        print(f"< {response}", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--left", type=int, default=1000)
    ap.add_argument("--right", type=int, default=2000)
    ap.add_argument("--center", type=int, default=1500)
    ap.add_argument("--rate", type=int, default=900)
    ap.add_argument("--dwell", type=float, default=1.35)
    args = ap.parse_args()

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.08
    ser.write_timeout = 1
    ser.dtr = False
    ser.rts = False
    ser.open()

    try:
        time.sleep(0.5)
        for response in read_available(ser, 0.5):
            print(f"< {response}", flush=True)

        lo = min(args.left, args.right, args.center)
        hi = max(args.left, args.right, args.center)
        send(ser, "PING")
        send(ser, f"LIM {lo} {hi}")
        send(ser, f"RATE {args.rate}")
        send(ser, f"SET {args.center}", 0.5)

        side = args.left
        while True:
            send(ser, f"SET {side}", 0.15)
            time.sleep(args.dwell)
            side = args.right if side == args.left else args.left
    finally:
        try:
            send(ser, f"SET {args.center}", 0.2)
        finally:
            ser.close()


if __name__ == "__main__":
    main()
