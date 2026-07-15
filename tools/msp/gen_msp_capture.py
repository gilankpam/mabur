#!/usr/bin/env python3
"""Write a small valid MSP DisplayPort byte capture to stdout (or a file arg).

Emits N full-screen cycles: CLEAR, a couple of DRAW_STRING rows, DRAW_SCREEN —
with correct MSP v1 xor checksums. For maburd --dry-run --msp-in and bench.
"""
import sys

MSP_CMD_DISPLAYPORT = 182


def msp(cmd, payload):
    out = bytearray([ord('$'), ord('M'), ord('<'), len(payload), cmd])
    cks = len(payload) ^ cmd
    for b in payload:
        out.append(b)
        cks ^= b
    out.append(cks & 0xFF)
    return bytes(out)


def draw_string(row, col, text):
    return msp(MSP_CMD_DISPLAYPORT, [3, row, col, 0] + [ord(c) for c in text])


def main():
    data = bytearray()
    for i in range(3):
        data += msp(MSP_CMD_DISPLAYPORT, [2])                 # CLEAR
        data += draw_string(1, 2, f"MABUR OSD {i}")
        data += draw_string(3, 0, "ALT 123  SPD 45")
        data += msp(MSP_CMD_DISPLAYPORT, [4])                 # DRAW_SCREEN
    out = open(sys.argv[1], "wb") if len(sys.argv) > 1 else sys.stdout.buffer
    out.write(bytes(data))


if __name__ == "__main__":
    main()
