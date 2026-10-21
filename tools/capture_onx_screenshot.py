import argparse
import base64
import re
import struct
import sys
import time

import serial


BEGIN_RE = re.compile(rb"ONXSHOT:BEGIN:(\d+):(\d+):RGB565LE")
ROW_RE = re.compile(rb"ONXSHOT:ROW:(\d+):(\d+):([A-Za-z0-9+/=]+)")
CHUNK_BYTES = 60


def rgb565_row_to_bgr24(row):
    pixels = bytearray()
    for offset in range(0, len(row), 2):
        color = row[offset] | (row[offset + 1] << 8)
        red = (color >> 11) & 0x1F
        green = (color >> 5) & 0x3F
        blue = color & 0x1F
        pixels.extend(
            (
                (blue << 3) | (blue >> 2),
                (green << 2) | (green >> 4),
                (red << 3) | (red >> 2),
            )
        )
    return pixels


def write_bmp(path, rows, width, height):
    row_size = (width * 3 + 3) & ~3
    pixel_size = row_size * height
    with open(path, "wb") as image:
        image.write(b"BM")
        image.write(struct.pack("<IHHI", 54 + pixel_size, 0, 0, 54))
        image.write(struct.pack("<IiiHHIIiiII", 40, width, height, 1, 24, 0,
                                pixel_size, 2835, 2835, 0, 0))
        padding = bytes(row_size - width * 3)
        for row in reversed(rows):
            image.write(row)
            image.write(padding)


def capture(port, output, timeout):
    deadline = time.monotonic() + timeout
    width = height = None
    rows = {}
    row_chunks = {}
    print(f"Waiting for ONX screenshot on {port}; reset the display board if needed.")

    with serial.Serial(port, 115200, timeout=1) as stream:
        while time.monotonic() < deadline:
            line = stream.readline().strip()
            if not line:
                continue

            match = BEGIN_RE.search(line)
            if match:
                width, height = map(int, match.groups())
                rows.clear()
                row_chunks.clear()
                print(f"Receiving {width}x{height} screenshot...")
                continue

            if b"ONXSHOT:END" in line and width and height:
                if len(rows) == height:
                    ordered = [rows[index] for index in range(height)]
                    write_bmp(output, ordered, width, height)
                    print(f"Saved {output} ({width}x{height})")
                    return
                raise ValueError(
                    f"Screenshot stream ended with {height - len(rows)} incomplete rows"
                )

            match = ROW_RE.search(line)
            if match and width and height:
                row_index = int(match.group(1))
                chunk_index = int(match.group(2))
                chunk_count = (width * 2 + CHUNK_BYTES - 1) // CHUNK_BYTES
                if row_index >= height or chunk_index >= chunk_count:
                    continue
                try:
                    raw = base64.b64decode(match.group(3), validate=True)
                except ValueError:
                    continue
                expected_length = min(
                    CHUNK_BYTES, width * 2 - chunk_index * CHUNK_BYTES
                )
                if len(raw) != expected_length:
                    continue
                chunks = row_chunks.setdefault(row_index, {})
                chunks[chunk_index] = raw
                if len(chunks) == chunk_count:
                    packed_row = b"".join(chunks[index] for index in range(chunk_count))
                    if len(packed_row) == width * 2:
                        rows[row_index] = rgb565_row_to_bgr24(packed_row)
                        if len(rows) % 32 == 0:
                            print(f"Received {len(rows)}/{height} rows")

    raise TimeoutError(
        f"Timed out waiting for screenshot (received {len(rows)} rows); "
        "start capture before rebooting the screenshot-enabled firmware."
    )


def main():
    parser = argparse.ArgumentParser(
        description="Receive an ONX ST7789 screenshot and save it as BMP."
    )
    parser.add_argument("port", help="serial port, e.g. COM13")
    parser.add_argument("-o", "--output", default="onx_screenshot.bmp")
    parser.add_argument("--timeout", type=int, default=90)
    args = parser.parse_args()

    try:
        capture(args.port, args.output, args.timeout)
    except (OSError, TimeoutError, serial.SerialException, ValueError) as error:
        print(f"Screenshot capture failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
