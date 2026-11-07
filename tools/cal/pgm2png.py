"""PGM (P5) -> grayscale PNG with stdlib only. Usage: pgm2png.py in.pgm out.png [scale-down]"""
import struct
import sys
import zlib

data = open(sys.argv[1], "rb").read()
parts = data.split(b"\n", 3)
w, h = map(int, parts[1].split())
px = parts[3]
step = int(sys.argv[3]) if len(sys.argv) > 3 else 1
rows = [b"\0" + px[y * w:(y + 1) * w][::step] for y in range(0, h, step)]
W, H = len(rows[0]) - 1, len(rows)


def chunk(tag, body):
    return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)


png = (b"\x89PNG\r\n\x1a\n"
       + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 0, 0, 0, 0))
       + chunk(b"IDAT", zlib.compress(b"".join(rows), 9))
       + chunk(b"IEND", b""))
open(sys.argv[2], "wb").write(png)
