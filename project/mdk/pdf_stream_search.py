import re
import sys
import zlib
from pathlib import Path


patterns = [
    b"ADC", b"SPI", b"MISO", b"MOSI", b"SCK", b"CLK", b"CS",
    b"PB12", b"PB13", b"PB14", b"PB15", b"PC0", b"PA", b"PB", b"PC", b"PD", b"PE",
    b"AD", b"R_", b"PT", b"NTC", b"RES"
]


def extract_streams(data: bytes):
    for match in re.finditer(rb"stream\r?\n(.*?)\r?\nendstream", data, re.S):
        start = max(0, match.start() - 600)
        header = data[start:match.start()]
        payload = match.group(1)
        if b"FlateDecode" in header:
            try:
                payload = zlib.decompress(payload)
            except Exception:
                pass
        yield payload


def printable_windows(blob: bytes):
    # Pull out useful ASCII-ish runs from schematic text and PDF commands.
    for m in re.finditer(rb"[\x20-\x7e]{3,}", blob):
        s = m.group(0)
        if any(p.lower() in s.lower() for p in patterns):
            yield s.decode("latin1", "ignore")


for name in sys.argv[1:]:
    path = Path(name)
    data = path.read_bytes()
    print(f"==== {path} ====")
    hit_count = 0
    for idx, stream in enumerate(extract_streams(data)):
        for text in printable_windows(stream):
            hit_count += 1
            print(f"[stream {idx}] {text[:220]}")
            if hit_count >= 300:
                break
        if hit_count >= 300:
            break
    print(f"hits={hit_count}")
