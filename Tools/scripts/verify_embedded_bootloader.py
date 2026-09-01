#!/usr/bin/env python3

import argparse
import base64
import hashlib
import json
import re
import zlib
from pathlib import Path


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify that an APJ contains the expected ROMFS bootloader")
    parser.add_argument("--apj", required=True, type=Path)
    parser.add_argument("--romfs-header", required=True, type=Path)
    parser.add_argument("--bootloader", required=True, type=Path)
    return parser.parse_args()


def embedded_bootloader(header: str) -> tuple[bytes, int]:
    entry = re.search(
        r'\{\s*"bootloader\.bin",\s*sizeof\(ap_romfs_(\d+)\),\s*(\d+),'
        r'\s*0x[0-9a-fA-F]+,\s*ap_romfs_\1\s*\}', header)
    if entry is None:
        raise ValueError("bootloader.bin is absent from the generated ROMFS")

    index = entry.group(1)
    array = re.search(
        rf'static const uint8_t ap_romfs_{index}\[\]\s*=\s*\{{([^}}]*)\}};',
        header,
        re.DOTALL,
    )
    if array is None:
        raise ValueError("bootloader.bin ROMFS data array is absent")
    compressed = bytes(int(value) for value in array.group(1).split(",") if value.strip())
    return compressed, int(entry.group(2))


def expected_bootloader(path: Path) -> bytes:
    data = path.read_bytes()
    padding = (-len(data)) % 32
    return data + bytes([0xFF]) * padding


def apj_image(path: Path) -> bytes:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("magic") != "APJFWv1":
        raise ValueError("input is not an APJFWv1 document")
    return zlib.decompress(base64.b64decode(document["image"]))


def main() -> int:
    arguments = parse_arguments()
    compressed, declared_size = embedded_bootloader(
        arguments.romfs_header.read_text(encoding="utf-8"))
    embedded = zlib.decompress(compressed, -15)
    expected = expected_bootloader(arguments.bootloader)
    if declared_size != len(expected) or embedded != expected:
        raise ValueError("embedded bootloader differs from the freshly built image")
    if compressed not in apj_image(arguments.apj):
        raise ValueError("APJ image does not contain the verified ROMFS bootloader")

    digest = hashlib.sha256(embedded).hexdigest()
    print(f"verified embedded bootloader: {len(embedded)} bytes, sha256={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
