"""Offline wire fixtures and read-only validation of raw TI UART captures.

Run without arguments for fixtures. --capture PATH checks a real capture;
--require-frames 10000 enforces the requested export-measurement sample count.
This script never opens a serial port or changes an RF profile.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import math
from pathlib import Path
import struct
import unittest
import zlib

MAGIC = bytes.fromhex("0201040306050807")
HEADER = struct.Struct("<8s8I")
TLV = struct.Struct("<II")
BOX = struct.Struct("<7f2h")
STATUS = struct.Struct("<HHI8H6I")
LIMIT_FLAGS = 0x1FF | (1 << 11) | (1 << 12)


def crc64_iso(raw: bytes) -> int:
    """TI SBL: ISO 3309 polynomial, MSB first, byte swap enabled, seed zero."""
    table = []
    mask = (1 << 64) - 1
    for byte in range(256):
        crc = byte << 56
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1B if crc >> 63 else 0)) & mask
        table.append(crc)
    crc = 0
    for byte in raw:
        crc = ((crc << 8) & mask) ^ table[(crc >> 56) ^ byte]
    return crc


def validate_image(raw: bytes, expected_dss: bytes | None = None):
    # Layout: installed SDK ti/utils/sbl/src/metaheader_parser.c.
    if len(raw) < 132 or zlib.crc32(raw[:-4]) != struct.unpack_from("<I", raw, len(raw) - 4)[0]:
        raise ValueError("Image CRC32 mismatch")
    start, number, device, hi, lo, total = struct.unpack_from("<6I", raw)
    if start != 0x5254534D or number != 3 or device != 0x37 or total != len(raw) - 4:
        raise ValueError("Invalid multicore metaheader")
    header = bytearray(raw[:32 + 32 * number])
    header[12:20] = bytes(8)
    if struct.unpack_from("<I", header, len(header) - 4)[0] != 0x444E454D:
        raise ValueError("Missing metaheader end marker")
    if crc64_iso(header) != hi << 32 | lo:
        raise ValueError("Metaheader CRC64 mismatch")
    images, previous_end = [], len(header)
    for index, core in enumerate((0x35510000, 0xB5510000, 0xD5510000)):
        _, magic, offset, hi, lo, size, _, _ = struct.unpack_from("<8I", raw, 24 + 32 * index)
        if magic != core or offset < previous_end or offset % 64 or size == 0 or offset + size > total:
            raise ValueError("Invalid core ID, offset, or image size")
        payload = raw[offset:offset + size]
        if payload[:4] != b"RPRC" or crc64_iso(payload) != hi << 32 | lo:
            raise ValueError(f"RPRC/CRC64 mismatch for core {core:#x}")
        if core == 0xD5510000 and expected_dss is not None:
            if not payload.startswith(expected_dss) or any(payload[len(expected_dss):]):
                raise ValueError("Multicore image does not embed the current DSS RPRC")
        images.append({"core": f"{core:#010x}", "offset": offset, "size": size})
        previous_end = offset + size
    return {"size": len(raw), "sha256": hashlib.sha256(raw).hexdigest(),
            "crc32_and_all_crc64": "verified", "images": images}


def recommended_period_us(export_us: int) -> int:
    """20% measured margin, minimum 50 ms, rounded upward to 1 ms."""
    return ((max(50_000, (export_us * 5 + 3) // 4) + 999) // 1000) * 1000


def packets(raw: bytes):
    offset = 0
    while offset < len(raw):
        if raw[offset:offset + 8] != MAGIC:
            raise ValueError(f"Missing magic at byte {offset}")
        if len(raw) - offset < HEADER.size:
            raise ValueError("Truncated packet header")
        fields = HEADER.unpack_from(raw, offset)
        total, num_objects, num_tlvs = fields[2], fields[6], fields[7]
        if total < HEADER.size or total % 32 or offset + total > len(raw):
            raise ValueError("Invalid or truncated packet length")
        end = offset + total
        pos = offset + HEADER.size
        tlvs = {}
        for _ in range(num_tlvs):
            if pos + TLV.size > end:
                raise ValueError("Truncated TLV header")
            kind, size = TLV.unpack_from(raw, pos)
            pos += TLV.size
            if size > end - pos or kind in tlvs:
                raise ValueError("Invalid TLV length or duplicate type")
            tlvs[kind] = raw[pos:pos + size]
            pos += size
        if end - pos >= 32 or any(raw[pos:end]):
            raise ValueError("Invalid packet padding")
        yield fields[4], fields[8], num_objects, total, tlvs
        offset = end


def validate_capture(raw: bytes, baud: int = 921600, mode: str = "mobile"):
    if baud <= 0:
        raise ValueError("Baud rate must be positive")
    measurements = collections.defaultdict(dict)
    frame_counts = collections.Counter()
    limits = collections.Counter()
    for frame, sf, num_objects, size, tlvs in packets(raw):
        if 21 not in tlvs or len(tlvs[21]) != STATUS.size:
            raise ValueError("Missing TLV21 v1 status")
        st = STATUS.unpack(tlvs[21])
        version, payload_size, flags = st[:3]
        returned, processed, produced, clustered, clusters, boxes, held, ranges = st[3:11]
        processing, maximum, previous_export, period, uart, previous_frame = st[11:17]
        if version != 1 or payload_size != STATUS.size:
            raise ValueError("Unsupported status ABI")
        if not (processed <= returned <= 1536 and clustered <= produced <= 16384):
            raise ValueError("Inconsistent detection/sample counts")
        if boxes > 255 or held > boxes or clusters > 16384 or ranges > 1024:
            raise ValueError("Invalid cluster/box/range count")
        if processing > maximum:
            raise ValueError("Current processing exceeds reported maximum")
        if previous_export and previous_frame == frame:
            raise ValueError("Export timing incorrectly tagged as the current frame")
        if period == 0:
            raise ValueError("Missing configured frame period")
        expected_uart = (size * 10_000_000 + baud - 1) // baud
        if uart != expected_uart:
            raise ValueError("UART estimate does not describe the actual padded packet")
        payload = tlvs.get(20, b"")
        if len(payload) != boxes * BOX.size:
            raise ValueError("TLV20 count/length mismatch")
        if 1 not in tlvs and num_objects != boxes:
            raise ValueError("Header object count differs from TLV20")
        for rank, values in enumerate(BOX.iter_unpack(payload), 1):
            if not all(math.isfinite(v) for v in values[:7]):
                raise ValueError("Non-finite box")
            if min(values[3:6]) <= 0 or values[1] < 0:
                raise ValueError("Invalid box geometry")
            if values[8] & 255 != rank:
                raise ValueError("Duplicate or unstable frame rank encoding")
        if mode == "mobile" and held:
            raise ValueError("Mobile mode emitted held coordinates")
        if previous_export:
            if previous_frame in measurements[sf]:
                raise ValueError("Duplicate export timing sample")
            measurements[sf][previous_frame] = previous_export
        frame_counts[sf] += 1
        if flags & LIMIT_FLAGS:
            limits[sf] += 1
    if not frame_counts:
        raise ValueError("Empty capture")
    return {
        sf: {
            "frames": frame_counts[sf],
            "export_samples": len(measurements[sf]),
            "max_export_us": max(measurements[sf].values(), default=0),
            "recommended_period_us": recommended_period_us(
                max(measurements[sf].values(), default=0)),
            "frames_with_limits_or_unverified_coverage": limits[sf],
        }
        for sf in frame_counts
    }


def fixture_packet(frame=2, boxes=(), previous_export=20_000, flags=0,
                   previous_frame=1, held=0, extra=()):
    payload = b"".join(BOX.pack(*b) for b in boxes)
    entries = ([(20, payload)] if payload else []) + list(extra)
    count = len(boxes)
    size = (HEADER.size + sum(TLV.size + len(p) for _, p in entries)
            + TLV.size + STATUS.size + 31) & ~31
    st = STATUS.pack(1, STATUS.size, flags, count, count, count, count,
                     count, count, held, count, 1000, 2000, previous_export,
                     50_000, (size * 10_000_000 + 921599) // 921600,
                     previous_frame)
    entries.append((21, st))
    header = HEADER.pack(MAGIC, 0x03060200, size, 0xA1843, frame,
                         0, count, len(entries), 0)
    raw = header + b"".join(TLV.pack(k, len(p)) + p for k, p in entries)
    return raw + bytes(size - len(raw))


class WireFixtures(unittest.TestCase):
    def test_image_container_and_crc_reject_corruption(self):
        raw = bytearray(320)
        struct.pack_into("<6I", raw, 0, 0x5254534D, 3, 0x37, 0, 0, 320)
        for index, core in enumerate((0x35510000, 0xB5510000, 0xD5510000)):
            offset = 128 + index * 64
            payload = b"RPRC" + bytes(28)
            raw[offset:offset + len(payload)] = payload
            crc = crc64_iso(payload)
            struct.pack_into("<8I", raw, 24 + index * 32, 1, core, offset,
                             crc >> 32, crc & 0xFFFFFFFF, len(payload), 0, 0)
        struct.pack_into("<2I", raw, 120, 8, 0x444E454D)
        crc = crc64_iso(raw[:128])
        struct.pack_into("<2I", raw, 12, crc >> 32, crc & 0xFFFFFFFF)
        complete = raw + struct.pack("<I", zlib.crc32(raw))
        self.assertEqual(validate_image(complete)["size"], 324)
        for position in (12, 150, 323):
            damaged = bytearray(complete)
            damaged[position] ^= 1
            if position != 323:
                struct.pack_into("<I", damaged, 320, zlib.crc32(damaged[:-4]))
            with self.assertRaises(ValueError):
                validate_image(damaged)

    def test_empty_and_nonempty_frames_unknown_tlv_and_padding(self):
        box = (0.1, 2.0, 0.2, 0.5, 0.1, 0.4, -0.25, 250, 0x101)
        raw = fixture_packet(previous_export=0, previous_frame=0)
        raw += fixture_packet(frame=3, previous_frame=2, boxes=[box], extra=[(99, b"abc")])
        result = validate_capture(raw)[0]
        self.assertEqual((result["frames"], result["export_samples"]), (2, 1))
        self.assertEqual(result["recommended_period_us"], 50_000)

    def test_large_counts_are_not_truncated_to_eight_bits(self):
        raw = bytearray(fixture_packet())
        status_offset = HEADER.size + TLV.size
        st = list(STATUS.unpack_from(raw, status_offset))
        st[3:8] = [1392, 1392, 2784, 2784, 512]
        st[2] = 1 << 3
        STATUS.pack_into(raw, status_offset, *st)
        self.assertEqual(validate_capture(bytes(raw))[0]["frames"], 1)
        st[3:8] = [1536, 1536, 11904, 11904, 992]
        st[2] = (1 << 0) | (1 << 3)
        STATUS.pack_into(raw, status_offset, *st)
        self.assertEqual(validate_capture(bytes(raw))[0]["frames"], 1)

    def test_malformed_or_truncated_streams_are_rejected(self):
        valid = fixture_packet()
        padded = fixture_packet(extra=[(99, b"a")])
        for raw in (valid[:-1], b"x" + valid, valid[:HEADER.size],
                    padded[:-1] + b"x"):
            with self.subTest(length=len(raw)), self.assertRaises(ValueError):
                validate_capture(raw)

    def test_box_and_time_contracts(self):
        bad = (0, 2, 0, 1, 1, 1, float("nan"), 100, 1)
        for raw in (fixture_packet(boxes=[bad]),
                    fixture_packet(previous_frame=2),
                    fixture_packet(held=1),
                    fixture_packet(boxes=[(0, 2, 0, 1, 1, 1, 0, 100, 2)])):
            with self.assertRaises(ValueError):
                validate_capture(raw)

    def test_fixed_mode_and_measured_period_rounding(self):
        box = (0, 2, 0, 1, 1, 1, 0, 100, 1)
        raw = fixture_packet(boxes=[box], held=1, previous_export=41_001, flags=1 << 10)
        report = validate_capture(raw, mode="fixed")[0]
        self.assertEqual(report["recommended_period_us"], 52_000)
        self.assertEqual(recommended_period_us(89_584), 112_000)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path)
    parser.add_argument("--image", type=Path, help="Validate a generated multicore .bin")
    parser.add_argument("--dss", type=Path, help="Require this exact DSS RPRC in --image")
    parser.add_argument("--mode", choices=("mobile", "fixed"), default="mobile")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--require-frames", type=int, default=0)
    args = parser.parse_args()
    if args.image:
        print(validate_image(args.image.read_bytes(),
                             args.dss.read_bytes() if args.dss else None))
    elif args.capture:
        report = validate_capture(args.capture.read_bytes(), args.baud, args.mode)
        for sf, result in report.items():
            print(f"Subframe {sf}: {result}")
            if result["export_samples"] < args.require_frames:
                raise SystemExit("Insufficient completed-export samples")
    else:
        unittest.main(argv=[__file__])
