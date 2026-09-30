#!/usr/bin/env python3
"""Protocol tests: Python implementation + cross-check against the C header.

Run:  python3 tests/test_protocol.py        (from rover/)
"""
import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gui"))
import protocol as proto

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestCrc8(unittest.TestCase):
    def test_crc_vectors(self):
        self.assertEqual(proto.crc8(b""), 0x00)
        self.assertEqual(proto.crc8(bytes([0xAA, 0x01, 0x04])), 0xC6)
        # byte-wise identity with the shift-loop in protocol.h is checked in
        # TestCParity; here we lock a couple of regression vectors:
        self.assertEqual(proto.crc8(bytes([0xAA, 0x03, 0x06])), 0xE2)


class TestEncodeDecode(unittest.TestCase):
    def test_drive_roundtrip(self):
        frame = proto.make_drive(123, -45)
        self.assertEqual(frame[0], proto.SYNC)
        self.assertEqual(frame[1], proto.CMD_DRIVE)
        self.assertEqual(frame[2], 4)
        self.assertEqual(proto.parse_drive(frame[3:7]), (123, -45))

    def test_drive_clamps(self):
        self.assertEqual(proto.parse_drive(proto.make_drive(999, -999)[3:7]),
                         (255, -255))

    def test_gimbal_roundtrip(self):
        frame = proto.make_gimbal(12, 170)
        self.assertEqual(proto.parse_gimbal(frame[3:5]), (12, 170))

    def test_telem_parse(self):
        payload = bytes([0x00, 0x9B, 0x1C, 0xE8, 0x21, 0x01])
        t = proto.parse_telem(payload)
        self.assertEqual(t["dist_cm"], 155)
        self.assertEqual(t["batt_mv"], 7400)
        self.assertEqual(t["loop_ms"], 33)
        self.assertEqual(t["radio_ok"], 1)

    def test_ping_ack(self):
        self.assertEqual(proto.make_ping(), bytes([0xAA, 0x04, 0x00, 0x9B]))
        self.assertEqual(proto.make_ack()[1], proto.ACK)


class TestParser(unittest.TestCase):
    def test_stream_with_garbage(self):
        p = proto.Parser()
        good = proto.encode(proto.TELEM, bytes([0, 0, 0x1C, 0xE8, 3, 1]))
        stream = bytes([0x55, 0x00]) + good + bytes([0xAA, 0xFF, 0x99]) + good
        frames = p.feed(stream)
        self.assertEqual(len(frames), 2)
        self.assertEqual(p.frames_bad, 1)

    def test_byte_by_byte(self):
        p = proto.Parser()
        good = proto.make_drive(50, 50)
        count = 0
        for b in good:
            count += len(p.feed(bytes([b])))
        self.assertEqual(count, 1)

    def test_split_chunks(self):
        p = proto.Parser()
        good = proto.make_gimbal(90, 90)
        mid = len(good) // 2
        frames = p.feed(good[:mid]) + p.feed(good[mid:])
        self.assertEqual(len(frames), 1)

    def test_corrupt_byte_then_recover(self):
        p = proto.Parser()
        good = proto.make_drive(10, 10)
        bad = bytearray(good)
        bad[3] ^= 0xFF  # corrupt payload
        frames = p.feed(bytes(bad)) + p.feed(good)
        self.assertEqual(len(frames), 1)
        self.assertEqual(p.frames_bad, 1)
        self.assertEqual(p.frames_ok, 1)

    def test_bogus_length_resyncs(self):
        p = proto.Parser()
        # 0xAA followed by type then a bogus len of 0xFF must not hang the parser
        p.feed(bytes([0xAA, 0x03, 0xFF]))
        good = proto.make_drive(7, -7)
        frames = p.feed(good)
        self.assertEqual(len(frames), 1)


class TestHtmlSync(unittest.TestCase):
    """The phone UI lives in two places; they must stay identical."""

    def test_embedded_page_matches_file(self):
        import re
        ino = open(os.path.join(ROOT, "esp32cam", "esp32cam.ino")).read()
        m = re.search(r'R"rawliteral\((.*?)\)rawliteral"', ino, re.S)
        self.assertIsNotNone(m, "INDEX_HTML raw string not found in esp32cam.ino")
        embedded = m.group(1)
        page = open(os.path.join(ROOT, "esp32cam", "index.html")).read()
        self.assertEqual(embedded, page,
                         "esp32cam.ino embedded UI != esp32cam/index.html")


class TestCParity(unittest.TestCase):
    """Compile the C header and diff its output against the Python mirror."""

    def test_c_matches_python(self):
        exe = os.path.join(tempfile.gettempdir(), "c_parity_check")
        cc = os.environ.get("CC", "cc")
        subprocess.run(
            [cc, "-I", os.path.join(ROOT, "shared"),
             os.path.join(ROOT, "tests", "c_parity.c"), "-o", exe],
            check=True, capture_output=True)
        out = subprocess.run([exe], capture_output=True, check=True).stdout.decode()
        lines = out.strip().splitlines()

        crc_c = lines[0].split(":")[1]
        self.assertEqual(proto.crc8(bytes([0xAA, 0x01, 0x04])), int(crc_c, 16))

        drive_c = lines[1]
        drive_py = proto.make_drive(123, -45).hex()
        self.assertEqual(drive_c, drive_py, "CMD_DRIVE frame mismatch C vs Python")

        telem_c = lines[2]
        telem_py = proto.encode(proto.TELEM, bytes([0x00, 0x9B, 0x1C, 0xE8, 33, 1])).hex()
        self.assertEqual(telem_c, telem_py, "TELEM frame mismatch C vs Python")

        ok_c, bad_c, _, payload4_c = lines[3].split(":")[1:]
        self.assertEqual(int(payload4_c), 33)
        # Python parser on same stream: good + garbage + good
        p = proto.Parser()
        good = proto.encode(proto.TELEM, bytes([0x00, 0x9B, 0x1C, 0xE8, 33, 1]))
        garbage = bytes([0x55, 0x00, 0xAA, 0xFF, 0x12])
        p.feed(good)
        p.feed(garbage)
        p.feed(good)
        self.assertEqual(p.frames_ok, int(ok_c))
        self.assertEqual(p.frames_bad, int(bad_c))

        endian_c = lines[4].split(":")[1]
        self.assertEqual(endian_c, str(0x1234))


if __name__ == "__main__":
    unittest.main(verbosity=2)
