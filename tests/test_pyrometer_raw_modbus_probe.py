#!/usr/bin/env python3
"""Tests for scripts/pyrometer_raw_modbus_probe.py.

No hardware, no pyserial. Everything here exercises frame construction,
CRC, response parsing, and classification against byte strings.

The CRC vectors are not invented: they are real frames whose checksums
were computed by other implementations, which makes them an independent
check on ours.

  01 05 00 13 00 00 3C 0F   BASF Exactus manual v5.04 p.52
                            (Modbus -> Exactus mode switch)
  01 03 00 01 00 01 D5 CA   Jacques's notebook, cell 1 comments
  01 03 00 02 00 01 25 CA   same
  01 03 00 03 00 01 74 0A   same, and the frame minimalmodbus actually
                            transmitted in the stored cell output
"""
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from scripts.pyrometer_raw_modbus_probe import (  # noqa: E402
    ALLOWED_REGISTERS,
    build_read_frame,
    classify,
    crc16_modbus,
    decode_float_pair,
    hexs,
    main,
    parse_response,
)


def frame(hex_str: str) -> bytes:
    return bytes.fromhex(hex_str.replace(" ", ""))


class Crc16Tests(unittest.TestCase):
    """CRC-16/MODBUS against frames checksummed by other implementations."""

    def _assert_frame_crc(self, full_hex: str):
        data = frame(full_hex)
        body, crc_bytes = data[:-2], data[-2:]
        expected = struct.unpack("<H", crc_bytes)[0]
        self.assertEqual(crc16_modbus(body), expected, f"CRC mismatch for {full_hex}")

    def test_basf_manual_mode_switch_frame(self):
        """BASF Exactus manual v5.04 page 52."""
        self._assert_frame_crc("01 05 00 13 00 00 3C 0F")

    def test_jacques_notebook_frames(self):
        for f in (
            "01 03 00 01 00 01 D5 CA",
            "01 03 00 02 00 01 25 CA",
            "01 03 00 03 00 01 74 0A",
        ):
            with self.subTest(frame=f):
                self._assert_frame_crc(f)

    def test_empty_payload_is_init_value(self):
        self.assertEqual(crc16_modbus(b""), 0xFFFF)


class BuildReadFrameTests(unittest.TestCase):
    def test_reproduces_jacques_transmitted_frame(self):
        """Our builder must produce the exact bytes minimalmodbus sent.

        Jacques read register 3 on slave 1, count 1 — a Eurotherm address
        pointed at a BASF probe, which is why it never returned data. The
        frame construction was still valid Modbus, so it is a good vector.
        """
        self.assertEqual(
            build_read_frame(1, 0x0003, 1),
            frame("01 03 00 03 00 01 74 0A"),
        )

    def test_frame_is_eight_bytes(self):
        self.assertEqual(len(build_read_frame(1, 0x1300, 1)), 8)

    def test_function_code_is_always_03(self):
        for addr, count in ((0x0000, 2), (0x1300, 1), (0x1305, 9)):
            self.assertEqual(build_read_frame(1, addr, count)[1], 0x03)

    def test_address_and_count_are_big_endian(self):
        f = build_read_frame(1, 0x1300, 2)
        self.assertEqual(f[2:4], b"\x13\x00")
        self.assertEqual(f[4:6], b"\x00\x02")

    def test_device_id_is_honoured(self):
        self.assertEqual(build_read_frame(7, 0x1300, 1)[0], 7)


class ClassifyTests(unittest.TestCase):
    TX = frame("01 03 13 00 00 01 82 9A")

    def test_no_response(self):
        self.assertEqual(classify(self.TX, b""), "no_response")

    def test_echo_of_sent(self):
        self.assertEqual(classify(self.TX, self.TX), "echo_of_sent")

    def test_echo_plus_trailing(self):
        rx = self.TX + frame("01 03 02 00 09 78 0F")
        self.assertEqual(classify(self.TX, rx), "echo_plus_trailing")

    def test_partial_echo(self):
        self.assertEqual(classify(self.TX, self.TX[:4]), "partial_echo")

    def test_non_echo_response(self):
        self.assertEqual(
            classify(self.TX, frame("01 03 02 00 09 78 0F")), "non_echo_response"
        )


class ParseResponseTests(unittest.TestCase):
    def _valid_reply(self, device_id: int, regs: list[int]) -> bytes:
        body = struct.pack(">BBB", device_id, 0x03, 2 * len(regs))
        body += b"".join(struct.pack(">H", r) for r in regs)
        return body + struct.pack("<H", crc16_modbus(body))

    def test_parses_valid_single_register_reply(self):
        data = self._valid_reply(1, [0x0903])
        p = parse_response(data, 1, 1)
        self.assertTrue(p["valid"])
        self.assertTrue(p["crc_ok"])
        self.assertEqual(p["registers"], [0x0903])

    def test_parses_valid_two_register_reply(self):
        data = self._valid_reply(1, [0x437C, 0x8CCC])
        p = parse_response(data, 1, 2)
        self.assertTrue(p["valid"])
        self.assertEqual(p["registers"], [0x437C, 0x8CCC])

    def test_rejects_crc_mismatch(self):
        data = bytearray(self._valid_reply(1, [0x1234]))
        data[-1] ^= 0xFF
        p = parse_response(bytes(data), 1, 1)
        self.assertFalse(p["valid"])
        self.assertEqual(p["reason"], "CRC mismatch")

    def test_rejects_wrong_device_id(self):
        p = parse_response(self._valid_reply(9, [0x1234]), 1, 1)
        self.assertFalse(p["valid"])
        self.assertIn("device id", p["reason"])

    def test_detects_modbus_exception_reply(self):
        body = struct.pack(">BBB", 1, 0x83, 0x02)
        data = body + struct.pack("<H", crc16_modbus(body))
        p = parse_response(data, 1, 1)
        self.assertFalse(p["valid"])
        self.assertIn("exception", p["reason"])

    def test_short_frame_is_described_not_raised(self):
        p = parse_response(b"\x01\x03", 1, 1)
        self.assertFalse(p["valid"])
        self.assertIn("too short", p["reason"])

    def test_empty_input_is_safe(self):
        self.assertFalse(parse_response(b"", 1, 1)["valid"])

    def test_echoed_request_does_not_parse_as_reply(self):
        """The failure at the heart of this whole investigation.

        An echoed request has function 0x03 and looks superficially like a
        reply, but its third byte is an address byte rather than a byte
        count. It must not be mistaken for device data.
        """
        echoed = build_read_frame(1, 0x1300, 1)
        p = parse_response(echoed, 1, 1)
        self.assertFalse(p["valid"])


class FloatDecodeTests(unittest.TestCase):
    def test_manual_worked_example(self):
        """BASF manual p.53: 252.55 C -> 437C8CCC, high word at low address."""
        d = decode_float_pair([0x437C, 0x8CCC])
        self.assertAlmostEqual(d["documented_high_word_first"], 252.55, places=2)

    def test_word_swapped_is_reported_and_differs(self):
        d = decode_float_pair([0x437C, 0x8CCC])
        self.assertNotAlmostEqual(
            d["documented_high_word_first"], d["word_swapped"], places=2
        )

    def test_short_input_returns_empty(self):
        self.assertEqual(decode_float_pair([0x437C]), {})


class WhitelistTests(unittest.TestCase):
    def test_no_config_or_command_registers_are_reachable(self):
        """Config + command registers must stay out of the read surface.

        0x1007/0x1008 carry device address and baud (bricking risk if ever
        written by a future edit); 0x8000 is the command register taking
        CMD_SAVE and CMD_REBOOT. This script is read-only, but the
        whitelist is what makes that auditable rather than incidental.
        """
        forbidden = {0x1007, 0x1008, 0x1011, 0x8000}
        addresses = {addr for addr, _, _ in ALLOWED_REGISTERS.values()}
        self.assertEqual(addresses & forbidden, set())

    def test_addresses_match_basf_manual_page_55(self):
        expected = {
            "REG_VER": 0x1300,
            "REG_CH1_TEMP": 0x0000,
            "REG_CH1_CURR": 0x0004,
            "REG_AMBIENT": 0x0800,
            "REG_SN0": 0x1305,
        }
        for name, addr in expected.items():
            self.assertEqual(ALLOWED_REGISTERS[name][0], addr, name)

    def test_unknown_register_is_rejected_before_opening_port(self):
        rc = main(["--registers", "REG_CMD", "--dry-run"])
        self.assertEqual(rc, 2)

    def test_empty_register_list_is_rejected(self):
        self.assertEqual(main(["--registers", ",", "--dry-run"]), 2)


class DryRunTests(unittest.TestCase):
    def test_dry_run_succeeds_without_hardware(self):
        self.assertEqual(main(["--dry-run"]), 0)

    def test_dry_run_writes_bundle_with_frames(self):
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(main(["--dry-run", "--output-dir", td]), 0)
            path = Path(td) / "raw_modbus_probe.json"
            self.assertTrue(path.exists(), "bundle not written")
            bundle = json.loads(path.read_text())
            self.assertEqual(bundle["verdict"]["state"], "dry_run")
            self.assertEqual(len(bundle["dry_run_frames"]), 2)

    def test_bundle_filename_avoids_result_json_collision(self):
        """Two sibling scripts clobbered each other's result.json on Jul 30."""
        import tempfile

        with tempfile.TemporaryDirectory() as td:
            main(["--dry-run", "--output-dir", td])
            self.assertFalse((Path(td) / "result.json").exists())


class HexFormattingTests(unittest.TestCase):
    def test_uppercase_space_separated(self):
        self.assertEqual(hexs(b"\x02\x56\x56\x03"), "02 56 56 03")

    def test_empty(self):
        self.assertEqual(hexs(b""), "")


if __name__ == "__main__":
    unittest.main()
