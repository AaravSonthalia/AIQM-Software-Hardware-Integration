#!/usr/bin/env python3
"""Tests for the pyrometer RTS control-line setting.

Root cause of every silent/echo pyrometer verdict recorded Jul 28 and
Jul 30 2026. pyserial asserts RTS by default; on Bulbasaur's IFD-5 (RS232
position) + Prolific PL2303GS path an asserted RTS loops the link back, so
every request returns byte-for-byte and the probe is never reached.

Measured on COM4, Jul 30 2026, request 01 03 13 00 00 01 80 8E:

    RTS=True,  DTR=True   -> 01 03 13 00 00 01 80 8E   (loopback)
    RTS=True,  DTR=False  -> 01 03 13 00 00 01 80 8E   (loopback)
    RTS=False, DTR=True   -> 01 03 02 09 03 FE 15      (version 9.3)
    RTS=False, DTR=False  -> 01 03 02 09 03 FE 15      (version 9.3)

RTS alone determines the outcome; DTR is irrelevant and is deliberately
left unconfigured.

No hardware: these assert the configuration surface and that the driver
accepts and stores the setting.
"""
import unittest

from drivers.config import CHALCOGENIDE_MBE, OXIDE_MBE, MBESystemConfig
from drivers.pyrometer import ModbusPyrometer


class ConfigSurfaceTests(unittest.TestCase):
    def test_field_exists_with_safe_default(self):
        """Default must be the working value, not pyserial's."""
        self.assertFalse(MBESystemConfig.pyrometer_rts)

    def test_ombe_is_false(self):
        """O-MBE is the verified case — probe answered with RTS de-asserted."""
        self.assertFalse(OXIDE_MBE.pyrometer_rts)

    def test_chmbe_has_an_explicit_value(self):
        """Ch-MBE is unverified but must still be explicit, not implicit.

        Its cabling has never been characterised; the point of asserting
        this is that the value is a decision someone can find and revisit,
        rather than an inherited default nobody noticed.
        """
        self.assertIsInstance(CHALCOGENIDE_MBE.pyrometer_rts, bool)

    def test_setting_is_per_chamber_not_module_global(self):
        """A future edit must not collapse this into one shared constant."""
        self.assertIn("pyrometer_rts", MBESystemConfig.__dataclass_fields__)


class DriverAcceptsRtsTests(unittest.TestCase):
    def test_defaults_to_deasserted(self):
        self.assertFalse(ModbusPyrometer()._rts)

    def test_stores_explicit_value(self):
        self.assertTrue(ModbusPyrometer(rts=True)._rts)
        self.assertFalse(ModbusPyrometer(rts=False)._rts)

    def test_constructing_opens_no_port(self):
        """Construction must stay inert — connect() owns all I/O."""
        p = ModbusPyrometer(port="COM_DOES_NOT_EXIST", rts=False)
        self.assertFalse(p.connected)


if __name__ == "__main__":
    unittest.main()
