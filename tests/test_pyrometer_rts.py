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
    def test_default_is_none_not_false(self):
        """Unconfigured must mean "no claim", never a silent behaviour change.

        Defaulting to False would export an O-MBE measurement to every
        chamber that has not been tested, changing their serial behaviour
        without evidence. None leaves them exactly as they were.
        """
        self.assertIsNone(MBESystemConfig.pyrometer_rts)

    def test_ombe_is_false(self):
        """O-MBE is the verified case — probe answered with RTS de-asserted."""
        self.assertFalse(OXIDE_MBE.pyrometer_rts)
        self.assertIsNotNone(OXIDE_MBE.pyrometer_rts)

    def test_chmbe_stays_unset(self):
        """Ch-MBE's cabling is uncharacterised, so it must make no claim.

        This test is the guard against a well-meaning future edit copying
        O-MBE's False across "for consistency". Changing it requires
        measuring the Ch-MBE probe first, and changing this test with it.
        """
        self.assertIsNone(CHALCOGENIDE_MBE.pyrometer_rts)

    def test_setting_is_per_chamber_not_module_global(self):
        """A future edit must not collapse this into one shared constant."""
        self.assertIn("pyrometer_rts", MBESystemConfig.__dataclass_fields__)


class DriverAcceptsRtsTests(unittest.TestCase):
    def test_defaults_to_none(self):
        """Constructing the driver must not change anyone's serial state."""
        self.assertIsNone(ModbusPyrometer()._rts)

    def test_stores_explicit_value(self):
        self.assertTrue(ModbusPyrometer(rts=True)._rts)
        self.assertFalse(ModbusPyrometer(rts=False)._rts)

    def test_false_is_distinguishable_from_unset(self):
        """`if self._rts:` would treat False and None alike — it must not.

        False means "de-assert the line"; None means "do not touch it".
        Collapsing them reintroduces exactly the untested rollout this
        design exists to prevent.
        """
        self.assertIsNotNone(ModbusPyrometer(rts=False)._rts)
        self.assertIsNone(ModbusPyrometer()._rts)

    def test_constructing_opens_no_port(self):
        """Construction must stay inert — connect() owns all I/O."""
        p = ModbusPyrometer(port="COM_DOES_NOT_EXIST", rts=False)
        self.assertFalse(p.connected)


if __name__ == "__main__":
    unittest.main()
