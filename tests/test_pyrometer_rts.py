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

    def test_chmbe_is_false(self):
        """Ch-MBE Modbus mode must de-assert RTS by default."""
        self.assertIs(CHALCOGENIDE_MBE.pyrometer_rts, False)

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


class WorkerWiringTests(unittest.TestCase):
    """The GUI must thread pyrometer_rts from chamber config to the driver.

    On Jul 30 2026 the standalone scripts read the probe successfully and
    then the GUI, launched seconds later on the same port, failed — because
    PyrometerWorker._create_sensor called ModbusPyrometer() with no
    arguments at all, so the driver got its own None default instead of
    OXIDE_MBE.pyrometer_rts. The console said exactly that, which is the
    only reason it took seconds rather than another afternoon.
    """

    def test_worker_accepts_and_stores_rts(self):
        from gui.workers import PyrometerWorker

        self.assertIsNone(PyrometerWorker(mode="modbus").rts)
        self.assertFalse(PyrometerWorker(mode="modbus", rts=False).rts)
        self.assertIsNotNone(PyrometerWorker(mode="modbus", rts=False).rts)

    def test_forwards_all_three_inputs_to_the_driver(self):
        """port, baudrate and rts must all reach ModbusPyrometer.

        The factory previously called ModbusPyrometer() bare, so every one
        of these was silently replaced by a driver default.
        """
        from gui.workers import PyrometerWorker

        sensor = PyrometerWorker(
            mode="modbus", port="COM9", baudrate=57600, rts=False,
        )._create_sensor()
        self.assertEqual(sensor._port, "COM9")
        self.assertEqual(sensor._baudrate, 57600)
        self.assertFalse(sensor._rts)

    def test_ombe_config_reaches_the_driver_as_false(self):
        """The exact path that failed in the Jul 30 GUI session."""
        from gui.workers import PyrometerWorker

        sensor = PyrometerWorker(
            mode="modbus", rts=OXIDE_MBE.pyrometer_rts,
        )._create_sensor()
        self.assertIs(sensor._rts, False)

    def test_chmbe_config_reaches_the_driver_as_false(self):
        """The Ch-MBE chamber default must reach the Modbus driver."""
        from gui.workers import PyrometerWorker

        sensor = PyrometerWorker(
            mode="modbus", rts=CHALCOGENIDE_MBE.pyrometer_rts,
        )._create_sensor()
        self.assertIs(sensor._rts, False)
