#!/usr/bin/env python3
"""Mac-side unit tests for drivers/mistral_ads.py using a pyads mock.

Validates:
  - connect() / disconnect() lifecycle
  - read() output shape: 4 standard keys + all extended keys
  - Cell1 V/I mapped to v_actual / i_actual
  - Failed reads return None (not exceptions)
  - connect() raises AdsConnectionError when pyads is missing
  - connect() raises AdsConnectionError when open() fails
  - Interface parity with MistralGui (connected, hwnd properties)

Runs entirely Mac-side; no real ADS connection needed.

Usage:
    PYTHONPATH=. python scripts/test_mistral_ads.py
"""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path
from typing import Any, Optional
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.mistral_ads import (  # noqa: E402
    AdsConnectionError,
    MistralAdsClient,
)


# ---------------------------------------------------------------------------
# pyads mock — simulates the subset of the API that MistralAdsClient uses
# ---------------------------------------------------------------------------

def _make_pyads_mock(cell_T: float = 75.0, cell_V: float = 12.5, cell_I: float = 3.2,
                     pressure: float = 5.0e-9) -> types.ModuleType:
    """Build a minimal pyads mock with PLCTYPE constants and Connection."""

    mock_pyads = types.ModuleType("pyads")

    # Type constants — real pyads uses ctypes descriptors, but MistralAdsClient
    # only passes them through to read_by_name. Any sentinel works here.
    for attr in ("PLCTYPE_LREAL", "PLCTYPE_BOOL", "PLCTYPE_UINT", "PLCTYPE_INT"):
        setattr(mock_pyads, attr, attr)  # use the string name as the sentinel value

    def _make_values(cV=cell_V, cI=cell_I, cT=cell_T, P=pressure):
        """Build a name→value dict for the variables MistralAdsClient reads."""
        values = {
            "Main.ServiceMode": False,
            "PVCXProgram.ionGauge1.IGC_ModbusCtrl.IonGaugePressure": P,
            "PVCXProgram.ionGauge2.IGC_ModbusCtrl.IonGaugePressure": 1.0e3,  # atm (off)
            "PVCXProgram.ionGauge1.IGC_ModbusCtrl.PiraniPressure": 1.2e-2,
            "PVCXProgram.ionGauge2.IGC_ModbusCtrl.PiraniPressure": 2.3e-2,
            "TurboProgram.pump1.spdRPM": 60030,
            "TurboProgram.pump2.spdRPM": 24490,
            "PIDProgram.EBVM_Shutter.StatusOpen": False,
            "PIDProgram.EBVM_Shutter.StatusClosed": True,
            "PIDProgram.EBVM_FlowMeter.Flow": 1.5,
        }
        for i in range(1, 8):
            px = f"PIDProgram.Cell{i}"
            values.update({
                f"{px}_pidTDK.ActualTemperature":          cT + i,
                f"{px}_SetPoint":                          cT + i + 5,
                f"{px}_pidTDK.powerSupply.MeasuredVoltage": cV + i * 0.1,
                f"{px}_pidTDK.powerSupply.MeasuredCurrent": cI + i * 0.01,
                f"{px}_pidTDK.OutputPower":                 50.0 + i,
                f"{px}_State":                              2,
                f"{px}_Shutter.StatusOpen":                 (i == 1),
                f"{px}_Shutter.StatusClosed":               (i != 1),
            })
        return values

    VALUES = _make_values()

    class MockConnection:
        def __init__(self, netid, port):
            self._netid = netid
            self._port = port
            self._open = False

        def open(self):
            self._open = True

        def close(self):
            self._open = False

        def read_by_name(self, name: str, dtype: Any) -> Any:
            if name not in VALUES:
                raise KeyError(f"unknown symbol: {name}")
            return VALUES[name]

    mock_pyads.Connection = MockConnection
    return mock_pyads


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestMistralAdsClientLifecycle(unittest.TestCase):

    def setUp(self):
        self.mock_pyads = _make_pyads_mock()

    def test_not_connected_before_connect(self):
        client = MistralAdsClient()
        self.assertFalse(client.connected)

    def test_hwnd_always_zero(self):
        client = MistralAdsClient()
        self.assertEqual(client.hwnd, 0)

    def test_connect_sets_connected(self):
        client = MistralAdsClient()
        with patch.dict("sys.modules", {"pyads": self.mock_pyads}):
            client.connect()
        self.assertTrue(client.connected)

    def test_disconnect_clears_connected(self):
        client = MistralAdsClient()
        with patch.dict("sys.modules", {"pyads": self.mock_pyads}):
            client.connect()
            client.disconnect()
        self.assertFalse(client.connected)

    def test_connect_raises_if_pyads_missing(self):
        client = MistralAdsClient()
        # Temporarily remove pyads from sys.modules so the lazy import fails
        with patch.dict("sys.modules", {"pyads": None}):
            with self.assertRaises(AdsConnectionError) as ctx:
                client.connect()
        self.assertIn("pyads not installed", str(ctx.exception))

    def test_connect_raises_if_open_fails(self):
        bad_pyads = _make_pyads_mock()

        class FailConn:
            def __init__(self, *a): pass
            def open(self): raise RuntimeError("ADS route not found")

        bad_pyads.Connection = FailConn
        client = MistralAdsClient()
        with patch.dict("sys.modules", {"pyads": bad_pyads}):
            with self.assertRaises(AdsConnectionError) as ctx:
                client.connect()
        self.assertIn("ADS open() failed", str(ctx.exception))


class TestMistralAdsClientRead(unittest.TestCase):

    def _connected_client(self, **kw) -> MistralAdsClient:
        mock_pyads = _make_pyads_mock(**kw)
        client = MistralAdsClient()
        with patch.dict("sys.modules", {"pyads": mock_pyads}):
            client.connect()
        # Keep the mock in modules so read() can import it too
        sys.modules["pyads"] = mock_pyads
        return client

    def tearDown(self):
        sys.modules.pop("pyads", None)

    def test_standard_keys_present(self):
        client = self._connected_client()
        result = client.read()
        for key in ("v_set", "v_actual", "i_set", "i_actual"):
            self.assertIn(key, result, f"standard key missing: {key}")

    def test_cell1_mapped_to_standard_keys(self):
        # Cell1_V = 12.5 + 1*0.1 = 12.6;  Cell1_I = 3.2 + 1*0.01 = 3.21
        client = self._connected_client(cell_V=12.5, cell_I=3.2)
        result = client.read()
        self.assertAlmostEqual(result["v_actual"], 12.6, places=5)
        self.assertAlmostEqual(result["i_actual"], 3.21, places=5)

    def test_v_set_i_set_are_none(self):
        client = self._connected_client()
        result = client.read()
        self.assertIsNone(result["v_set"])
        self.assertIsNone(result["i_set"])

    def test_extended_cell_keys_present(self):
        client = self._connected_client()
        result = client.read()
        for i in range(1, 8):
            for suffix in ("T", "T_set", "V", "I", "power", "state",
                           "shutter_open", "shutter_closed"):
                key = f"cell{i}_{suffix}"
                self.assertIn(key, result, f"extended key missing: {key}")

    def test_pressure_keys_present(self):
        client = self._connected_client(pressure=5.0e-9)
        result = client.read()
        self.assertIn("ion_gauge_1_P", result)
        self.assertAlmostEqual(result["ion_gauge_1_P"], 5.0e-9, places=15)

    def test_ebvm_keys_present(self):
        client = self._connected_client()
        result = client.read()
        for key in ("ebvm_shutter_open", "ebvm_shutter_closed", "ebvm_coolant"):
            self.assertIn(key, result)

    def test_failed_read_returns_none_not_exception(self):
        # Remove one symbol from the mock so read_by_name raises KeyError
        mock_pyads = _make_pyads_mock()
        original_read = mock_pyads.Connection.read_by_name

        def patched_read(self, name, dtype):
            if name == "TurboProgram.pump1.spdRPM":
                raise RuntimeError("simulated ADS timeout")
            return original_read(self, name, dtype)

        mock_pyads.Connection.read_by_name = patched_read
        client = MistralAdsClient()
        with patch.dict("sys.modules", {"pyads": mock_pyads}):
            client.connect()
        sys.modules["pyads"] = mock_pyads

        result = client.read()  # must not raise
        self.assertIsNone(result.get("turbo1_rpm"))
        # Other keys should still be populated
        self.assertIsNotNone(result.get("v_actual"))


class TestMistralAdsWorkerIntegration(unittest.TestCase):
    """Verify MistralWorker._create_driver() returns MistralAdsClient for mode='ads'."""

    def test_worker_creates_ads_client(self):
        # Minimal PyQt stub so workers.py can be imported without a display
        try:
            from gui.workers import MistralWorker
        except Exception:
            self.skipTest("PyQt6 not available in this env")

        mock_pyads = _make_pyads_mock()
        with patch.dict("sys.modules", {"pyads": mock_pyads}):
            worker = MistralWorker(mode="ads")
            driver = worker._create_driver()

        from drivers.mistral_ads import MistralAdsClient
        self.assertIsInstance(driver, MistralAdsClient)


if __name__ == "__main__":
    print("Running MistralAdsClient unit tests...\n")
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for cls in (
        TestMistralAdsClientLifecycle,
        TestMistralAdsClientRead,
        TestMistralAdsWorkerIntegration,
    ):
        suite.addTests(loader.loadTestsFromTestCase(cls))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
