"""
OWON SPE Power Supply Test Script

Tests basic communication, OVP/OCP settings, and safety limits.
Run this to verify your power supply is communicating correctly.

Usage:
    python test.py
    python test.py /dev/tty.usbserial-XXXX  # specify port manually
"""

import sys
import time
import pyvisa


def find_owon_port():
    """Auto-detect OWON power supply port."""
    import serial.tools.list_ports

    print("Scanning for OWON power supply...")
    rm = pyvisa.ResourceManager("@py")

    for port in serial.tools.list_ports.comports():
        resource = f"ASRL{port.device}::INSTR"
        try:
            inst = rm.open_resource(resource)
            inst.timeout = 2000
            inst.baud_rate = 115200
            inst.read_termination = "\n"
            inst.write_termination = "\n"

            idn = inst.query("*IDN?")
            inst.close()

            if "OWON" in idn.upper():
                print(f"  Found: {idn.strip()}")
                print(f"  Port:  {port.device}")
                rm.close()
                return resource
        except Exception:
            pass

    rm.close()
    return None


def run_tests(resource: str):
    """Run power supply tests."""

    print("\n" + "="*60)
    print("OWON POWER SUPPLY TEST SUITE")
    print("="*60)

    rm = pyvisa.ResourceManager("@py")
    inst = rm.open_resource(resource)
    inst.timeout = 3000
    inst.baud_rate = 115200
    inst.read_termination = "\n"
    inst.write_termination = "\n"

    results = []

    def test(name: str, passed: bool, details: str = ""):
        status = "PASS" if passed else "FAIL"
        results.append((name, passed))
        print(f"  [{status}] {name}")
        if details:
            print(f"         {details}")

    try:
        # Test 1: Identification
        print("\n[Test 1] Device Identification")
        idn = inst.query("*IDN?")
        test("Read *IDN?", "OWON" in idn.upper(), idn.strip())

        # Test 2: Read current settings
        print("\n[Test 2] Read Current Settings")
        volt = inst.query("VOLT?")
        curr = inst.query("CURR?")
        test("Read voltage setpoint", True, f"VOLT? = {volt}")
        test("Read current setpoint", True, f"CURR? = {curr}")

        # Test 3: Read measurements
        print("\n[Test 3] Read Measurements")
        meas_v = inst.query("MEAS:VOLT?")
        meas_i = inst.query("MEAS:CURR?")
        meas_p = inst.query("MEAS:POW?")
        test("Measure voltage", True, f"MEAS:VOLT? = {meas_v}")
        test("Measure current", True, f"MEAS:CURR? = {meas_i}")
        test("Measure power", True, f"MEAS:POW? = {meas_p}")

        # Test 4: Read output state
        print("\n[Test 4] Output State")
        outp = inst.query("OUTP?")
        test("Read output state", True, f"OUTP? = {outp} ({'ON' if outp.strip() == '1' else 'OFF'})")

        # Test 5: Read current OVP/OCP
        print("\n[Test 5] Protection Settings (Before)")
        ovp_before = inst.query("VOLT:LIM?")
        ocp_before = inst.query("CURR:LIM?")
        test("Read OVP", True, f"VOLT:LIM? = {ovp_before} V")
        test("Read OCP", True, f"CURR:LIM? = {ocp_before} A")

        # Test 6: Set new OVP/OCP values (safe for 24W heater)
        print("\n[Test 6] Set Protection Limits")
        print("         Setting OVP=26V, OCP=1.2A (safe margins for 24V/1A heater)")

        # Set OVP to 26V (24V + ~10% margin)
        inst.write("VOLT:LIM 26.0")
        time.sleep(0.2)

        # Set OCP to 1.2A (1A + ~20% margin)
        inst.write("CURR:LIM 1.2")
        time.sleep(0.2)

        test("Set OVP command sent", True, "VOLT:LIM 26.0")
        test("Set OCP command sent", True, "CURR:LIM 1.2")

        # Test 7: Verify new OVP/OCP values
        print("\n[Test 7] Protection Settings (After)")
        ovp_after = inst.query("VOLT:LIM?")
        ocp_after = inst.query("CURR:LIM?")

        ovp_ok = abs(float(ovp_after) - 26.0) < 0.5
        ocp_ok = abs(float(ocp_after) - 1.2) < 0.1

        test("Verify OVP changed", ovp_ok, f"VOLT:LIM? = {ovp_after} V (expected ~26.0)")
        test("Verify OCP changed", ocp_ok, f"CURR:LIM? = {ocp_after} A (expected ~1.2)")

        # Test 8: Set voltage/current (without enabling output)
        print("\n[Test 8] Set Voltage/Current (Output Remains OFF)")
        inst.write("VOLT 5.0")
        time.sleep(0.1)
        inst.write("CURR 0.5")
        time.sleep(0.1)

        volt_new = inst.query("VOLT?")
        curr_new = inst.query("CURR?")

        volt_ok = abs(float(volt_new) - 5.0) < 0.1
        curr_ok = abs(float(curr_new) - 0.5) < 0.05

        test("Set voltage to 5V", volt_ok, f"VOLT? = {volt_new}")
        test("Set current to 0.5A", curr_ok, f"CURR? = {curr_new}")

        # Verify output is still off
        outp_still_off = inst.query("OUTP?").strip() in ("0", "OFF")
        test("Output still OFF", outp_still_off, "Safety check: output not accidentally enabled")

        # Test 9: Measure all info
        print("\n[Test 9] Extended Status (MEAS:ALL:INFO?)")
        try:
            info = inst.query("MEAS:ALL:INFO?")
            test("Read MEAS:ALL:INFO?", True, info.strip())
        except Exception as e:
            test("Read MEAS:ALL:INFO?", False, str(e))

    except Exception as e:
        print(f"\n!!! ERROR: {e}")
        results.append(("Exception occurred", False))

    finally:
        # Ensure safe state
        print("\n[Cleanup] Ensuring safe state...")
        try:
            inst.write("OUTP OFF")
            print("  Output disabled")
        except:
            pass

        inst.close()
        rm.close()

    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)

    passed = sum(1 for _, p in results if p)
    total = len(results)

    print(f"\n  Passed: {passed}/{total}")

    if passed == total:
        print("\n  ✓ All tests passed!")
    else:
        print("\n  ✗ Some tests failed:")
        for name, p in results:
            if not p:
                print(f"    - {name}")

    return passed == total


def main():
    # Check for manual port argument
    if len(sys.argv) > 1:
        port = sys.argv[1]
        resource = f"ASRL{port}::INSTR"
        print(f"Using specified port: {port}")
    else:
        # Auto-detect
        resource = find_owon_port()
        if resource is None:
            print("\nERROR: No OWON power supply found!")
            print("Make sure it's connected via USB and try:")
            print("  python test.py /dev/tty.usbserial-XXXX")
            sys.exit(1)

    success = run_tests(resource)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
