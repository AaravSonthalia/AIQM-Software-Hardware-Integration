"""
Quick PSU diagnostic — run this before heater_step_test.py to verify
remote mode and voltage control are working correctly.

Takes ~15 seconds. Safe: sets 2V briefly then immediately zeros out.

Usage:
    python psu_diagnostic.py
    python psu_diagnostic.py --port /dev/tty.usbserial-1120
"""

import argparse
import sys
import time
from pathlib import Path

# Allow running as `python scripts/psu_diagnostic.py`.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from owon_power_supply import OWONPowerSupply, find_owon_supplies


def run_diagnostic(port: str | None) -> None:
    if port:
        resource = f"ASRL{port}::INSTR"
    else:
        supplies = find_owon_supplies()
        if not supplies:
            print("[ERROR] No OWON supply found. Specify --port explicitly.")
            return
        resource, _ = supplies[0]

    print(f"\nConnecting to {resource} ...")
    psu = OWONPowerSupply(resource)
    psu.connect()

    try:
        # --- Step 1: identify ---
        idn = psu.identify()
        print(f"  IDN : {idn}")

        # --- Step 2: set remote mode ---
        print("\n[1] Sending SYST:REM ...")
        psu.set_remote()
        time.sleep(0.3)
        errors = psu.check_errors()
        print(f"  Error queue after SYST:REM : {errors if errors else 'empty (good)'}")

        # --- Step 3: set 2 V setpoint ---
        print("\n[2] Sending VOLT 2.000 ...")
        psu.set_voltage(2.0)
        time.sleep(0.5)
        errors = psu.check_errors()
        print(f"  Error queue after VOLT     : {errors if errors else 'empty (good)'}")

        sp = psu.get_voltage_setpoint()
        print(f"  Voltage setpoint (VOLT?)   : {sp:.3f} V  ", end="")
        if abs(sp - 2.0) < 0.05:
            print("<-- PASS: setpoint accepted")
        else:
            print("<-- FAIL: setpoint was NOT accepted (still 0 V?)")

        # --- Step 4: output on ---
        print("\n[3] Sending OUTP ON ...")
        psu.output_on()
        time.sleep(1.0)
        errors = psu.check_errors()
        print(f"  Error queue after OUTP ON  : {errors if errors else 'empty (good)'}")

        state = psu.get_output_state()
        print(f"  Output state (OUTP?)       : {'ON' if state else 'OFF'}  ", end="")
        print("<-- PASS" if state else "<-- FAIL: output did not turn on")

        # --- Step 5: measure actuals ---
        v, i, p = psu.measure_all()
        print(f"\n[4] Actual measurements:")
        print(f"  V = {v:.4f} V   I = {i:.4f} A   P = {p:.4f} W")
        if v > 1.5:
            print("  <-- PASS: voltage is being applied")
        else:
            print("  <-- Note: voltage is low — is the heater connected and load present?")

        # --- Step 6: zero immediately ---
        print("\n[5] Zeroing voltage and turning output off ...")
        psu.set_voltage(0.0)
        time.sleep(0.3)
        psu.output_off()
        psu.set_local()
        print("  Done. Front-panel control restored.")

        # --- Summary ---
        print("\n=== Summary ===")
        sp_ok = abs(sp - 2.0) < 0.05
        state_ok = state
        print(f"  SYST:REM accepted  : {'YES' if not errors else 'CHECK ERRORS ABOVE'}")
        print(f"  Voltage setpoint   : {'OK' if sp_ok else 'FAIL — commands may be ignored'}")
        print(f"  Output state       : {'OK' if state_ok else 'FAIL — output did not enable'}")
        if sp_ok and state_ok:
            print("\n  All checks passed. Safe to run heater_step_test.py.")
        else:
            print("\n  One or more checks failed. Do not run step test until resolved.")

    finally:
        try:
            psu.set_voltage(0.0)
            psu.output_off()
            psu.set_local()
        except Exception:
            pass
        psu.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Quick PSU diagnostic before step test.")
    parser.add_argument("--port", metavar="PORT",
                        help="Serial port (e.g. /dev/tty.usbserial-1120). Auto-detected if omitted.")
    args = parser.parse_args()
    run_diagnostic(args.port)
