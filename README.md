# AIQM Software-Hardware Integration

Python tools for integrating hardware used in AI-driven MBE process control.

## What is included

- `SCPI_class.py`: Base SCPI communication wrapper (PyVISA).
- `owon_power_supply.py`: OWON SPE power supply interface.
- `owon_data_logger.py`: OWON voltage/current/power CSV logger.
- `owon_self_test.py`: OWON communication and safety test script.
- `dracal_thermocouple_reader.py`: Dracal thermocouple reader (VCP mode).
- `gui.py`: GUI entry point (work in progress).

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pyvisa pyvisa-py pyserial
```

## Quick start

OWON logger:

```bash
python owon_data_logger.py --interactive
```

Dracal thermocouple reader:

```bash
python dracal_thermocouple_reader.py --port /dev/tty.usbserial-XXXX
```

Dracal reader with CSV output:

```bash
python dracal_thermocouple_reader.py --port /dev/tty.usbserial-XXXX --interval-ms 500 --frac 2 --csv logs/thermocouple.csv
```

## Notes

- Use the exact serial port shown by your OS (for example `/dev/tty.usbserial-*` on macOS).
- OWON and Dracal tools are currently separate scripts. Control-loop integration is the next step.
