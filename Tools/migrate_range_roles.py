#!/usr/bin/env python3
"""Migrate a single-vehicle QGC .params export to explicit range sensor roles.

Writes a new file only. Never connects to a controller or changes firmware.
"""
import argparse
import math
from pathlib import Path


RENAMES = {
    "EKF2_RNG_CAN_MAX": ("EKF2_RNG_S_MAX", 20.0, 0.1, 100.0),
    "EKF2_RNG_CAN_HYS": ("EKF2_RNG_HYST", 2.0, 0.1, 10.0),
    "EKF2_RNG_MAV_MAX": ("EKF2_RNG_L_MAX", 200.0, 0.1, 655.35),
    "EKF2_RNG_CAN_TO": ("EKF2_RNG_S_TO", 1.0, 0.05, 10.0),
}


def device_id(value):
    """Accept unsigned IDs printed by listener and signed INT32 parameters."""
    number = int(value, 0)
    if not -(2**31) <= number < 2**32:
        raise ValueError("device ID must fit 32 bits")
    return number & 0xFFFFFFFF


def signed_id(value):
    return value if value < 2**31 else value - 2**32


def migrate(text, short_id, long_id, long_timeout_from):
    if not 0 <= short_id < 2**32 or not 0 <= long_id < 2**32:
        raise ValueError("device ID must fit 32 bits")
    if short_id == long_id:
        raise ValueError("assign at least one role; nonzero role IDs must differ")
    if long_timeout_from not in ("radar", "mavlink"):
        raise ValueError("choose the old long-range timeout explicitly: radar or mavlink")

    records = {}
    address = None
    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError("expected QGC columns: system component name value type")
        system, component, name, value, param_type = fields
        current_address = (int(system), int(component))
        if address is not None and current_address != address:
            raise ValueError("export must contain exactly one system/component")
        address = current_address
        if name in records:
            raise ValueError("duplicate parameter: " + name)
        if not math.isfinite(float(value)):
            raise ValueError("non-finite parameter: " + name)
        records[name] = (value, int(param_type))
    if address is None:
        raise ValueError("empty parameter export")

    additions = []
    notes = []

    def add(name, value, param_type):
        if name in records:
            previous, previous_type = records[name]
            if float(previous) != float(value) or previous_type != param_type:
                raise ValueError("existing new parameter conflicts with migration: " + name)
        else:
            additions.append(f"{address[0]}\t{address[1]}\t{name}\t{value}\t{param_type}")

    def copy(old, new, default, minimum, maximum):
        value, param_type = records.get(old, (str(default), 9))
        if param_type != 9 or not minimum <= float(value) <= maximum:
            raise ValueError("invalid legacy range setting: " + old)
        if old not in records:
            notes.append(f"# {old} absent: using its legacy firmware default {default}")
        add(new, value, 9)

    for old, (new, default, minimum, maximum) in RENAMES.items():
        copy(old, new, default, minimum, maximum)
    old_timeout = "EKF2_RNG_RAD_TO" if long_timeout_from == "radar" else "EKF2_RNG_MAV_TO"
    copy(old_timeout, "EKF2_RNG_L_TO", 0.3 if long_timeout_from == "radar" else 2.0, 0.05, 10.0)
    add("EKF2_RNG_S_ID", signed_id(short_id), 6)
    add("EKF2_RNG_L_ID", signed_id(long_id), 6)

    return text.rstrip() + "\n\n# Explicit range roles; import while disarmed, then reboot.\n" + "\n".join(
        [f"# Long-range timeout copied from {old_timeout}", *notes, *additions]) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--short-id", required=True, type=device_id)
    parser.add_argument("--long-id", required=True, type=device_id)
    parser.add_argument("--long-timeout-from", required=True, choices=("radar", "mavlink"))
    args = parser.parse_args()
    try:
        result = migrate(args.input.read_text(encoding="utf-8-sig"), args.short_id,
                         args.long_id, args.long_timeout_from)
        with args.output.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(result)
    except (ValueError, OSError) as error:
        parser.exit(1, f"Migration failed: {error}\n")
    print(f"Wrote {args.output}; review it, import while disarmed, and reboot.")


if __name__ == "__main__":
    main()
