#!/usr/bin/env python3
"""Sanity check for the ART gripper daemon.

Run this immediately after `install_daemon.sh --system` to confirm the full
EtherCAT → daemon → TCP → Python client stack is alive. It does not move the
gripper unless --motion is passed (motion includes one open/close cycle in air,
no object).

Usage:
    python3 tests/sanity_check.py                # ping + info + status only
    python3 tests/sanity_check.py --motion       # also do one air cycle
"""
import argparse
import os
import sys
import time

THIS = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(THIS), "python"))

from art_gripper_client import ArtGripperInterface  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=50053)
    ap.add_argument("--motion", action="store_true",
                    help="Do one air close/open cycle (no object)")
    args = ap.parse_args()

    print(f"[sanity] connecting to {args.host}:{args.port} ...")
    g = ArtGripperInterface(ip_address=args.host, port=args.port,
                            auto_motor_on=False)
    info = g.get_info()
    print(f"[sanity] daemon: name={info['name']!r} version={info['version']!r}")
    print(f"[sanity] metadata.max_width = {g.metadata.max_width:.3f} m")

    s = g.get_state()
    print(f"[sanity] initial state: width={s.width*1000:.0f} mm, "
          f"raw_status=0x{s.raw_status:02x}, "
          f"ready={s.is_ready}, fault={s.is_fault}")

    if not args.motion:
        print("[sanity] OK (no motion requested). Pass --motion to test gripper.")
        g.close()
        return

    print("[sanity] motor on (1 s settle) ...")
    g.motor_on()

    print("[sanity] open to 100 mm ...")
    g.goto(width=0.100, speed=0.150, force=30.0, blocking=True)
    print(f"        after open : width={g.get_state().width*1000:.0f} mm")

    print("[sanity] close to 0 mm ...")
    g.goto(width=0.000, speed=0.150, force=30.0, blocking=True)
    print(f"        after close: width={g.get_state().width*1000:.0f} mm "
          f"(expect 0 mm; finger tips touch firmware mechanical limit)")

    print("[sanity] open to 100 mm ...")
    g.goto(width=0.100, speed=0.150, force=30.0, blocking=True)
    print(f"        after open : width={g.get_state().width*1000:.0f} mm")

    g.motor_off()
    g.close()
    print("[sanity] OK — air cycle completed cleanly.")


if __name__ == "__main__":
    main()
