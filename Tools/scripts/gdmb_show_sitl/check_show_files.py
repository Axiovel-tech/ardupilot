#!/usr/bin/env python3
"""SITL check of the show slot, /axio/show.bin.

1. SHOW_SLOT_VER is 1, and a ground station cannot change it.
2. A .gdmb file in the slot loads; so does a .skyb file.
3. A file with an unknown magic is refused, naming the magic, and a reload
   (MAV_CMD_USER_1, param1 = 0) fails.
4. The legacy /collmot/show.skyb is never loaded: with only that file, a
   warning is sent and no show is loaded; with a show in the slot, the slot
   is loaded and the warning is still sent.
5. Removing the show (MAV_CMD_USER_1, param1 = 1) deletes the slot and the
   legacy file.

Usage: check_show_files.py --gdmb FILE --skyb FILE [--binary ...] [--instance N]
Exits non-zero on the first failed expectation.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def expect(cond, what):
    print(("ok   " if cond else "FAIL ") + what, flush=True)
    if not cond:
        sys.exit(1)


def texts(mav, seconds):
    out, t0 = [], time.time()
    while time.time() - t0 < seconds:
        m = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if m is not None:
            out.append(m.text)
    return out


def user1(mav, param1):
    """Sends MAV_CMD_USER_1 and returns (result, STATUSTEXTs sent meanwhile)."""
    mav.mav.command_int_send(mav.target_system, mav.target_component, 0,
                             mavutil.mavlink.MAV_CMD_USER_1, 0, 0, param1, 0, 0, 0, 0, 0, 0)
    t0, result, seen = time.time(), None, []
    while time.time() - t0 < 10 and result is None:
        m = mav.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
        if m is None:
            continue
        if m.get_type() == "STATUSTEXT":
            seen.append(m.text)
        elif m.command == mavutil.mavlink.MAV_CMD_USER_1:
            result = m.result
    return result, seen + texts(mav, 1)


def get_param(mav, name):
    mav.mav.param_request_read_send(mav.target_system, mav.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 5:
        m = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m is not None and m.param_id == name:
            return m.param_value
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gdmb", required=True)
    ap.add_argument("--skyb", required=True)
    ap.add_argument("--binary", default=os.path.join(ROOT, "build", "sitl", "bin", "arducopter"))
    ap.add_argument("--instance", type=int, default=40)
    args = ap.parse_args()

    rundir = tempfile.mkdtemp(prefix="show-slot-")
    slot = os.path.join(rundir, "axio", "show.bin")
    legacy = os.path.join(rundir, "collmot", "show.skyb")
    os.makedirs(os.path.dirname(slot))
    os.makedirs(os.path.dirname(legacy))
    # Boot with only the legacy file
    shutil.copy(args.skyb, legacy)

    sitl = subprocess.Popen(
        [args.binary, "--model", "+", "-I", str(args.instance),
         "--defaults", os.path.join(ROOT, "Tools", "autotest", "default_params", "copter.parm")],
        cwd=rundir, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(1.5)
        mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{5760 + 10 * args.instance}",
                                         source_system=250, retries=50)
        mav.wait_heartbeat(timeout=30)

        boot = texts(mav, 6)
        expect(any("Legacy show file" in t for t in boot), "legacy file only, at boot: warning sent")
        expect(user1(mav, 0)[0] == mavutil.mavlink.MAV_RESULT_ACCEPTED,
               "legacy file only: reload succeeds (no show is not an error)")

        expect(get_param(mav, "SHOW_SLOT_VER") == 1, "SHOW_SLOT_VER is 1")
        mav.mav.param_set_send(mav.target_system, mav.target_component, b"SHOW_SLOT_VER", 7,
                               mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        time.sleep(2)
        expect(get_param(mav, "SHOW_SLOT_VER") == 1, "SHOW_SLOT_VER stays 1 after a write")

        shutil.copy(args.gdmb, slot)
        result, seen = user1(mav, 0)
        expect(result == mavutil.mavlink.MAV_RESULT_ACCEPTED, ".gdmb in the slot: reload accepted")
        expect(any("Loaded .gdmb show" in t for t in seen), ".gdmb in the slot: loaded")
        expect(any("Legacy show file" in t for t in seen), ".gdmb in the slot: legacy warning still sent")

        result, _ = user1(mav, 1)
        expect(result == mavutil.mavlink.MAV_RESULT_ACCEPTED, "remove show: accepted")
        expect(not os.path.exists(slot) and not os.path.exists(legacy),
               "remove show: slot and legacy file deleted")

        shutil.copy(args.skyb, slot)
        result, seen = user1(mav, 0)
        expect(result == mavutil.mavlink.MAV_RESULT_ACCEPTED, ".skyb in the slot: reload accepted")
        expect(not any("rejected" in t or "Legacy" in t for t in seen), ".skyb in the slot: no error or warning")

        with open(slot, "wb") as f:
            f.write(b"PK\x03\x04 not a show")
        result, seen = user1(mav, 0)
        expect(result == mavutil.mavlink.MAV_RESULT_FAILED, "unknown magic: reload fails")
        expect(any("unknown format (magic 504b0304)" in t.lower() for t in seen), "unknown magic: named in the error")
    finally:
        os.killpg(sitl.pid, signal.SIGTERM)
        shutil.rmtree(rundir, ignore_errors=True)


if __name__ == "__main__":
    main()
