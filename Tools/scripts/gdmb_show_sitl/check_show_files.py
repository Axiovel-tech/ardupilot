#!/usr/bin/env python3
"""SITL check of the show slot, /axio/show.bin.

1. SHOW_SLOT_VER is 1, and a ground station cannot change it.
2. A .gdmb file in the slot loads, and "Loaded .gdmb show" is announced;
   so does a .skyb file.
3. A valid CRC is required for both formats: a .gdmb or a .skyb with a
   corrupted byte is refused, and so is a .skyb whose header declares no CRC.
4. A file with an unknown magic is refused, naming the magic.
5. /collmot/show.skyb plays no part: with only that file nothing is loaded
   and nothing is said, and removing the show leaves it alone.
6. Removing the show (MAV_CMD_USER_1, param1 = 1) deletes the slot.

Every refused file also makes the reload (MAV_CMD_USER_1, param1 = 0) fail.

Usage: check_show_files.py --gdmb FILE --skyb FILE [--binary ...] [--instance N]
The .skyb must be version 2 with a CRC, as Axio Server writes it.
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


def corrupt(data, offset):
    b = bytearray(data)
    b[offset] ^= 0x01
    return bytes(b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gdmb", required=True)
    ap.add_argument("--skyb", required=True)
    ap.add_argument("--binary", default=os.path.join(ROOT, "build", "sitl", "bin", "arducopter"))
    ap.add_argument("--instance", type=int, default=40)
    args = ap.parse_args()

    gdmb = open(args.gdmb, "rb").read()
    skyb = open(args.skyb, "rb").read()
    if skyb[:4] != b"skyb" or skyb[4] != 2 or not skyb[5] & 1:
        sys.exit("--skyb must be a version 2 .skyb with a CRC")
    # The same .skyb with the CRC feature cleared and the CRC field removed
    skyb_no_crc = skyb[:5] + bytes([skyb[5] & ~1]) + skyb[10:]

    rundir = tempfile.mkdtemp(prefix="show-slot-")
    slot = os.path.join(rundir, "axio", "show.bin")
    legacy = os.path.join(rundir, "collmot", "show.skyb")
    os.makedirs(os.path.dirname(slot))
    os.makedirs(os.path.dirname(legacy))
    # Boot with only a file at the old .skyb location
    shutil.copy(args.skyb, legacy)

    def put(data):
        with open(slot, "wb") as f:
            f.write(data)

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
        expect(not any("show" in t.lower() for t in boot),
               "only /collmot/show.skyb at boot: nothing loaded, nothing said")

        expect(get_param(mav, "SHOW_SLOT_VER") == 1, "SHOW_SLOT_VER is 1")
        mav.mav.param_set_send(mav.target_system, mav.target_component, b"SHOW_SLOT_VER", 7,
                               mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        time.sleep(2)
        expect(get_param(mav, "SHOW_SLOT_VER") == 1, "SHOW_SLOT_VER stays 1 after a write")

        put(gdmb)
        result, seen = user1(mav, 0)
        expect(result == mavutil.mavlink.MAV_RESULT_ACCEPTED, ".gdmb in the slot: reload accepted")
        expect(any("Loaded .gdmb show" in t for t in seen), ".gdmb in the slot: loaded and announced")

        result, _ = user1(mav, 1)
        expect(result == mavutil.mavlink.MAV_RESULT_ACCEPTED, "remove show: accepted")
        expect(not os.path.exists(slot), "remove show: slot deleted")
        expect(os.path.exists(legacy), "remove show: /collmot/show.skyb left alone (the server deletes it)")

        put(skyb)
        result, seen = user1(mav, 0)
        expect(result == mavutil.mavlink.MAV_RESULT_ACCEPTED, ".skyb in the slot: reload accepted")
        expect(not any("rejected" in t for t in seen), ".skyb in the slot: no error")

        for name, data, message in (
            (".gdmb with a corrupted byte", corrupt(gdmb, len(gdmb) // 2), "E_CRC"),
            (".skyb with a corrupted byte", corrupt(skyb, len(skyb) // 2), ".skyb CRC mismatch"),
            (".skyb without CRC", skyb_no_crc, ".skyb without CRC"),
            ("unknown magic", b"PK\x03\x04 not a show", "unknown format (magic 504b0304)"),
        ):
            put(data)
            result, seen = user1(mav, 0)
            expect(result == mavutil.mavlink.MAV_RESULT_FAILED, f"{name}: reload fails")
            expect(any(message.lower() in t.lower() for t in seen), f"{name}: '{message}' reported")
    finally:
        os.killpg(sitl.pid, signal.SIGTERM)
        shutil.rmtree(rundir, ignore_errors=True)


if __name__ == "__main__":
    main()
