#!/usr/bin/env python3
"""SITL check of which show file SHOW mode loads.

1. /axio/show.gdmb and /collmot/show.skyb both present at boot: neither is
   loaded, a STATUSTEXT says so, and a reload (MAV_CMD_USER_1, param1 = 0)
   fails.
2. Remove the show (MAV_CMD_USER_1, param1 = 1): accepted, and both files are
   gone.
3. Only the .gdmb file, then only the .skyb file: each reloads successfully.

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
    mav.mav.command_int_send(mav.target_system, mav.target_component, 0,
                             mavutil.mavlink.MAV_CMD_USER_1, 0, 0, param1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 10:
        a = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if a is not None and a.command == mavutil.mavlink.MAV_CMD_USER_1:
            return a.result
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gdmb", required=True)
    ap.add_argument("--skyb", required=True)
    ap.add_argument("--binary", default=os.path.join(ROOT, "build", "sitl", "bin", "arducopter"))
    ap.add_argument("--instance", type=int, default=40)
    args = ap.parse_args()

    rundir = tempfile.mkdtemp(prefix="show-files-")
    gdmb = os.path.join(rundir, "axio", "show.gdmb")
    skyb = os.path.join(rundir, "collmot", "show.skyb")
    os.makedirs(os.path.dirname(gdmb))
    os.makedirs(os.path.dirname(skyb))
    shutil.copy(args.gdmb, gdmb)
    shutil.copy(args.skyb, skyb)

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
        expect(any("Two show files present" in t for t in boot),
               "both files at boot: 'Two show files present' reported")
        expect(not any("Loaded .gdmb show" in t for t in boot), "both files at boot: .gdmb not loaded")
        expect(user1(mav, 0) == mavutil.mavlink.MAV_RESULT_FAILED, "both files: reload fails")

        expect(user1(mav, 1) == mavutil.mavlink.MAV_RESULT_ACCEPTED, "remove show: accepted")
        expect(not os.path.exists(gdmb) and not os.path.exists(skyb), "remove show: both files deleted")

        shutil.copy(args.gdmb, gdmb)
        expect(user1(mav, 0) == mavutil.mavlink.MAV_RESULT_ACCEPTED, ".gdmb alone: reload accepted")
        expect(any("Loaded .gdmb show" in t for t in texts(mav, 2)), ".gdmb alone: loaded")

        expect(user1(mav, 1) == mavutil.mavlink.MAV_RESULT_ACCEPTED, "remove show again: accepted")
        shutil.copy(args.skyb, skyb)
        expect(user1(mav, 0) == mavutil.mavlink.MAV_RESULT_ACCEPTED, ".skyb alone: reload accepted")
        expect(os.path.exists(skyb) and not os.path.exists(gdmb), ".skyb alone: only the .skyb file present")
    finally:
        os.killpg(sitl.pid, signal.SIGTERM)
        shutil.rmtree(rundir, ignore_errors=True)


if __name__ == "__main__":
    main()
