#!/usr/bin/env python3
"""Fly one drone of a show in ArduCopter SITL and measure its tracking error.

The show file (.gdmb or .skyb) is placed in the SITL filesystem's show slot
before boot (/axio/show.bin, i.e. ./axio/show.bin in the run directory), the
show is scheduled by GPS time and authorized, and SHOW mode flies it. The
SHRF log messages (reference and EKF position, NEU metres) are exported to
CSV and summarized.

Usually driven by run_matrix.py. Alone:
  sitl_show.py --binary build/sitl/bin/arducopter
               --defaults Tools/autotest/default_params/copter.parm,axiolight-nav.parm
               --show FILE --start-xy X Y --duration SECONDS --rundir DIR
               [--instance N] [--speedup K] [--ctrl-mode M] [--param NAME=VALUE ...]
"""

import argparse
import csv
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time

from pymavlink import mavutil

ORIGIN_LAT = -35.3632621  # CMAC, SITL's usual home
ORIGIN_LNG = 149.1652374
ORIGIN_ALT = 584.0
M_PER_DEG_LAT = 111318.84502145034  # ArduPilot LATLON_TO_M * 1e7

GPS_EPOCH_UNIX = 315964800
GPS_LEAP_SECONDS = 18
GPS_WEEK = 604800


def offset(lat, lng, north, east):
    return (
        lat + north / M_PER_DEG_LAT,
        lng + east / (M_PER_DEG_LAT * math.cos(math.radians(lat))),
    )


def log(rundir, msg):
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with open(os.path.join(rundir, "harness.log"), "a") as f:
        f.write(line + "\n")


def set_param(mav, name, value, retries=10):
    for _ in range(retries):
        mav.mav.param_set_send(mav.target_system, mav.target_component, name.encode(), float(value),
                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < 2:
            m = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if m and m.param_id == name and abs(m.param_value - float(value)) < 1e-3 * max(1, abs(float(value))):
                return
    raise RuntimeError(f"could not set {name}={value}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--defaults", required=True, help="comma-separated parameter files")
    ap.add_argument("--show", required=True)
    ap.add_argument("--start-xy", nargs=2, type=float, default=[0.0, 0.0], help="show-frame NWU start of the drone")
    ap.add_argument("--rundir", required=True)
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--speedup", type=int, default=1)
    ap.add_argument("--ctrl-mode", type=int, default=1)
    ap.add_argument("--duration", type=float, required=True, help="show duration in seconds")
    ap.add_argument("--lead", type=float, default=40.0, help="seconds from scheduling to show start")
    ap.add_argument("--param", action="append", default=[])
    args = ap.parse_args()

    rundir = os.path.abspath(args.rundir)
    if os.path.exists(rundir):
        shutil.rmtree(rundir)
    os.makedirs(os.path.join(rundir, "axio"))
    # one slot for both formats; the firmware tells them apart by the magic
    shutil.copy(args.show, os.path.join(rundir, "axio", "show.bin"))

    x, y = args.start_xy
    home_lat, home_lng = offset(ORIGIN_LAT, ORIGIN_LNG, x, -y)  # NWU: east = -y
    port = 5760 + 10 * args.instance
    cmd = [
        os.path.abspath(args.binary), "--model", "+", "--speedup", str(args.speedup),
        "--defaults", args.defaults, "-I", str(args.instance),
        "--home", f"{home_lat:.9f},{home_lng:.9f},{ORIGIN_ALT},0",
    ]
    log(rundir, " ".join(cmd))
    sitl = subprocess.Popen(cmd, cwd=rundir, stdout=open(os.path.join(rundir, "sitl.out"), "w"),
                            stderr=subprocess.STDOUT, start_new_session=True)
    result = {"rundir": rundir, "show": os.path.abspath(args.show), "ctrl_mode": args.ctrl_mode}
    try:
        time.sleep(2)
        mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=250, retries=50)
        mav.wait_heartbeat(timeout=60)
        mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                         mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

        params = {
            "SHOW_ORIGIN_LAT": round(ORIGIN_LAT * 1e7),
            "SHOW_ORIGIN_LNG": round(ORIGIN_LNG * 1e7),
            "SHOW_ORIENTATION": 0,
            "SHOW_SYNC_MODE": 1,
            "SHOW_CTRL_MODE": args.ctrl_mode,
            "SHOW_POST_ACTION": 1,
            "SHOW_OPTIONS": 8,  # bit 3: log SHRF
        }
        for p in args.param:
            k, v = p.split("=")
            params[k] = float(v)
        for k, v in params.items():
            set_param(mav, k, v)

        # Wait until the EKF is happy and GPS time is known
        t0 = time.time()
        unix = None
        while time.time() - t0 < 120 / args.speedup + 60:
            m = mav.recv_match(type=["SYSTEM_TIME", "GPS_RAW_INT", "STATUSTEXT"], blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                log(rundir, f"STATUSTEXT {m.text}")
            if m.get_type() == "SYSTEM_TIME" and m.time_unix_usec > 1e15:
                unix = m.time_unix_usec / 1e6
            if m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3 and unix and time.time() - t0 > 30 / args.speedup + 5:
                break
        if unix is None:
            raise RuntimeError("no GPS time")

        # The show starts args.lead seconds from now, in GPS time of week
        m = mav.recv_match(type="SYSTEM_TIME", blocking=True, timeout=10)
        if m is None:
            raise RuntimeError("SYSTEM_TIME lost")
        unix = m.time_unix_usec / 1e6
        gps = unix - GPS_EPOCH_UNIX + GPS_LEAP_SECONDS
        start_tow = int(gps + args.lead) % GPS_WEEK
        set_param(mav, "SHOW_START_TIME", start_tow)
        set_param(mav, "SHOW_START_AUTH", 1)
        log(rundir, f"show scheduled at GPS TOW {start_tow}")

        armed_seen = False
        deadline_sim = gps + args.lead + args.duration + 90
        while True:
            m = mav.recv_match(type=["HEARTBEAT", "STATUSTEXT", "SYSTEM_TIME"], blocking=True, timeout=5)
            if m is None:
                if sitl.poll() is not None:
                    raise RuntimeError("SITL exited")
                continue
            t = m.get_type()
            if t == "STATUSTEXT":
                log(rundir, f"STATUSTEXT {m.text}")
            elif t == "HEARTBEAT" and m.get_srcSystem() == mav.target_system and m.type != mavutil.mavlink.MAV_TYPE_GCS:
                armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed and not armed_seen:
                    log(rundir, "armed")
                    armed_seen = True
                if armed_seen and not armed:
                    log(rundir, "disarmed")
                    break
            elif t == "SYSTEM_TIME" and m.time_unix_usec > 1e15:
                if m.time_unix_usec / 1e6 - GPS_EPOCH_UNIX + GPS_LEAP_SECONDS > deadline_sim:
                    log(rundir, "timeout")
                    result["timeout"] = True
                    break
        time.sleep(2)
    finally:
        os.killpg(sitl.pid, signal.SIGTERM)
        try:
            sitl.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(sitl.pid, signal.SIGKILL)

    # Export SHRF and summarize
    logs = sorted(f for f in os.listdir(os.path.join(rundir, "logs")) if f.endswith(".BIN"))
    rows = []
    for name in logs:
        dfl = mavutil.mavlink_connection(os.path.join(rundir, "logs", name))
        while True:
            m = dfl.recv_match(type="SHRF")
            if m is None:
                break
            rows.append(m.to_dict())
    if not rows:
        raise RuntimeError("no SHRF messages in the log")
    keys = ["TimeUS", "ShowUS", "Src", "FF", "PN", "PE", "PU", "VN", "VE", "VU", "AN", "AE", "AU", "N", "E", "U"]
    with open(os.path.join(rundir, "tracking.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys + ["err_h", "err_v", "err"])
        for r in rows:
            dn, de, du = r["N"] - r["PN"], r["E"] - r["PE"], r["U"] - r["PU"]
            w.writerow([r[k] for k in keys] + [math.hypot(dn, de), du, math.sqrt(dn * dn + de * de + du * du)])

    def stats(sel):
        def pct(v, q):
            v = sorted(v)
            return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]
        out = {"n": len(sel)}
        for name, fn in (
            ("err", lambda r: math.dist((r["N"], r["E"], r["U"]), (r["PN"], r["PE"], r["PU"]))),
            ("err_h", lambda r: math.hypot(r["N"] - r["PN"], r["E"] - r["PE"])),
            ("err_v", lambda r: abs(r["U"] - r["PU"])),
        ):
            v = [fn(r) for r in sel]
            out[name] = {"p50": pct(v, 0.5), "p95": pct(v, 0.95), "max": max(v)}
        return out

    result.update({
        "source": "gdmb" if rows[0]["Src"] == 1 else "skyb",
        "accel_ff": bool(rows[0]["FF"]),
        # every SHRF sample; analyze.py computes the statistics of the show window
        "performing": stats(rows),
        "show_time_s": [rows[0]["ShowUS"] / 1e6, rows[-1]["ShowUS"] / 1e6],
    })
    with open(os.path.join(rundir, "result.json"), "w") as f:
        json.dump(result, f, indent=2)
    log(rundir, json.dumps(result))


if __name__ == "__main__":
    main()
