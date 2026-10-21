#!/usr/bin/env python3
"""Fly every drone of a .gdm show in SITL under several arms, in parallel.

Each arm is NAME:FORMAT:SHOW_CTRL_MODE, where FORMAT is "gdm" (the drone's
.gdmb from the .gdm container) or "skyb" (NNN.skyb in slot order from
--skyb-dir, as written by skyc_to_skyb.py). One SITL instance flies one drone;
instances do not interact, so the drones of a show can fly at the same time.

Example (the three arms of ADR 0002 phase 1, plus .skyb with feed-forward):

    run_matrix.py --gdm fable-v2.gdm --skyb-dir skyb --out runs \\
        --arm skyb:skyb:1 --arm skyb-ff:skyb:3 --arm gdm-noff:gdm:5 --arm gdm-ff:gdm:1 \\
        --parallel 14 --speedup 2 --param SHOW_TAKEOFF_ALT=1

Then: analyze.py results.json gdm-ff=runs/gdm-ff-d001,runs/gdm-ff-d002,...
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import zipfile
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gdm", required=True, help=".gdm container of the show")
    ap.add_argument("--skyb-dir", help="directory of NNN.skyb files, for skyb arms")
    ap.add_argument("--arm", action="append", required=True, help="NAME:FORMAT:SHOW_CTRL_MODE")
    ap.add_argument("--out", required=True)
    ap.add_argument("--binary", default=os.path.join(ROOT, "build", "sitl", "bin", "arducopter"))
    ap.add_argument("--parallel", type=int, default=4)
    ap.add_argument("--speedup", type=int, default=1)
    ap.add_argument("--first-instance", type=int, default=40)
    ap.add_argument("--drone", action="append", help="only these drone ids")
    ap.add_argument("--param", action="append", default=[])
    ap.add_argument("--time-axis", action="store_true", help="passed to sitl_show.py")
    args = ap.parse_args()

    out = os.path.abspath(args.out)
    gdm_dir = os.path.join(out, "gdm")
    os.makedirs(gdm_dir, exist_ok=True)
    zipfile.ZipFile(args.gdm).extractall(gdm_dir)
    show = json.load(open(os.path.join(gdm_dir, "show.json")))
    duration = show["duration_us"] / 1e6

    jobs = []
    for spec in args.arm:
        name, fmt, ctrl = spec.split(":")
        for slot, d in enumerate(show["drones"]):
            if args.drone and d["id"] not in args.drone:
                continue
            drone = json.load(open(os.path.join(gdm_dir, "drones", f"{d['id']}.json")))
            if fmt == "gdm":
                path = os.path.join(gdm_dir, "drones", f"{d['id']}.gdmb")
            else:
                path = os.path.join(os.path.abspath(args.skyb_dir), f"{slot:03d}.skyb")
            x, y, _ = drone["start"]["p"]
            jobs.append({"name": f"{name}-{d['id']}", "show": path, "xy": (x, y), "ctrl": int(ctrl)})

    defaults = ",".join([
        os.path.join(ROOT, "Tools", "autotest", "default_params", "copter.parm"),
        os.path.join(HERE, "axiolight-nav.parm"),
    ])
    free = list(range(args.first_instance, args.first_instance + args.parallel))
    lock = threading.Lock()

    def run(job):
        with lock:
            instance = free.pop()
        try:
            cmd = [
                sys.executable, os.path.join(HERE, "sitl_show.py"), "--binary", args.binary,
                "--defaults", defaults, "--show", job["show"],
                "--start-xy", str(job["xy"][0]), str(job["xy"][1]),
                "--rundir", os.path.join(out, job["name"]), "--instance", str(instance),
                "--speedup", str(args.speedup), "--ctrl-mode", str(job["ctrl"]),
                "--duration", str(duration), "--lead", "30",
            ]
            for p in args.param:
                cmd += ["--param", p]
            if args.time_axis:
                cmd.append("--time-axis")
            r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, timeout=3600)
            ok = os.path.exists(os.path.join(out, job["name"], "result.json"))
            print(job["name"], "OK" if ok else "FAILED " + r.stderr[-300:], flush=True)
        finally:
            with lock:
                free.append(instance)

    with ThreadPoolExecutor(args.parallel) as ex:
        list(ex.map(run, jobs))


if __name__ == "__main__":
    main()
