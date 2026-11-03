#!/usr/bin/env python3
"""Tracking-error statistics over SITL runs written by sitl_show.py.

Usage: analyze.py OUT.json ARM=RUNDIR[,RUNDIR...] [ARM=...]

Window ("show"): SHRF samples whose reference is at least 0.5 m above the
ground (the lowest reference height of the run, i.e. the landing pad), from
2 s after SHOW mode started tracking the trajectory (the handover from the
auto-takeoff) to the end. Errors are EKF position minus reference, in metres.
"""

import csv
import json
import math
import sys


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


def load(rundir):
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(f"{rundir}/tracking.csv"))]
    ground = min(r["PU"] for r in rows)
    t_first = rows[0]["TimeUS"]
    return [r for r in rows if r["PU"] - ground >= 0.5 and r["TimeUS"] - t_first >= 2e6], rows


def summarize(samples):
    out = {"n": len(samples)}
    for name, fn in (
        ("err", lambda r: math.sqrt(r["err_h"] ** 2 + r["err_v"] ** 2)),
        ("err_h", lambda r: r["err_h"]),
        ("err_v", lambda r: abs(r["err_v"])),
    ):
        v = [fn(r) for r in samples]
        out[name] = {"p50": pct(v, 0.5), "p95": pct(v, 0.95), "max": max(v)}
    return out


def main():
    result = {}
    for spec in sys.argv[2:]:
        arm, dirs = spec.split("=", 1)
        pooled, per_run = [], {}
        for d in dirs.split(","):
            window, rows = load(d)
            pooled += window
            per_run[d.rstrip("/").split("/")[-1]] = {
                "show": summarize(window),
                "all_performing": summarize(rows),
                "src": "gdmb" if rows[0]["Src"] == 1 else "skyb",
                "ff": bool(rows[0]["FF"]),
            }
        result[arm] = {"pooled_show": summarize(pooled), "runs": per_run}
        s = result[arm]["pooled_show"]
        print(f"{arm:14s} n={s['n']:6d}  3D p50 {s['err']['p50']*100:5.1f} cm  p95 {s['err']['p95']*100:5.1f} cm  max {s['err']['max']*100:5.1f} cm"
              f"   | horiz p50 {s['err_h']['p50']*100:5.1f} p95 {s['err_h']['p95']*100:5.1f} max {s['err_h']['max']*100:5.1f}"
              f"   | vert p50 {s['err_v']['p50']*100:5.1f} p95 {s['err_v']['p95']*100:5.1f} max {s['err_v']['max']*100:5.1f}")
    json.dump(result, open(sys.argv[1], "w"), indent=2)


if __name__ == "__main__":
    main()
