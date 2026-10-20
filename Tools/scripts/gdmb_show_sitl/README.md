# .gdmb SHOW mode SITL comparison

Flies every drone of a show in ArduCopter SITL, once per *arm*, and measures
how well SHOW mode tracks the trajectory. This is the SITL comparison of
AxioStack ADR 0002 phase 1: `.skyc` (via `.skyb`) against `.gdm` without
acceleration feed-forward against `.gdm` complete.

| File | What |
|---|---|
| `run_matrix.py` | flies every drone of a `.gdm` show under each arm, in parallel SITL instances |
| `sitl_show.py` | one drone: places the show file in the show slot (`/axio/show.bin`, either format), boots SITL, schedules the show by GPS time, authorizes it, waits for landing, exports the `SHRF` log messages to `tracking.csv` |
| `analyze.py` | tracking-error statistics (p50, p95, max) per arm, pooled over drones and per drone |
| `skyc_to_skyb.py` | compiles each drone of a `.skyc` into the `.skyb` Axio Server would upload (run with Axio Server's Python environment) |
| `check_show_files.py` | SITL check of the show slot: each format loads from `/axio/show.bin`, an unknown magic is refused, the legacy `/collmot/show.skyb` is ignored with a warning, removing the show deletes both, `SHOW_SLOT_VER` is 1 |
| `axiolight-nav.parm` | AxioLight REVB navigation and show limits, applied on top of SITL's default quad |

## Arms

`SHOW_CTRL_MODE` selects the feed-forward:

| Arm | File | `SHOW_CTRL_MODE` | Feed-forward |
|---|---|---|---|
| `skyb` | `.skyb` from the engine's C¹ `.skyc` | 1 | velocity (today's production setting) |
| `skyb-ff` | same | 3 | velocity + acceleration (bit 1) |
| `gdm-noff` | `.gdmb` from the engine's C² `.gdm` | 5 | velocity (bit 2 disables acceleration) |
| `gdm-ff` | same | 1 | velocity + acceleration |

## What is measured

`SHRF` is written with every guided-mode command during the performance
(`SHOW_CTRL_RATE`, 50 Hz here). It holds the reference position, velocity
and acceleration and the EKF position, all in NEU metres from the EKF origin
and without the centimetre rounding of `Location`. The error is the EKF
position minus the reference.

`analyze.py` reports the *show window*: samples whose reference is at least
0.5 m above the landing pad, starting 2 s after SHOW mode begins tracking
the trajectory (the handover from its auto-takeoff is common to every arm).

## Reproducing

```bash
./waf configure --board sitl && ./waf copter
# .skyb arms only: compile the .skyc with Axio Server's environment
repos/sw/axio-server/.venv/bin/python Tools/scripts/gdmb_show_sitl/skyc_to_skyb.py show.skyc skyb
Tools/scripts/gdmb_show_sitl/run_matrix.py --gdm show.gdm --skyb-dir skyb --out runs \
    --arm skyb:skyb:1 --arm skyb-ff:skyb:3 --arm gdm-noff:gdm:5 --arm gdm-ff:gdm:1 \
    --parallel 14 --speedup 2 --param SHOW_TAKEOFF_ALT=1
Tools/scripts/gdmb_show_sitl/analyze.py results.json gdm-ff=runs/gdm-ff-d001,runs/gdm-ff-d002 ...
```

`sitl_show.py` needs `pymavlink`. SITL instances use ports from 6160
(`--first-instance 40`) so that they do not collide with a SITL on the
default port.
