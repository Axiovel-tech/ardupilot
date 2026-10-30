"""SITL check of the AVCopter show schedule (SHOW_SYNC_MODE 2) without tags.

Plays the tag (AXIO_SHOW_CLOCK 0x43 at 10 Hz on a simulated cluster clock) and
the ground (AXIO_SHOW_SCHEDULE 0x42), and asserts on the drone's echo
(AXIO_SHOW_SCHEDULE_STATUS, DATA16 type 0x5E) and STATUSTEXT.

usage: python Tools/scripts/show_schedule_sitl.py build/sitl/bin/arducopter <run dir> [instance]
"""

from __future__ import annotations

import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil

GCS_TO_DRONE = 0x5C
AXIO_DRONE_TO_GCS = 0x5E
SCHEDULE = 0x42
SHOW_CLOCK = 0x43

HAS_START, COMMITTED, CLOCK_VALID, SET_BY_RC = 1, 2, 4, 8


@dataclass
class Echo:
    schedule_id: int
    flags: int
    start: int
    at: float


class Rig:
    def __init__(self, binary: str, run_dir: Path, instance: int):
        self.run_dir = run_dir
        self.instance = instance
        self.port = 5760 + 10 * instance
        shutil.rmtree(run_dir, ignore_errors=True)
        run_dir.mkdir(parents=True)
        root = Path(binary).resolve().parents[3]
        defaults = run_dir / "defaults.parm"
        defaults.write_text(
            (root / "Tools/autotest/default_params/copter.parm").read_text()
            + "\nSHOW_SYNC_MODE 2\nLOG_DISARMED 1\n"
        )
        self.proc = subprocess.Popen(
            [binary, "--model", "+", "--speedup", "1", "--defaults", str(defaults),
             "-I", str(instance), "--home", "41.0,2.0,100,0"],
            cwd=run_dir, stdout=open(run_dir / "sitl.log", "w"), stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.time() + 30
        while True:
            try:
                socket.create_connection(("127.0.0.1", self.port), timeout=1).close()
                break
            except OSError:
                if time.time() > deadline:
                    raise RuntimeError("SITL did not open its GCS port")
                time.sleep(0.5)
        self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.port}", source_system=250)
        self.mav.wait_heartbeat(timeout=30)
        self.lock = threading.Lock()
        # Cluster clock = monotonic + this offset, in seconds
        self.cluster_offset = 1000.0
        self.clock_running = True
        self.echo: Echo | None = None
        self.texts: list[str] = []
        self.stop = False
        self.threads = [
            threading.Thread(target=self._clock_loop, daemon=True),
            threading.Thread(target=self._rx_loop, daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def cluster_us(self) -> int:
        return int((time.monotonic() + self.cluster_offset) * 1e6)

    def _send16(self, payload: bytes) -> None:
        data = list(payload) + [0] * (16 - len(payload))
        with self.lock:
            self.mav.mav.data16_send(GCS_TO_DRONE, len(payload), data)

    def _clock_loop(self) -> None:
        while not self.stop:
            if self.clock_running:
                self._send16(struct.pack("<BQ", SHOW_CLOCK, self.cluster_us()))
            time.sleep(0.1)

    def _rx_loop(self) -> None:
        while not self.stop:
            msg = self.mav.recv_match(type=["DATA16", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                self.texts.append(msg.text)
            elif msg.type == AXIO_DRONE_TO_GCS and msg.data[0] == SCHEDULE:
                _, sid, flags, start = struct.unpack("<BHBQ", bytes(msg.data[:12]))
                self.echo = Echo(sid, flags, start, time.monotonic())

    def schedule(self, schedule_id: int, start_us: int) -> None:
        self._send16(struct.pack("<BHQB", SCHEDULE, schedule_id, start_us, 0))

    def fresh_echo(self, after: float | None = None, timeout: float = 5.0) -> Echo:
        after = time.monotonic() if after is None else after
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            echo = self.echo
            if echo is not None and echo.at > after + 0.6:
                return echo
            time.sleep(0.05)
        raise AssertionError("no fresh echo")

    def close(self) -> None:
        self.stop = True
        for thread in self.threads:
            thread.join(2)
        self.mav.close()
        os.killpg(self.proc.pid, signal.SIGTERM)
        try:
            self.proc.wait(5)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)


def check(label: str, condition: bool, detail: str = "") -> None:
    print(("PASS " if condition else "FAIL ") + label + (f": {detail}" if detail else ""))
    if not condition:
        check.failed += 1


check.failed = 0


def main() -> int:
    binary, run_dir = sys.argv[1], Path(sys.argv[2])
    instance = int(sys.argv[3]) if len(sys.argv) > 3 else 9
    rig = Rig(binary, run_dir, instance)
    try:
        time.sleep(3)
        e = rig.fresh_echo()
        check("clock valid, no start", e.flags & CLOCK_VALID and not e.flags & HAS_START, f"{e}")

        start1 = rig.cluster_us() + 30_000_000
        rig.schedule(1, start1)
        e = rig.fresh_echo()
        check("schedule 1 echoed exactly", e.schedule_id == 1 and e.flags == HAS_START | CLOCK_VALID and e.start == start1, f"{e}")

        rig.schedule(1, start1 + 700)
        e = rig.fresh_echo()
        check("re-send of id 1 updates the value", e.schedule_id == 1 and e.start == start1 + 700, f"{e}")

        start2 = rig.cluster_us() + 40_000_000
        rig.schedule(2, start2)
        e = rig.fresh_echo()
        check("move before commit (id 2)", e.schedule_id == 2 and e.start == start2, f"{e}")

        rig.schedule(3, rig.cluster_us() + 5_000_000)
        e = rig.fresh_echo()
        check("start 5 s ahead refused", e.schedule_id == 3 and not e.flags & HAS_START, f"{e}")
        check("refusal reported", any("refused" in t for t in rig.texts), f"{rig.texts[-3:]}")
        rig.schedule(3, rig.cluster_us() + 30_000_000)
        e = rig.fresh_echo()
        check("re-send of a refused id stays refused", not e.flags & HAS_START, f"{e}")

        rig.schedule(4, 0)
        e = rig.fresh_echo()
        check("clear with no start", e.schedule_id == 4 and not e.flags & HAS_START, f"{e}")

        start5 = rig.cluster_us() + 14_000_000
        rig.schedule(5, start5)
        e = rig.fresh_echo()
        check("schedule 5 before commit", e.schedule_id == 5 and e.flags == HAS_START | CLOCK_VALID and e.start == start5, f"{e}")
        time.sleep(5)
        e = rig.fresh_echo()
        check("committed at T-10", e.schedule_id == 5 and e.flags & COMMITTED and abs(e.start - start5) < 2000, f"{e} off {e.start - start5} us")

        rig.schedule(6, start5 + 3_000_000)
        e = rig.fresh_echo()
        check("move after commit ignored", e.schedule_id == 5 and e.flags & COMMITTED and abs(e.start - start5) < 2000, f"{e}")

        rig.schedule(7, 0)
        e = rig.fresh_echo()
        check("abort after commit", e.schedule_id == 7 and not e.flags & HAS_START, f"{e}")
        check("abort reported", any("aborted" in t for t in rig.texts), f"{rig.texts[-3:]}")

        start8 = rig.cluster_us() + 40_000_000
        rig.schedule(8, start8)
        e = rig.fresh_echo()
        check("schedule 8", e.schedule_id == 8 and e.flags & HAS_START, f"{e}")
        rig.cluster_offset -= 500.0  # cluster restart: time drops by 500 s
        time.sleep(1.5)
        e = rig.fresh_echo()
        check("timebase step drops an uncommitted start", e.schedule_id == 0 and not e.flags & HAS_START and e.flags & CLOCK_VALID, f"{e}")
        start8b = rig.cluster_us() + 40_000_000
        rig.schedule(8, start8b)
        e = rig.fresh_echo()
        check("same id re-sent in the new timebase is taken", e.schedule_id == 8 and e.start == start8b, f"{e}")

        rig.clock_running = False
        time.sleep(3)
        e = rig.fresh_echo()
        check("clock invalid after a 2 s gap", not e.flags & CLOCK_VALID, f"{e}")
        start9 = rig.cluster_us() + 40_000_000
        rig.schedule(9, start9)
        e = rig.fresh_echo()
        check("new id without clock drops the start, id not taken", e.schedule_id == 8 and not e.flags & HAS_START, f"{e}")
        rig.clock_running = True
        time.sleep(1.5)
        rig.schedule(9, start9)
        e = rig.fresh_echo()
        check("repetition after the clock is back is scheduled", e.schedule_id == 9 and e.start == start9 and e.flags & CLOCK_VALID, f"{e}")

        # One sample 300 ms ahead just before the commit must not move the
        # committed start; a confirmed 30 ms step must be followed
        start10 = rig.cluster_us() + 12_500_000
        rig.schedule(10, start10)
        time.sleep(1.0)
        rig.cluster_offset += 0.3
        time.sleep(0.12)
        rig.cluster_offset -= 0.3
        time.sleep(3.0)
        e = rig.fresh_echo()
        check("300 ms-ahead outlier before the commit ignored", e.schedule_id == 10 and e.flags & COMMITTED and abs(e.start - start10) < 2000, f"{e} off {e.start - start10} us")
        rig.schedule(11, 0)
        time.sleep(1.0)

        start12 = rig.cluster_us() + 13_000_000
        rig.schedule(12, start12)
        time.sleep(1.0)
        rig.cluster_offset += 0.03  # the cluster clock steps 30 ms ahead
        time.sleep(3.5)
        e = rig.fresh_echo()
        # Frozen after the step: internal start = start12 mapped with the new
        # offset, so the echo through the same offset returns start12
        check("confirmed 30 ms step followed before the commit", e.schedule_id == 12 and e.flags & COMMITTED and abs(e.start - start12) < 2000, f"{e} off {e.start - start12} us")
    finally:
        rig.close()
    print(f"{check.failed} failed")
    return 1 if check.failed else 0


if __name__ == "__main__":
    sys.exit(main())
