# Axiovel fork: AVCopter

This is AVCopter, the flight firmware of Axiovel's indoor drone-light shows. It
is ArduPilot Copter-4.7.0 with CollMot's Skybrush layer (SHOW mode,
`AC_DroneShowManager`, `.skyb` shows through `modules/libskybrush`, the hard and
bubble fences, pyro), ported from their CMCopter-4.6, and Axiovel's changes on
top. `AGENTS.md` is upstream ArduPilot's file (layout, style, build, tests,
parameter annotations, commit prefixes); this file is what the fork adds. It
is developed inside the AxioStack superproject; cross-repo facts live in
[AxioStack `contracts/`](https://github.com/Axiovel-tech/AxioStack/tree/main/contracts),
and the contract names below are files there.

## Branches and PRs

`AVCopter-4.7` is Axiovel's branch and the one AxioStack pins. Every Axiovel
PR targets it and lands as a merge commit. Upstream's `Subsystem: description`
commit prefix still applies; its master-branch, rebase-only, Discourse and
Discord guidance does not. `AVCopter-4.6` is no longer supported
(`show-timing.md`). What CI runs is in [docs/axiovel-ci.md](docs/axiovel-ci.md).

## What Axiovel adds

| Area | Code | Read |
|---|---|---|
| Show slot `/axio/show.bin`: one file, `.gdmb` or `.skyb` by magic, CRC required, `SHOW_SLOT_VER` = 1 | `libraries/AC_DroneShowManager/AC_DroneShowManager_Loader.cpp` | `gdm-format.md` §9, `show-artifacts.md` |
| `.gdmb` reader and SHOW-mode executor with acceleration feed-forward (`SHOW_CTRL_MODE` bits, `SHOW_ACC_FF_GAIN`, `SHOW_ACC_FF_MAX`) | `DroneShow_Gdmb.c` (plain C, no ArduPilot dependency), `AC_DroneShowManager_Gdmb.cpp`, `ArduCopter/mode_drone_show.cpp` | `gdm-format.md`; AxioStack ADR 0002 |
| Show start on the UWB cluster clock, `SHOW_SYNC_MODE` 2 (`AXIO_SHOW_CLOCK`, `AXIO_SHOW_SCHEDULE`, the status echo) | `AC_DroneShowManager_Timing.cpp`, `DroneShow_CustomPackets.h` | `show-timing.md` |
| RC show-start switch sampled at the RC loop rate | `RC_Channels::read_aux_show_start()`, `ArduCopter/Copter.cpp` | the code comment |
| Second LED output (`SHOW_LED1_*`), RGBW/GRBW strips, LED blackout before the tag cuts power (`MAV_CMD_USER_2`, param1 = 2) | `AC_DroneShowManager_Lights.cpp`, `DroneShowLED_NeoPixel_RGBW.cpp`, `AC_DroneShowManager_Commands.cpp` | |
| IMU die temperatures in the standard telemetry profile | `AC_DroneShowManager_Telemetry.cpp` | `rtls-fleet-health.md` |
| External navigation from the RTLS tag: per-axis covariance (`VISO_ALT_M_NSE`), extnav Z on the private origin, fixes up to 100 Hz, clock restart after a tag reboot | `libraries/AP_VisualOdom/`, `libraries/AP_NavEKF3/`, `libraries/AP_RTC/JitterCorrection.cpp` | `rtls-tag-avcopter.md` |
| EKF3 virtual compass yaw source (`EK3_SRC1_YAW` = 9, `EK3_SRC_VC_YAW`) | `AP_NavEKF3_MagFusion.cpp`, `AP_NavEKF/AP_NavEKF_Source.cpp` | |
| BMP581 soft reset and configuration check | `libraries/AP_Baro/AP_Baro_BMP581.cpp` | |
| AXIOLIGHT boards: REVA (board ID 1153), REVB (1177), REVB-NOBARO (REVB without a barometer, the variant the fleet flies) | `libraries/AP_HAL_ChibiOS/hwdef/AXIOLIGHT-*/`, `Tools/bootloaders/AXIOLIGHT-REV*_bl.bin` | `defaults.parm` comments |
| Firmware update from the SD card (REVB bootloader), and its SITL emulation | `Tools/AP_Bootloader/flash_from_sd.cpp`, `libraries/AP_HAL_SITL/SDCardOTA.cpp` | `Tools/AP_Bootloader/README.md` |
| SITL IMU at 2 kHz (fast-rate thread), deeper Vicon buffer | `AP_InertialSensor_SITL.h`, `libraries/SITL/SIM_Vicon.cpp` | |

## Rules a change must keep

- **Stay mergeable with ArduPilot and Skybrush.** Put fork code in the show
  manager, the AXIOLIGHT hwdefs or new files; keep edits to upstream files
  small and do not restyle or move them.
- **Parameter indices are on the fleet's EEPROM.** Never renumber or reuse
  one. Axiovel's `SHOW_*` indices differ from Skybrush's (`SHOW_LED1_*` hold
  40-45, so `SHOW_START_MSEC` is 46 here and 40 upstream); check for
  collisions when merging Skybrush, as SITL's "Bad parameter table" panic does.
- **Board ID 1177 stays** on AXIOLIGHT-REVB and REVB-NOBARO, although upstream
  gave it to CrazyF405 (`Tools/AP_Bootloader/board_types.txt`). Both boards
  share it, so only the image's board name tells them apart.
- **A bootloader change ships as a rebuilt `Tools/bootloaders/AXIOLIGHT-REVB_bl.bin`**
  in the same PR; the application embeds that file.
- **Show slot, `.gdmb` and show-clock behaviour are contracts.** A change
  updates `gdm-format.md`, `show-artifacts.md` or `show-timing.md` and the
  Axio Server side with it. The loader fails closed: a file that does not
  validate leaves no show loaded. Flight code never parses with
  `GDMB_PARSE_PROVISIONAL`. In mode 2, elapsed show time comes from
  `micros64`; after the commit (T−10 s) the schedule can only abort the
  start, and from T0 it cannot act at all.
- **`DroneShow_Gdmb.c` is written from the spec, not from the engine's
  writer**, and stays free of ArduPilot headers. Its golden vectors
  (`libraries/AC_DroneShowManager/tests/gdmb_vectors.h`) are generated from
  AxioStack `contracts/gdm-vectors/` by `tests/gen_gdmb_vectors.py`; rerun it
  when the contract's vectors change and name the AxioStack revision.
- **Extnav changes follow `rtls-tag-avcopter.md`**, which also lists the
  AXIOLIGHT defaults the tag relies on (`MAV3_OPTIONS` 2, `VISO_DELAY_MS`).

## Testing

Of these, CI runs only `VisionPosition` and `DroneShowGdmb`
([docs/axiovel-ci.md](docs/axiovel-ci.md)).
Run what covers your change and say in the PR what you ran.

- Unit tests (GTest): `./waf configure --board sitl`, then
  `./waf --targets tests/<name>` and `build/sitl/tests/<name>`: `test_gdmb`
  (reader against the golden vectors), `test_sdcard_ota`,
  `test_jitter_correction`, `test_baro_bmp581`.
- Autotests (`Tools/autotest/autotest.py build.Copter test.Copter.<name>`):
  `DroneShowGdmb` (a certified `.gdmb` flown in SHOW mode; its fixture is
  one drone of axio-e2e's `specs/shows/quartet.gdm`), `DroneShowTakeoffPadSink`,
  `ExtNavClockReset`,
  `VisionPosition` (per-axis covariance, extnav Z origin), `VirtualCompass`,
  `VirtualCompassMission` and their `RateThread` variants.
- Show-mode SITL scripts in `Tools/scripts/`: `show_schedule_sitl.py` (mode 2
  schedule without tags), `gdmb_show_sitl/check_show_files.py` (show slot),
  and the `gdmb_show_sitl/` tracking comparison (its README).
- Across the stack (server, tags, SITL): AxioStack
  `contracts/production-simulation.md` and `tooling/show-sync-gate`. These run
  `build/sitl/bin/arducopter` of the AxioStack checkout (`tooling/stack env`
  exports it as `ARDUPILOT_SITL`; `tooling/stack artifact fetch` installs the
  pinned CI build there), so a local build there replaces the pinned binary.

## Building

Use upstream's commands (`AGENTS.md` §4). What this checkout adds:

- **The repo is a submodule**: its `.git` points into AxioStack's
  `.git/modules/`. A container that mounts only this repo cannot run git, and
  waf needs git. Mount the AxioStack root at its own path.
- **Nested submodules.** By default waf runs `git submodule update --init
  --recursive` for any nested submodule that is missing or behind
  (`Tools/ardupilotwaf/git_submodule.py`). `--no-submodule-update` turns that
  off; use it once they are populated and git must stay out of the build (CI's
  AXIOLIGHT-REVB job does). A new worktree (`tooling/stack worktree ardupilot
  <branch>`) starts with them empty, and letting waf fill them is a fresh
  clone of every module; attaching them as worktrees of the primary
  checkout's stores (`.git/modules/repos/fw/ardupilot/modules/modules/<name>`
  in AxioStack) avoids that.
- **Docker** is the tested toolchain, the same image CI uses. Run as your
  UID, with `HOME` and the ccache inside the container, and build each board
  in turn (one waf lock per build tree). From this repo in AxioStack:

  ```bash
  root=$(git rev-parse --path-format=absolute --git-common-dir)
  root=${root%%/.git/modules/*}          # the AxioStack root, also from a worktree
  mkdir -p "$HOME/.ccache-ardupilot-docker"
  docker run --rm --memory=4g --cpus=4 -u "$(id -u):$(id -g)" \
      -v "$root:$root" -w "$PWD" \
      -v "$HOME/.ccache-ardupilot-docker:/tmp/ccache" \
      -e CCACHE_DIR=/tmp/ccache -e HOME=/tmp \
      ardupilot/ardupilot-dev-chibios:v0.1.3 \
      bash -c './waf configure --board AXIOLIGHT-REVB-NOBARO && ./waf copter -j4'
  ```

  `ardupilot-dev-chibios` has both the native and the ARM toolchains, so
  `--board sitl` works in it too. It lacks `intelhex`, which
  `make_intel_hex.py` needs: `pip install --user intelhex` inside it.
- **Never mix mount paths.** waf stores absolute paths in `.lock-waf_*` and
  `build/c4che/`; a build with the repo mounted elsewhere (upstream's
  `/ardupilot`) breaks every later build with `Missing configuration file`.
  Recover with `rm -rf build .lock-waf*` and reconfigure.

## Flashing AXIOLIGHT-REVB over USB DFU

For a board without a working bootloader, or the first install of the
SD-update bootloader (see `Tools/AP_Bootloader/README.md`), write the
bootloader and application together with STM32CubeProgrammer (AxioStack
`tooling/install-stm32cubeprog.sh` installs it). CI publishes the combined
image as the `axiolight-revb[-nobaro]-provisioning` artifact; to make it from
a local build:

```bash
python3 Tools/scripts/make_intel_hex.py build/AXIOLIGHT-REVB-NOBARO/bin/arducopter.bin \
    Tools/bootloaders/AXIOLIGHT-REVB_bl.bin 384
"$STM32_CLI" -c port=usb1 -w build/AXIOLIGHT-REVB-NOBARO/bin/arducopter_with_bl.hex -v
"$STM32_CLI" -c port=usb1 -g 0x08000000
```

`384` is the application offset in KB (`FLASH_RESERVE_START_KB`). If
CubeProgrammer reports readout protection, `"$STM32_CLI" -c port=usb1 -y -rdu`
removes it and erases the flash; re-enter DFU before flashing. Later
application updates go over the SD card or `./waf copter --upload`.
