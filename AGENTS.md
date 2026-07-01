# Agent Notes

## ArduPilot Build Environment

Use the shared Python virtual environment from the primary checkout, even when
working from a git worktree:

```bash
AP_ROOT=/home/singu-dev/dev/fw/repos/axiovel/ardupilot
export PATH="$AP_ROOT/.venv-build/bin:/home/singu-dev/.local/toolchains/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
```

Invoke waf with that Python explicitly from the active checkout or worktree:

```bash
"$AP_ROOT/.venv-build/bin/python" ./waf configure --board AXIOLIGHT-REVB
"$AP_ROOT/.venv-build/bin/python" ./waf copter
```

The venv must provide `python` on `PATH` because some ArduPilot helper scripts
use `#!/usr/bin/env python`. ChibiOS board builds also require the ARM embedded
toolchain path above so `arm-none-eabi-*` tools are visible.

One-time DroneCAN setup for this environment:

```bash
cd "$AP_ROOT"
. .venv-build/bin/activate
python -m pip install 'setuptools<81' -e modules/DroneCAN/pydronecan
```

Keep the editable DroneCAN install pointed at the primary checkout, not a
temporary worktree, so removing a worktree does not break later builds.

For normal ArduPilot bootloader uploads, use `./waf copter --upload` after
configuring the target board. For STM32 ROM DFU flashing, install/provide
`dfu-util` on the host and flash the generated `*_with_bl.hex` image for an
initial bootloader-inclusive load.
