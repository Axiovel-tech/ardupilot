# Agent Notes

## ArduPilot Build Environment

Use the shared Python virtual environment from the primary checkout, even when
working from a git worktree. Set `AP_ROOT` to that primary checkout (a worktree
of it will not have its own `.venv-build`), and `ARM_TOOLCHAIN` to the
gcc-arm-none-eabi 10-2020-q4-major `bin` directory:

```bash
AP_ROOT=/path/to/ardupilot            # primary checkout, not the worktree
ARM_TOOLCHAIN="$HOME/.local/toolchains/gcc-arm-none-eabi-10-2020-q4-major/bin"
export PATH="$AP_ROOT/.venv-build/bin:$ARM_TOOLCHAIN:$PATH"
```

Invoke waf with that Python explicitly from the active checkout or worktree:

```bash
"$AP_ROOT/.venv-build/bin/python" ./waf configure --board AXIOLIGHT-REVB
"$AP_ROOT/.venv-build/bin/python" ./waf copter
```

The venv must provide `python` on `PATH` because some ArduPilot helper scripts
use `#!/usr/bin/env python`. ChibiOS board builds also require the ARM embedded
toolchain path above so `arm-none-eabi-*` tools are visible.

If `dronecan` is missing during `waf`, install it from the primary checkout:

```bash
cd "$AP_ROOT"
. .venv-build/bin/activate
python -m pip install 'setuptools<81' -e modules/DroneCAN/pydronecan
```

For normal ArduPilot bootloader uploads, use `./waf copter --upload` after
configuring the target board.

## AXIOLIGHT-REVB STM32 DFU Flashing

Use STM32CubeProgrammer for STM32 ROM DFU flashing. Point `STM32_CLI` at your
installation; the default install puts it here:

```bash
STM32_CLI="$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
```

Build from the active worktree:

```bash
AP_ROOT=/path/to/ardupilot            # primary checkout, not the worktree
export PATH="$HOME/.local/toolchains/gcc-arm-none-eabi-10-2020-q4-major/bin:$AP_ROOT/.venv-build/bin:$PATH"

"$AP_ROOT/.venv-build/bin/python" ./waf configure --board AXIOLIGHT-REVB
GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=safe.directory GIT_CONFIG_VALUE_0='*' \
    "$AP_ROOT/.venv-build/bin/python" ./waf copter
```

Regenerate the bootloader-inclusive Intel HEX image:

```bash
"$AP_ROOT/.venv-build/bin/python" Tools/scripts/make_intel_hex.py \
    build/AXIOLIGHT-REVB/bin/arducopter.bin \
    Tools/bootloaders/AXIOLIGHT-REVB_bl.bin \
    384
```

Flash and start the DFU target:

```bash
"$STM32_CLI" -c port=usb1 \
    -w build/AXIOLIGHT-REVB/bin/arducopter_with_bl.hex \
    -v

"$STM32_CLI" -c port=usb1 -g 0x08000000
```

If CubeProgrammer reports readout protection, `"$STM32_CLI" -c port=usb1 -y
-rdu` removes read protection and erases flash. Re-enter DFU mode after that
before flashing.
