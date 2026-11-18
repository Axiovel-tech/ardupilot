# ArduPilot Bootloader

This is the bootloader used for STM32 boards for ArduPilot. To build
the bootloader do this:

```bash
 ./waf configure --board BOARDNAME --bootloader
 ./waf bootloader
```

the bootloader will be in build/BOARDNAME/bin. If you have the
intelhex module installed it will build in both bin format and hex
format. Both are usually uploaded with DFU. The elf file will be in
build/BOARDNAME/AP_Bootloader for loading with gdb.

The --bootloader option tells waf to get the hardware config from
the hwdef-bl.dat file for the board. It will look in
libraries/AP_HAL_CHibiOS/hwdef/BOARDNAME/hwdef-bl.dat

The bootloader protocol is compatible with that used by the PX4
project for boards like the Pixhawk. For compatibility purposes we
maintain a list of board IDs in the board_types.txt file in this
directory.
  
the board IDs in that file match the APJ_BOARD_ID in the hwdef.dat and
hwdef-bl.dat files

The bootloader can load from USB or UARTs. The list of devices to load
from is given in the SERIAL_ORDER option in hwdef-bl.dat

## SD-card application updates

Boards with `AP_BOOTLOADER_FLASH_FROM_SD_ENABLED` accept an application image
at `/ardupilot.abin`. The bootloader uses these marker names:

- `/ardupilot-verify.abin` means verification is in progress and can restart.
- `/ardupilot-flash.abin` means flashing started and no result was recorded,
  which only a reset or power loss during the flash leaves behind.
- `/ardupilot-flashed.abin` is terminal success.
- `/ardupilot-failed.abin` is terminal rejection. A new update must replace it.

Before erasing the application the bootloader reads the whole file and checks
the ABIN MD5 and, with `AP_CHECK_FIRMWARE_ENABLED`, the image's application
descriptor: its signature, board ID, length, both CRCs and that the image fits
the application area. These are the tests `check_good_firmware()` applies to
flash, so an image that could not boot is renamed to the failed marker while
the installed application is still intact. Only the descriptor format of the
bootloader build (unsigned or signed) is accepted, and a firmware signature is
still only checked after the write. An SD read error during this pass leaves
the marker in place for the next boot, as nothing has been erased.

Every failure the bootloader sees after it starts erasing (erase, write, SD
read or the final `check_good_firmware()`) is terminal and renames the file to
the failed marker. A leftover flash marker is resumed only when the installed
application is invalid, which is what an interrupted flash leaves. If the
installed application is valid, it is never erased for a leftover flash marker:
the file is marked flashed when it is that same image, and failed otherwise.

If an SD update does not complete, the bootloader returns to the installed
application only when its descriptor, board ID, length and CRC still validate.
This lets the ground stack reconnect after a pre-erase rejection without ever
booting a partially written application. An explicit bootloader reboot with no
SD update remains in the normal USB/UART bootloader flow.

### First-time AXIOLIGHT-REVB provisioning

The checking bootloader only boots applications that contain an ArduPilot
application descriptor. Older AXIOLIGHT-REVB firmware does not contain one, so
installing this bootloader by itself would leave that application in the
bootloader. For first-time OTA provisioning, use physical recovery to flash the
`axiolight-revb-provisioning` artifact's combined
`axiolight-revb-provisioning.hex`. It installs the matching bootloader and a
descriptor-enabled application in one programmer operation. Do not provision a
legacy application with the standalone bootloader artifact. Subsequent
application updates can use the SD-card flow above.

SITL can emulate the marker transaction when it receives a reboot-to-bootloader
request. Set `AXIO_SIM_SD_OTA=1` before starting SITL. Set
`AXIO_SIM_SD_OTA_RESULT=failure` or `interrupted` for fault injection. This
emulation does not run the ChibiOS bootloader or write STM32 flash.
