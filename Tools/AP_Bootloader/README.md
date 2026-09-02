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
- `/ardupilot-flash.abin` means flashing is in progress. A retry erases the
  application area and writes the complete image again.
- `/ardupilot-flashed.abin` is terminal success.
- `/ardupilot-failed.abin` is terminal rejection. A new update must replace it.

The bootloader checks the ABIN MD5 before erasing the application. It checks
the flashed application's board ID, length and CRC before marking success. A
transient erase, write, validation or SD read error leaves the flash marker in
place for the next boot.

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
