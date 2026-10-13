---
applyTo: "libraries/AP_HAL_ChibiOS/hwdef/**,Tools/AP_Bootloader/board_types.txt,Tools/bootloaders/**"
---

# Board (hwdef) rules

`Tools/scripts/test_new_boards.py` and
`Tools/scripts/check_branch_conventions.py` enforce most of these in CI; run
them locally before pushing.

## A new board

- `libraries/AP_HAL_ChibiOS/hwdef/<Board>/` holds `hwdef.dat`, and
  `hwdef-bl.dat` when the board has its own bootloader.
- It has a `README.md` that references at least one image added in the same
  branch, such as a pinout. OpenDroneID (`_ODID`) variants are exempt, and
  AP_Periph boards need the README but no image.
- A new `hwdef-bl.dat` comes with its prebuilt bootloader committed at
  `Tools/bootloaders/<Board>_bl.bin`, built with
  `Tools/scripts/build_bootloaders.py <Board>`.
- `defaults.parm` must not set what belongs in `hwdef.dat`:
  `SERIALn_PROTOCOL`, `SERIALn_BAUD` and `SERIALn_OPTIONS` become
  `define DEFAULT_SERIALn_...`, `ADSB_TYPE` becomes `AP_ADSB_TYPE_DEFAULT`,
  `NTF_LED_TYPES` stays `NTF_LED_TYPES`, and `BRD_TYPE` becomes
  `BOARD_TYPE_DEFAULT`.
- A serial port's default protocol must be compiled into the board.
- New board IDs in `Tools/AP_Bootloader/board_types.txt` are between 1001 and
  7199. An `_ODID` variant's ID is its base board's ID plus 10000.
- The board builds for every target it lists. Check its flash use with the
  `waf-build` skill.

## Review points

- Template leftovers: files, comments or settings copied from another board
  that do not apply. Ask whether each is real or left over.
- Serial port order and labels match the board's README and what is printed on
  the hardware.
- Reuse common includes and drivers rather than adding board-local copies.
- See the [porting guide](https://ardupilot.org/dev/docs/porting.html).
