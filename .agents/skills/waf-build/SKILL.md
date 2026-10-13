---
name: waf-build
description: Build ArduPilot for SITL or a hardware board with waf in the current checkout, without flashing anything.
---

# Build ArduPilot

## Inputs

- A board, for example `sitl`, `CubeOrange` or `MatekH743`. Default to `sitl`
  for generic features and bug fixes; use the specific board for hardware
  ports. Validate the name with `./waf list_boards`.
- A target: a vehicle (`copter`, `heli`, `plane`, `rover`, `sub`,
  `antennatracker`, `blimp`), `AP_Periph`, `bootloader`, or a single program
  such as `--targets tests/test_math`.

Build the current checkout as it is. Do not create a worktree or switch
branches to build.

## Prepare

1. Never run `waf` with `sudo`. Always call `./waf` from the repository root.
   A fresh checkout is cloned with its submodules:
   `git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git`.
2. If the toolchain is missing (`arm-none-eabi-gcc` for ChibiOS boards), point
   the user at the matching `Tools/environment_install/install-prereqs-*`
   script rather than installing it yourself.
3. waf updates submodules itself. Do not overwrite a submodule the user has
   modified locally.

## Build

```sh
./waf configure --board <board>
./waf <target> 2>&1 | tail -n 40
```

Add `--debug` to `configure` when the build is for a debugger. Use
`./waf clean` to clean the current board and `./waf distclean` to clean
everything; after changing the board's hwdef or switching toolchains, clean
first so stale objects are not linked.

Preserve the exit status: when piping through `tail` or `tee`, run
`set -o pipefail` first. Never treat an existing artifact as proof that a build
succeeded.

## Size impact

ArduPilot runs on boards with little flash. For changes to flight code, compare
against `master` on a flash-constrained board:

```sh
Tools/scripts/size_compare_branches.py --branch=<branch> --vehicle=copter --board=<board>
```

## Report

Artifacts are in `build/<board>/bin/`. Report the board, target, commit,
whether the tree was dirty, and the artifact path. Building is not flashing:
never upload to hardware without a separate request from the user.
