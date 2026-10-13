---
name: run-tests
description: Run ArduPilot's SITL autotests, C++ unit tests and Python tests for a change, and report exactly what ran and what it showed.
---

# Run ArduPilot tests

All changes must be tested. Pick the layers the change can actually affect,
run them, and report the commands and results as they are. Never report a
test that did not run, and never treat a test that was skipped as passed.

## SITL autotests (integration)

The main test system. Each test starts a simulated vehicle and flies a
scripted scenario. Suites are in `Tools/autotest/` (`arducopter.py`,
`arduplane.py`, `rover.py`, `ardusub.py` and others).

```sh
# list the tests in a suite
Tools/autotest/autotest.py --list-subtests test.Copter

# rebuild, then run one test
Tools/autotest/autotest.py build.Copter test.Copter.RTLYaw

# rebuild, then run the whole suite (slow)
Tools/autotest/autotest.py build.Copter test.Copter
```

Run the tests that exercise the changed code. Find them by grepping the suite
for the mode, parameter or message the change touches. When behaviour changes
and no test covers it, add one in the matching suite, following the style of
the tests around it.

## C++ unit tests (GTest)

Tests live in `libraries/<lib>/tests/` and include `<AP_gtest.h>`.

```sh
./waf configure --board sitl
./waf --targets tests/test_math     # build one test
./build/sitl/tests/test_math        # run it
./waf tests                         # build all unit tests
./waf check                         # build and run all unit tests
```

## Python tests

Tests for the tooling are in `Tools/autotest/unittest/` and `tests/`. Run them
with `pytest`, for example `pytest Tools/autotest/unittest/`.

## Parameter documentation

When parameters change, validate their metadata:

```sh
Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter --no-emit
```

## Report

For each command: what was run, on which commit, and the result. Quote the
failing output for failures. If a test could not be run here (no display, no
hardware, too slow), say so plainly so the user can run it.
