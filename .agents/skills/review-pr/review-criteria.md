# ArduPilot review criteria

Used by the `review-pr` skill. Keep this file to substance; path-specific rules
belong in `.github/instructions/`.

Never judge a hunk from the diff alone. Read the enclosing function and file,
and grep for callers and users of what changed.

- **Need.** Does this solve a real problem, or hide one? Trace the failure
  mechanism and check the fix addresses the root cause at the right layer.
  Would something simpler do: an existing parameter, a different default,
  deleting code?
- **Physics and maths.** For control, estimation, navigation or any
  non-trivial maths, re-derive the result from first principles and check the
  code against your derivation, not against the PR description. Check units
  (ArduPilot still mixes centidegrees, centimetres, degrees and radians; names
  such as `_cd` and `_cm` mark the legacy units), frames (NED earth frame,
  body frame), signs, singularities and edge cases (division by zero, gimbal
  lock, low airspeed), dependence on loop rate or `dt`, and float precision
  (NaN propagation, cancellation, `is_zero()` rather than `== 0`).
- **Architecture.** Does the change follow the patterns around it: the
  frontend and backend split, singletons, `AP_Param` groups, the HAL instead of
  platform code? Does it reimplement something a library already provides
  (`AP_Math`, `Filter`, `AC_PID`)? Does the logic live in the library that owns
  the data? Watch for hidden coupling: a change that silently breaks an
  assumption elsewhere.
- **Feature guards and size.** New optional code sits behind
  `#if AP_<FEATURE>_ENABLED` with a default in the library's `*_config.h`, and
  non-trivial features get an entry in `Tools/scripts/build_options.py`. A core
  component must still compile with optional features disabled. Flash is
  scarce on many boards: question every byte on code that every board builds.
- **Correctness and reliability.** Initialisation, bounds, overflow, unsigned
  arithmetic, integer promotion, races with other threads and interrupts,
  timeouts, failure paths, and behaviour when a sensor or link disappears.
  Flag real defects, not style.
- **Compatibility.** Parameters: existing `AP_GROUPINFO` indices never change,
  and a removed or renamed parameter needs a conversion so stored settings
  survive the upgrade. Every new parameter is configuration burden: challenge
  it. Log messages: changing fields or units of an existing message breaks
  log analysis tools. MAVLink: behaviour ground stations and companion
  computers rely on. Lua scripting bindings: existing scripts must keep
  working.
- **Alternatives.** Name plausible other implementations and judge each:
  better, worse or not worth mentioning. Raise only the ones the author should
  really consider, and say why.
- **Test evidence.** Behaviour changes need an autotest or unit test where
  practical; flight-critical changes without any test, SITL run or log
  evidence are a concern.

Also note CI status, reading failure logs only when they plausibly relate to
the change, and do not repeat feedback other reviewers already gave.
