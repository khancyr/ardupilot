---
applyTo: "ArduCopter/**,ArduPlane/**,ArduSub/**,Rover/**,AntennaTracker/**,Blimp/**,libraries/**,Tools/AP_Periph/**"
---

# Parameter rules

Parameters are documented inline with `@` annotations above their
`AP_GROUPINFO` entries:

```cpp
// @Param: ENABLE
// @DisplayName: Terrain data enable
// @Description: enable terrain data. This enables the vehicle storing
//   a database of terrain data on the SD card.
// @Values: 0:Disable,1:Enable
// @User: Advanced
AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Terrain, enable, 1, AP_PARAM_FLAG_ENABLE),
```

## Annotations

- `@Param:` short name
- `@DisplayName:` human-readable name
- `@Description:` what it does, not too long
- `@Values:` `value:label` pairs, comma-separated
- `@Bitmask:` `bit:label` pairs for bitmask parameters
- `@Range:` `min max`
- `@Units:` unit string (`m`, `Hz`, `deg`, `s`...)
- `@Increment:` UI step size
- `@User:` `Standard` or `Advanced`
- `@RebootRequired:` `True` when a change needs a reboot
- `@Vehicles:` for vehicle-specific parameters

Every new or changed parameter carries all relevant annotations. Validate them
with `Tools/autotest/param_metadata/param_parse.py --vehicle <Vehicle> --no-emit`.

## Rules

- A parameter's full name is at most 16 characters.
- Never change an existing `AP_GROUPINFO` index. Indices identify values in
  users' stored configurations. A new parameter takes an unused index; a
  removed one leaves its index unused for good.
- When a released parameter is renamed, moved or rescaled, convert the stored
  value (the `AP_Param::convert_old_parameters` family) so tuned vehicles keep
  their settings.
- Every new parameter is configuration burden for users. Prefer a sensible
  default or an existing parameter when one does the job.
