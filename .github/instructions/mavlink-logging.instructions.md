---
applyTo: "libraries/GCS_MAVLink/**,libraries/AP_Logger/**,**/GCS_*.cpp,**/GCS_*.h,**/Log.cpp,**/LogStructure.h"
---

# MAVLink and logging rules

## MAVLink

- Use messages, commands and enums that exist in the MAVLink definitions under
  `modules/mavlink`. Never invent a message or a field. Changes to the
  definitions go to the MAVLink project, not into the submodule here.
- Ground stations and companion computers depend on current behaviour. A
  change in what is sent, when, or how a command is acknowledged is a
  compatibility change: say so in the PR.
- Every input from a link is untrusted data for robustness: malformed or
  out-of-range values must not crash or hang the vehicle.

## Logging

- Every log message is documented above its structure with `@LoggerMessage`,
  `@Description` and one `@Field` per field. `Tools/autotest/logger_metadata`
  builds the log documentation from these.
- Units and multipliers come from the tables in `LogStructure.h`, in base
  units. A new unit or format character is also added to
  `libraries/AP_Logger/README.md`.
- Do not change the fields, types or units of an existing message: log
  analysis tools and saved logs depend on them. Add a new field or message
  instead.
- Logging costs bandwidth and CPU. Keep the rate proportionate to what the
  data is for.
