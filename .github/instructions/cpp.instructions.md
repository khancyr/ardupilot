---
applyTo: "**/*.cpp,**/*.h,**/*.c,**/*.hpp"
---

# C++ coding rules

Does not apply to `modules/` (submodules) or vendored code such as
`libraries/AP_GyroFFT/CMSIS_5/`.

## Style

| Rule | Convention |
| --- | --- |
| Indentation | 4 spaces, no tabs |
| Braces | Linux/K&R, opening brace on the same line |
| Line endings | LF only |
| Header guards | `#pragma once` |
| Single-line blocks | always braced |

astyle with `Tools/CodeStyle/astylerc` is enforced by
`Tools/scripts/run_astyle.py` on the directories and files it lists. Elsewhere,
match the existing style of the file. Format only the lines you change, so
`git blame` stays useful.

## Naming

| Element | Convention | Example |
| --- | --- | --- |
| Classes | `AP_`, `AC_` or `AR_` prefix, PascalCase | `AP_GPS`, `AC_PID` |
| Methods | `snake_case` | `get_altitude()` |
| Member variables | leading underscore | `_singleton`, `_primary` |
| Constants and macros | `UPPER_SNAKE_CASE` | `AP_MOTORS_MOT_1` |
| Feature flags | `AP_<NAME>_ENABLED` | `AP_GPS_ENABLED` |

## Conventions

- Singletons use `get_singleton()`, and classes that must not be copied use
  `CLASS_NO_COPY()`.
- Files that need hardware access declare `extern const AP_HAL::HAL& hal;`.
- Compare floats with `is_zero()`, `is_positive()`, `is_negative()` and
  `is_equal()`, not `==`.
- User-facing messages use `GCS_SEND_TEXT()`, not `printf` or
  `gcs().send_text`.
- Time comes from `AP_HAL::millis()` and `AP_HAL::micros()`, not
  platform-specific functions.
- Optional features are wrapped in `#if AP_<FEATURE>_ENABLED` and
  `#endif // AP_<FEATURE>_ENABLED`, with the default in the library's
  `*_config.h`. Add a non-trivial feature to `Tools/scripts/build_options.py`
  so the custom build server can offer it.
- A core component never depends on an optional one: everything must compile
  with optional features disabled.
- No platform-specific code in shared libraries; go through the HAL.
