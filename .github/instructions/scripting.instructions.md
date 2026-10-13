---
applyTo: "libraries/AP_Scripting/**"
---

# Lua scripting rules

- Bindings are generated from
  `libraries/AP_Scripting/generator/description/bindings.desc`. A binding
  change also updates `libraries/AP_Scripting/docs/docs.lua`; CI compares the
  two with `libraries/AP_Scripting/tests/docs_check.py`.
- Existing scripts must keep working: do not rename or change the signature of
  a binding without a reason stated in the PR.
- Lua code is checked with luacheck and the Lua language server
  (`Tools/scripts/run_luacheck.sh`, `Tools/scripts/run_lua_language_check.py`).
- A new applet in `libraries/AP_Scripting/applets/` comes with a Markdown file
  of the same name explaining what it does and how to configure it.
- Scripts run on the vehicle: a script error or a slow script must not affect
  flight. Watch for unbounded loops and allocations in `update()` functions.
