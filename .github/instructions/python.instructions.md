---
applyTo: "**/*.py,**/wscript"
---

# Python coding rules

Does not apply to `modules/` (submodules).

- Files that opt into linting carry the marker comment `AP_FLAKE8_CLEAN`.
  Every new Python file adds it.
- flake8 follows `.flake8`: maximum line length 127. ruff also runs from
  pre-commit, configured in `pyproject.toml`.
- `black` with line length 120 applies only to `libraries/AP_DDS` and
  `Tools/ros2`.
- Import ordering follows `isort` with `profile = "black"`.
- Run `flake8` on every changed file that carries the marker, and
  `pre-commit run --files <files>` before committing.
