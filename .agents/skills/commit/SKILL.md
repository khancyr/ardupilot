---
name: commit
description: Create ArduPilot commits in the "Subsystem: description" format, one subsystem per commit, checked with the same script CI runs.
---

# ArduPilot commit

## Format

```text
Subsystem: short description of the change

Optional body explaining the motivation: why, not what.
```

- The first line starts with a subsystem prefix and a colon, and stays under
  about 72 characters.
- The prefix is the library or vehicle name: `AP_Terrain:`, `Copter:`,
  `GCS_MAVLink:`, `Tools:`. `git log --format=%s -- <file>` shows the prefix
  others used for the same files.
- One logical change per commit; split unrelated changes.
- No emoji, no jokes, no merge commits, no `fixup!` commits.

## One subsystem per commit

`Tools/scripts/allowed_subsystems.py` is the definitive list of prefixes: every
directory under `libraries/`, a `libraries/` directory the commit itself
creates, and curated extras such as vehicles, `Tools`, `autotest`, `waf`,
`hwdef` and `modules`. Every file in a commit must belong to its prefix. Some
files accept more than one:

- an autotest file: `autotest:` or `Tools:`
- a ChibiOS hwdef file: `hwdef:` or `AP_HAL_ChibiOS:`
- a vehicle file: the short prefix (`Plane:`, `Copter:`, `Sub:`, `Tracker:`)
  or the directory name (`ArduPlane:`, `ArduCopter:`...)

`subsystems_for_path()` in that file gives the candidates for any path.

## Steps

1. Check the branch with `git branch --show-current`. If it is `master`, ask
   the user for a feature branch name before committing.
2. Run `git status` and `git diff --staged`. If nothing is staged, ask what to
   stage.
3. Keep style changes to what the change needs, and match the file's existing
   style. Format-only commits go separately, and only when necessary.
4. Write the message from the actual diff. If the change spans several
   subsystems, commit it once and split it:
   `Tools/gittools/git-subsystems-split` (`git subsystems-split`) creates one
   commit per subsystem from your message. On an existing stack,
   `git subsystems-split --branch [BASE]` relabels wrong prefixes and splits
   multi-subsystem commits; add `-n` for a dry run.
5. Disclose AI assistance as `AGENTS.md` requires.
6. Check the branch exactly as CI does, and fix every failure before pushing:
   `Tools/scripts/check_branch_conventions.py`
7. Add new commits rather than amending pushed ones, unless the user asks for
   a squash. Never force-push unless the user asks or the branch is being
   rebased.
