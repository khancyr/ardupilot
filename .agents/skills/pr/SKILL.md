---
name: pr
description: Prepare and open an ArduPilot pull request that follows .github/PULL_REQUEST_TEMPLATE.md, with honest testing evidence and AI disclosure.
---

# ArduPilot pull request

## Before opening

1. Work on a feature branch in the user's fork, never on `master`.
2. Check for an open PR on the same change:
   `gh pr list --repo ArduPilot/ardupilot --search "<keywords>"`. Do not
   duplicate recent work.
3. Rebase onto the latest `master` (the `rebase-onto-master` skill). No merge
   commits.
4. Build what the change can affect with the `waf-build` skill: SITL and the
   relevant vehicle for generic changes, the specific board for hardware
   ports. Fix every build error.
5. Run the relevant tests with the `run-tests` skill: at least SITL for the
   affected vehicle, the unit tests of the modified library and the related
   autotests.
6. Check style matches the surrounding code, and run `flake8` on changed
   Python files that carry the `AP_FLAKE8_CLEAN` marker.
7. Run `Tools/scripts/check_branch_conventions.py` and fix every failure.
8. Keep the diff minimal: no format-only churn, no stray comments, no moved
   code without a reason.

Push the branch to the user's fork first. CI runs there, and the results show
in the fork's Actions tab before anyone at ArduPilot sees the PR. Some checks
run only on pull requests; a PR into the user's own fork runs them too. CI
covers SITL tests for each vehicle, C++ unit tests, ChibiOS board builds,
astyle on the directories it covers, flake8, commit message format and
subsystem prefixes, binary size, pre-commit hooks and Markdown lint.

## Description

Fill in `.github/PULL_REQUEST_TEMPLATE.md` exactly:

- **Summary:** one or two lines on what the PR does.
- **Classification & Testing:** tick only boxes that are true. "Checked by a
  human programmer" is the user's to tick, never yours.
- Testing details below the checklist: what was run (SITL scenarios,
  autotests, hardware), the parameters used, and what it showed. Ask the user
  what they actually ran; never report testing that did not happen.
- **Description:** what changed and why, related issues (`Fixes #12345`) and
  discussions, and parameter changes with any migration considerations.
- State that the contribution was AI-assisted and how much, as `AGENTS.md`
  requires.

Keep it concise. Do not restate the diff.

## Open

Push with `-u` if needed, then
`gh pr create --repo ArduPilot/ardupilot --base master` with the title and
body above. Return the PR URL.

## After opening

- CI must pass. If it fails, investigate and fix; never ask maintainers to
  merge with failing CI.
- Answer review comments briefly, without jokes. Make straightforward changes,
  ask when a request is unclear, push new commits, and squash only when asked.
- Maintainers are volunteers; reviews can take days or weeks. They may ask
  for a large PR to be split into smaller, focused ones.
- For a non-trivial change, a discussion on
  [Discourse](https://discuss.ardupilot.org/c/development-team) or
  [Discord](https://ardupilot.org/discord) before writing code saves rework.

## After merge

Watch the PR for follow-up issues reported by testers, and be available to fix
regressions promptly.
