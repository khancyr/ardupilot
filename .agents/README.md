# Agent instructions

`AGENTS.md` files hold repository and directory guidance. `.agents/skills/`
holds skills shared by every AI assistant. `.github/instructions/` holds
review and coding rules scoped by path through their `applyTo` globs; GitHub
Copilot reads them natively, and `AGENTS.md` tells other assistants to read
the ones matching the files they touch.

Keep skills client-agnostic: plain Markdown with only `name` and
`description` in the front matter, no tool-specific argument syntax and no
tool-specific attribution rules.

Claude Code reads `CLAUDE.md` and `.claude/skills/`, not `AGENTS.md` or
`.agents/skills/`. So each `CLAUDE.md` only imports its `AGENTS.md` with an
`@AGENTS.md` line, and
each `.claude/skills/<name>` is a relative symlink to `.agents/skills/<name>`.
When adding a skill or a nested `AGENTS.md`, add the matching symlink or
`CLAUDE.md` too.

| Skill | Use |
| --- | --- |
| `waf-build` | Build SITL or a board with waf, without flashing |
| `run-tests` | Run autotests, C++ unit tests and Python tests |
| `commit` | Commit with a valid subsystem prefix, one subsystem per commit |
| `pr` | Open a pull request that follows the ArduPilot template |
| `review-pr` | Substance-focused review debrief and a draft comment |
| `rebase-onto-master` | Rebase, including onto a squash-merged parent |
| `grilling` | Stress-test a plan or design one question at a time |

Create git worktrees where the user's own agent instructions put them.
Without such a rule, put them next to the repository in
`../ardupilot-worktrees/<name>/`, never inside it.
