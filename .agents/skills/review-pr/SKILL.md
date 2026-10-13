---
name: review-pr
description: Substance-focused review of an ArduPilot pull request (need, first-principles correctness, architecture fit, compatibility). Produces a debrief for the user and a draft review comment; never posts unless asked.
---

# ArduPilot pull request review

Goal: brief the user on a PR before they spend their own time on it, and draft
a comment they can choose to post. The review is the assistant's and is
presented as such.

Review substance only. Do not review formatting or commit-message style (CI
checks those) and do not recommend a merge strategy. Scale the effort with the
size of the diff and how flight-critical the code is. For a diff too large to
hold at once, split the work per subsystem when the client supports subagents.

## Gather

In parallel:

- `gh pr view <PR> --repo ArduPilot/ardupilot --json number,title,body,author,baseRefName,files,reviews`
- `gh pr diff <PR> --repo ArduPilot/ardupilot`. If GitHub refuses a very large
  diff, use `gh api repos/ArduPilot/ardupilot/pulls/<PR>/files --paginate`
  and fetch the important patches one by one.
- `gh pr checks <PR> --repo ArduPilot/ardupilot`
- existing feedback, inline and in the conversation:
  `gh api repos/ArduPilot/ardupilot/pulls/<PR>/comments --paginate` and
  `gh api repos/ArduPilot/ardupilot/issues/<PR>/comments --paginate`

Read any linked issue: the problem it claims is the baseline for judging
merit. To read full files after the change without touching the working tree:
`git fetch <ardupilot-remote> pull/<PR>/head`, then `git show FETCH_HEAD:<path>`.

Read the `.github/instructions/*.instructions.md` files whose `applyTo` globs
match the changed paths, then [review-criteria.md](review-criteria.md).

## History

When the PR changes existing behaviour rather than adding something new:

- Look for open issues and PRs on the same code or symptom (`gh issue list
  --search`, `gh pr list --search`).
- Run `git log -L` or `git blame` on the changed lines and read the PRs those
  commits came from. Was the current behaviour deliberate, and was the reason
  sound?
- Look for deferred decisions: TODOs, or review threads where this change was
  considered and postponed or rejected.

Report only what bears on the PR: a reverted decision the author does not
acknowledge, conflicting work in flight, or an unaddressed reason the code is
the way it is.

## Deliver

End with two things, then stop. Post nothing.

**1. Debrief for the user.** Lead with findings ordered by severity (blocker,
concern, nit), each with `file:line`. Give only the framing needed to judge
them: whether the claimed problem is real, scope and risk (subsystems touched,
flight-critical or not), and what CI and other reviewers already said. Restate
what the change does only when the description is missing or misleading.

**2. Draft review comment** in a fenced block. First line:
`**<assistant> review on behalf of @<login>**` (login from
`gh api user --jq .login`). Then terse, actionable findings only: what is
wrong, why it matters, what to do, with `file:line`. Frame each as an
engineering trade-off, not a judgement of the author. If you cannot show a
flaw through a concrete code path or a derivation, leave it out. No praise, no
filler, nothing other reviewers already raised.

`AGENTS.md` asks contributors not to paste raw AI text into reviews. The draft
is for the user to check, edit and own. Post it with `gh pr comment` only when
the user explicitly asks.
