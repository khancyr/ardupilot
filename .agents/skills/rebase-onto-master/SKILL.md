---
name: rebase-onto-master
description: Rebase a branch onto ArduPilot master, including a branch built on another PR that has since been squash-merged, and prove no change was lost.
---

# Rebase onto master

ArduPilot does not accept merge commits, so branches are kept up to date by
rebasing. The tricky case is a branch built on top of another PR's branch: once
that parent is merged with different hashes, a plain `git rebase` replays the
parent's commits again and conflicts. The fix is to move only this branch's own
commits.

## Steps

1. **Pick the branch:** the one the user named, otherwise the current one. A
   branch checked out in another worktree cannot be checked out here
   (`git worktree list`); rebase it in place with
   `git -C <worktree-path> rebase <remote>/master`.

   When rebasing several branches in a loop, test each checkout explicitly,
   otherwise a failed checkout silently rebases whatever is checked out:

   ```sh
   git checkout -q "$b" || { echo "SKIP $b"; continue; }
   ```

2. **Find the ArduPilot remote** (`git remote -v`; often `upstream`, sometimes
   `origin`) and fetch it: `git fetch <remote> master`.

3. **Record the old state** so the result can be checked:
   `old_head=$(git rev-parse <branch>)` and
   `old_base=$(git merge-base <branch> <remote>/master)`.

4. **List the branch's commits:** `git log --oneline $old_base..<branch>`.
   Work out whether the oldest ones belong to a parent branch that is now in
   `master` under different hashes (match subjects and PR titles). If every
   commit belongs to this branch, run `git rebase <remote>/master` and go to
   step 6.

5. **Move only this branch's commits.** With `<first>` the oldest commit that
   belongs to this branch:

   ```sh
   git rebase --onto <remote>/master <first>^ <branch>
   ```

   Resolve conflicts, then `git rebase --continue`.

6. **Prove nothing changed:**

   ```sh
   git range-diff $old_base..$old_head <remote>/master..<branch>
   ```

   For the commits that belong to this branch, every line must show `=`. A `!`
   means a commit's patch changed and `<` means one was dropped. Anything but
   `=` needs a look before pushing. `$old_head` stays reachable through the
   reflog until then.

7. **Check conventions:** `Tools/scripts/check_branch_conventions.py`.

8. **Ask the user before force-pushing.** Then
   `git push --force-with-lease <fork-remote> <branch>`, never a bare
   `--force`.

   If another branch was built on this one, move it as well:
   `git rebase --onto <branch> $old_head <branch-above>`.
