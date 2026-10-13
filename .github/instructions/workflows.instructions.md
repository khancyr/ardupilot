---
applyTo: ".github/workflows/**,.github/actions/**,.github/zizmor.yml,.github/actionlint.yaml"
---

# GitHub Actions rules

- `.github/workflows/workflows_lint.yml` runs actionlint and zizmor on every
  workflow. Configuration is in `.github/actionlint.yaml` and
  `.github/zizmor.yml`; read the reasoning there before adding an exception.
- Grant the least permissions: `permissions: {}` or `contents: read` at the
  top, and widen only in the job that needs it.
- A workflow that runs pull request code has a read-only token. When results
  must be posted back to the PR, split the work the way `labels-trigger.yml`
  and `labels.yml`, or `test_size.yml` and `test_size_comment.yml`, do: the
  `workflow_run` half never checks out or executes PR code, and validates
  everything it reads from the artifact.
- Actions are pinned to a release tag and updated by dependabot.
- Build workflows ignore documentation-only changes through `paths-ignore`
  (`**.md`, most of `.github/**`), and keep their own workflow file as an
  exception so edits to it still run. Follow the same pattern in new
  workflows.
- Add a `concurrency` group so a new push cancels the previous run.
