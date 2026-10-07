---
name: ship
description: Ship this chat's work in one step. Verifies the Definition of Done, reviews, cleans up, commits, opens or updates the PR, and merges. Stops only on failure.
disable-model-invocation: true
---

# Ship

`/ship` is the user's go-ahead for every step below, merge included; don't ask
between steps. Anything else still needs a yes. Stop at the first failure and
say what is needed.

1. **Scope.** Ship only this task's changes since `origin/main`; leave other
   edits and untracked files you didn't create. Unclear: ask. Nothing to ship:
   stop. Never stage `.env*`, keys, `.logs/` or large binaries.
2. **Verify.** Walk the Definition of Done in the root `AGENTS.md`. Worktrees
   have no `.venv` or `node_modules`: use `<main>/.venv/Scripts/python.exe`
   with `PYTHONPATH="$PWD/src"`, and run `npm ci` before frontend tests
   (pytest silently skips vitest without it). A failure is pre-existing only if
   it also fails on `origin/main`; name it in the PR. Any other failure stops.
3. **Review** (skip for docs-only diffs). `code-reviewer`, plus
   `security-reviewer` for comms, MAVLink, API, auth or network code. Fix
   blockers and repeat 2-3; list remaining warnings in the PR.
4. **Clean up.** `/cleanup` wrap-up steps 1-3.
5. **Commit and PR.** Worktree chat: commit on its branch. Main-folder chat:
   the folder is shared, so never switch its branch; save the changes in the
   scratchpad and apply them (`git apply --3way`) in a temporary worktree from
   `origin/main` on a new `claude/<slug>` branch. Repo-style message plus the
   session's attribution. Open or update the PR with Summary, Verification
   (level, commands, results), Review and Risk (roll back by reverting the
   merge commit). Finish it before merging: the chat may be archived as soon
   as the merge lands.
6. **Merge** when tests pass, review has no blockers, and the PR is mergeable
   and contains the latest `origin/main` (else sync with
   `sync_with_base_branch` or `git merge origin/main`, then repeat 2). Pending
   CI: stop, don't poll. In one command: `gh pr merge <n> --merge`; if it
   succeeds, delete the remote branch, restore the shipped files in the main
   folder (main-folder chat), `git -C <main> pull --ff-only` (never force),
   and delete the local branch unless a worktree holds it (`/cleanup sweep`
   gets those). Reply with one line and the PR link.
