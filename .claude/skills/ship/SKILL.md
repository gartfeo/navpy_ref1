---
name: ship
description: Ship this chat's work in one step. Checks the Definition of Done (tests, review), cleans up, commits, opens or updates the PR, and merges when everything passes; stops and asks only when something fails.
disable-model-invocation: true
---

# Ship

The user's `/ship` is the go-ahead for every step below, merge included; do
not ask again between steps. Run it only when the user invoked it. Stop at the
first failure and say what failed and what is needed. Anything outside these
steps still needs the user's yes.

## 1. Scope

- Review `git status` and everything this chat changed since `origin/main`.
  Ship only this task's changes: leave unrelated edits and untracked files you
  did not create (other chats own them). If it is unclear what belongs, ask.
  Nothing to ship: say so and stop.
- Never stage `.env*` files, tokens or keys, `.logs/`, or large binaries.

## 2. Verify

Walk every applicable item of the Definition of Done in the root `AGENTS.md`.
Mechanics:

- Python: `.venv` exists only in the main checkout. In a worktree, use
  `<main>/.venv/Scripts/python.exe` with `PYTHONPATH="$PWD/src"`. Run the
  targeted tests, then the full suite when shared, interface or
  safety-relevant paths changed.
- A failure is pre-existing only if it also fails on `origin/main` (re-run
  just those tests in a temporary worktree of `origin/main`); name it in the
  PR. Any other failure stops shipping.
- Frontend: `npm test` and `npm run build` in `src/gcs/frontend` (see
  `src/gcs/AGENTS.md` Verification). Worktrees have no `node_modules`: run
  `npm ci` there first, or pytest silently skips the vitest suite.
- GCS behavior: delegate to `live-tester`, unless this chat already has its
  verdict for the final code.
- `scripts/lua/*.lua`: redeploy to SITL as the Definition of Done says.
- `.claude/agents/*.md`: run `python scripts/gen_codex_agents.py` and ship
  both sides.

## 3. Review

Skip for docs-only diffs. Otherwise run `code-reviewer` on the diff, plus
`security-reviewer` when it touches comms, MAVLink, API routes, auth or
network code. Fix blockers (stop and ask if a fix needs a decision), repeat
steps 2 and 3, and list the warnings you leave in the PR.

## 4. Clean up

Run the `/cleanup` wrap-up steps 1-3: stop what this chat started and remove
its scratch files. Archiving comes with the merge.

## 5. Commit and PR

- Worktree chat: commit on its own branch.
- Main-folder chat: other chats share that folder, so never switch its branch
  or commit to `main`. Save the task's changes in the scratchpad (a
  `git diff HEAD --binary` patch plus copies of new files), create a temporary
  worktree from `origin/main` on a new `claude/<slug>` branch, apply them
  there (`git apply --3way`, copy the new files) and continue from it.
- Commit in the repo's style: imperative subject, a body that says why, and
  the session's attribution lines.
- Push, then open the PR against `main`, or update this chat's open PR. Body:
  **Summary** (what and why), **Verification** (level reached, commands and
  results, pre-existing failures), **Review** (verdict, warnings left),
  **Risk** (what could break; roll back by reverting the merge commit), then
  the attribution line. Finish it before merging: it is the durable record,
  and the chat may be archived as soon as the merge lands.

## 6. Merge

- Gate: no new test failures, no review blockers, the PR is mergeable, and
  the branch contains the latest `origin/main`. If `main` moved, bring it in
  (worktree chat: `sync_with_base_branch`; temporary worktree:
  `git merge origin/main`), resolve conflicts, repeat step 2 and push. If the
  repo has CI checks, they must have passed; if they are still pending, stop
  and report rather than poll.
- Merge in one command, so it completes before auto-archive stops the chat:
  `gh pr merge <n> --merge`; only if that succeeds,
  `git push origin --delete <branch>`, then (main-folder chat) restore the
  shipped tracked paths and delete the shipped new files in the main folder
  (the scratchpad copies are the backup), then `git -C <main> pull --ff-only`.
  If the pull refuses, report it; never force.
- Main-folder chat: then delete the local branch (`git branch -d`) and the
  temporary worktree.
- Reply with one line and the PR link. Auto-archive then closes a worktree
  chat and removes its worktree (if it is off, offer to archive);
  `/cleanup sweep` later removes leftover local branches.
