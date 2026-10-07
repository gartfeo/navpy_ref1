---
name: cleanup
description: Clean up after chats. Wrap-up stops what this chat started and removes its scratch files. Sweep reports what finished chats left (worktrees, branches, GCS slots) and removes what the user picks. Use when a task ends or PRs have merged.
argument-hint: "[wrap-up | sweep]"
---

# Cleanup

A chat can't fully clean up after itself: its worktree is its working
directory, and its branch and GCS slot outlive it. Archiving (automatic when
its PR merges or closes) removes only the worktree. Mode: `$ARGUMENTS`
(`wrap-up`, `sweep`, or empty for both).

Rules:
- Root `AGENTS.md` hard rules apply: never broad-kill, never
  `gcs_stop.py --all`.
- Beyond this chat's own processes and files, act only on items the user picks
  here; an app approval card is not that pick.
- Never touch running, pinned or open chats, dirty worktrees, unmerged
  branches, files you didn't create, or `.logs/`.
- Read `~/.gcs/instances.json` directly; `--list` prunes it.
- Chats in one directory share its GCS slot, and a new branch looks merged
  until it has commits: judge ownership and "finished" by chat and PR, not by
  directory or git ancestry.
- App tools (`get_status`, `set_monitor`, `list_sessions`, `archive_session`,
  `clean_up_worktrees`) exist only in the desktop app; elsewhere use `git` and
  `gh`, and leave archiving to the user.

## Wrap-up (this chat)

1. If this chat launched a GCS stack or eval SITL, stop it from its directory:
   `python scripts/gcs_stop.py` and/or `--eval`. Exit 2 means nothing to stop;
   ignore its `--all` hint.
2. Stop dev servers and background commands this chat started.
3. Delete scratch and debug files this chat created.
4. PR merged or closed: offer to archive this chat (`archive_session` with
   `self`). Open: if `get_status` shows auto-archive off, offer `set_monitor`.
   No PR: leave the worktree.

## Sweep (finished chats)

1. Inventory: `list_sessions` (include archived), `git worktree list`,
   `git fetch --prune`, `git branch -vv`,
   `gh pr list --state all --json number,state,headRefName`, the registry.
2. One table, a row per item: **archive** (idle chat, PR merged or closed),
   **stop slot** (owner directory gone or chat archived), **delete branch**
   (PR merged, chat archived or gone, not checked out), **free worktrees**
   (chats idle 30+ days), **leave** (with reason).
3. Ask which rows; default none. Act with `archive_session`,
   `gcs_stop.py --chat <n>`, `janitor` (`delete: <branches>`), or
   `clean_up_worktrees`.
4. Report removed, skipped (reason) and remaining.
