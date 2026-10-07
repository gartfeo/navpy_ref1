---
name: cleanup
description: Clean up after chats. Wrap-up stops what this chat started (GCS stack, dev servers, background commands), removes its scratch files, then archives the chat or sets auto-archive on its PR. Sweep reports what finished chats left behind (worktrees, branches, GCS slots) and removes what the user picks. Use when a task is done, after PRs merge, or when asked to clean up.
argument-hint: "[wrap-up | sweep]"
---

# Cleanup

A chat can't fully clean up after itself: its worktree is its own working
directory, and its branch and GCS slot outlive it. Archiving a chat stops it
and removes its worktree (the app archives automatically when the chat's PR
merges or closes), but leaves its GCS stack running, its registry slot claimed
and its branch in place. This skill covers both halves.

Mode (`$ARGUMENTS`): `wrap-up` for this chat, `sweep` for finished chats,
empty for both, wrap-up first.

## Rules

- Root `AGENTS.md` hard rules apply: never broad-kill, never
  `gcs_stop.py --all`.
- Report first. Beyond this chat's own processes and files, change nothing
  until the user picks the items in this chat. An app approval card does not
  replace that pick: in auto mode the app may approve on its own.
- Never touch running, pinned or open chats (other than this one during
  wrap-up), worktrees with uncommitted changes, unmerged branches, untracked
  files you did not create (other chats own them), or `.logs/` (evidence).
- Read the GCS registry file directly (`~/.gcs/instances.json`, or
  `$GCS_INSTANCE_REGISTRY`); `gcs_launch.py --list` and `gcs_stop.py --list`
  prune it.
- Chats in the same directory share its GCS slot, and a new chat's branch looks
  merged until it has commits. Decide ownership and "finished" from the chats
  and their PRs, never from the directory or git ancestry alone.
- The app tools named below (`get_status`, `set_monitor`, `list_sessions`,
  `archive_session`, `get_storage_usage`, `clean_up_worktrees`) exist only in
  the Claude desktop app. Elsewhere, inventory with `git` and `gh` and leave
  archiving to the user.

## Wrap-up (this chat)

1. GCS: if this chat launched a GCS stack or eval SITL, stop it from this
   chat's directory: `python scripts/gcs_stop.py` (interactive) and/or
   `python scripts/gcs_stop.py --eval`. Exit code 2 ("No ... instance for this
   directory") means nothing to stop; ignore its `--all` hint. Leave the stack
   running if the user wants it.
2. Stop dev servers and background commands this chat started.
3. `git status --short`: delete scratch and debug files this chat created;
   leave everything else.
4. Worktree, via this chat's PR (`get_status`):
   - merged or closed: offer to archive this chat (`archive_session` with
     `"self"`), which ends the conversation and removes the worktree; only on
     the user's yes;
   - open: auto-archive on close should be on; if `get_status` shows it off,
     offer to turn it on (`set_monitor`);
   - no PR: leave the worktree and say so.
5. Report what was stopped, removed and left, with reasons.

## Sweep (finished chats)

1. Inventory, read-only:
   - chats: `list_sessions` with `include_archived: true`;
   - worktrees: `git worktree list --porcelain`, `get_storage_usage`;
   - branches: `git fetch --prune`, `git branch -vv`, `git branch -r`,
     `gh pr list --state all --limit 200 --json number,state,headRefName`;
   - GCS slots: the registry file; match each entry's `clone` to a worktree
     and chat.
2. Classify each item:
   - **archive chat**: idle (not running, pinned or open) and its PR merged or
     closed;
   - **stop slot**: its `clone` directory is gone or its chat is archived;
   - **delete branch** (local, or remote): its PR merged, its chat archived or
     gone, and no worktree has it checked out;
   - **free worktrees**: chats inactive 30+ days without an open PR, via
     `clean_up_worktrees` (the app lists what qualifies on its card);
   - **leave**: everything else, with the reason (running, open PR,
     uncommitted changes, unmerged, no PR). The app's spare ready-made
     worktree is the app's to manage.
3. Show one short table and ask which rows to act on. Default: none.
4. Act only on the picked rows:
   - archive chat: `archive_session` (reason such as "PR #N merged");
   - stop slot: `python scripts/gcs_stop.py --chat <chat_index>`;
   - delete branch: spawn `janitor` with `delete: <branches>`; it re-checks
     safety and deletes;
   - free worktrees: `clean_up_worktrees` with the agreed day count.
5. Inventory again and report: removed, skipped with reason, remaining.
