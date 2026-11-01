# GCS: a confirm cue the operator cannot miss, and what zero does (todos 67, 68)

Status: v5. Codex review rounds 1-3 "rework", round 4 "approve with
changes" (finding 17 and the open parts of 3 and 7, all taken in here).
Approved by the owner 2026-10-28: wording "Auto-deny / Auto-approve"
(decision 5), chip with one badge and "+N" (decision 1), decision 7 a
separate bug, the rest as written.
Mock: `docs/design/gcs_confirm_cue_mock.html` (static drawing at the real
host sizes for 960 x 448: the 320 x 352 px right panel and the 320 x 173 px
RC list; cards simplified). Fit is checked live; jsdom has no layout.

## Problem (code on origin/main c4af1320)

- **Sound.** One 880 Hz, 0.15 s beep per request (`useTaskConfirmation.js:38-59`).
  It never repeats or escalates. The AudioContext is made on the first
  request, never `resume()`d and its state never checked, so a browser that
  blocks audio until a user gesture stays silent without saying so.
- **Visual.** Only the card's endless 1.5 s glow pulse (`TaskConfirmCard.jsx:114-123`).
  Nothing outside the cards shows the pending count, the UAVs or the soonest
  deadline. todo 69 reserved the first pinned line of `ConfirmCardList` for
  this (`gcs_estop_reach.md`, "Pinned part"; budget 34 px).
- **Order.** `ConfirmCardList.jsx:39-41` keeps `Object.entries` order (sys_id)
  in both hosts: the right panel (planning and monitor) and the RC list
  (`App.jsx:863`, `:921`). `visibleConfirmEntries`
  (`taskConfirmationState.js:199-204`) sorts by receipt time; only tests use
  it. NAV_CWT can differ per UAV, so receipt order is not deadline order.
- **Deadline.** Only inside each card: `receivedAt` (browser `Date.now()` at
  receipt, `useTaskConfirmation.js:57`) plus that UAV's cached NAV_CWT, or
  `AAS_DEFAULTS.nav_cwt` 30 s when not loaded (`ConfirmCardList.jsx:75`); the
  card also uses 30 s for a non-finite or negative wait (`TaskConfirmCard.jsx:61`).
  The request carries no deadline or policy (`backend/task_confirm_listener.py`).
- **Wording.** The card shows only "21s". At zero it sends Approve if
  `autoApprove`, else Deny (`TaskConfirmCard.jsx:83-89`). `autoApprove` is
  `getConfirmedValue(sid, 'nav_cm_fl', AAS_DEFAULTS.nav_cm_fl)`, so with
  params not loaded the GCS sends Deny. The fired guard is component-local:
  a card remounted by a host switch (RC <-> panel) can fire again; unchanged
  here.
- **What the vehicle does** (NavPy):
  - NAV_AUTO_CM 1: no card (`confirmation_coordinator.py:67-73`).
  - NAV_CWT 0 or less: after the request and its media, it confirms
    (`confirmation_round_runner.py:94-125`; a send error takes the failure
    policy instead).
  - Otherwise, at `first_send + NAV_CWT` it confirms only if NAV_CM_FL is on
    and the POI is still fresh; else TIMEOUT_REJECTED
    (`confirmation_dependencies.py:62-65`).
  - An operator response (`confirmation_response_state.py:37-52`): Approve on
    an open round confirms it, with no freshness check; Deny on an open round
    rejects it, and on a CONFIRMED POI recalls it; Approve on a rejected one
    does nothing. The GCS's timeout answer is such a response, and the link
    does not acknowledge it (`backend/routes/task_confirm.py:91`, `:127`).
  - The values are read at navigation start (`navigation_mode.py:166-168`,
    `nav_args.py:78-91`), and a companion CLI argument wins over the MAVLink
    param (`nav_args.py:42-48`). The GCS shows its cached MAVLink params
    (`useAasParams.js:278`, `:305`): an estimate of the vehicle's policy.
  - Two independent timers: the vehicle's starts after its first send, the
    GCS's at browser receipt. Which reaches zero first depends on link delay,
    scheduling, clock rates and whether the cached params match; neither is
    promised. A constant wall-clock offset cancels out (each side subtracts
    its own times); jumps and rate differences do not.

In the demo (`eval_gcs_demo_config.py:151-153`: nav_auto_cm off, nav_cm_fl
off, nav_cwt 30) a missed card rejects the dock.

## Proposal (recommended: 1-6; 7 and 8 separate tasks)

1. **Cue: a one-line summary chip**, the first item of the pinned part
   (`components/ConfirmSummary.jsx`, mounted by `ConfirmCardList`, so in
   both hosts, planning and monitor). 30 px plus a 4 px gap: exactly todo
   69's 34 px budget, so its measured room holds (right panel: 18 + 34 +
   8 x 36 = 340 of 352 px with eight approved rows; RC: unchanged, about
   50 px for the pending card with two rows). No other line is added in any
   state. The RC pending card already needs scrolling at 960 x 448 (accepted
   in todo 69; a usable minimum there is todo 73's handheld layout); the chip
   is the way to it.
   Content, left to right, in a fixed width budget (the 320 px host less
   16 + 16 px padding, 2 + 2 px border and 8 + 8 px chip padding = 268 px;
   5 gaps of 5 px): a bell and the pending count (28 px); the badge of the UAV
   whose time is shown (at most 72 px including its padding, ellipsis, full
   name in its title); "+N" for the other pending UAVs (up to 24 px; their
   names in the chip's title and on the cards right below); the shown time
   (up to 40 px, "~0:21"; coloured like that card's timer); the chip words
   (decision 5) in the rest, at least 79 px, with an ellipsis. Only the words
   shrink. One badge in both hosts: three badges do not fit with the time
   and the words. The card's full words and all pending UAV names are the
   chip's title and accessible name. Border amber, red when the shown time is 10 s
   or less (the card keeps green above 10 s, amber above 5 s, red at 5 s and
   below).
   Shown time: the soonest **live** round (pending, not fired, time left;
   decision 4). With no live round: the earliest **due** round (pending,
   at 0, not fired yet: the moment between the list's tick and the card's
   fire tick) with "0:00" and "Time up; sending answer"; else the first
   fired round with "Time up; answer not confirmed".
   The chip is a button (Enter and Space). A tap scrolls that UAV's pending
   card element (or, all expired, the first pending card) to the top of the
   scrolling part (an open still sits above the pending cards, so not
   scrollTop 0), resumes audio, and does nothing else (no approve, deny or
   focus move). It shows while any round is pending (expiry does not remove a
   pending round) and goes when none is; the open still and the decided rows
   do not change when it goes.
   Sound off: while a round is pending and the AudioContext is not running,
   the words read "Sound off, tap to enable" (the time stays); with no Web
   Audio, "No sound". Any tap on the page unlocks it.
   Other choices: (b) a badge next to E-STOP in the TopBar (always visible,
   but competes with E-STOP for the 361 px left at 960 x 448 and splits the
   cue from the cards); (c) a two-line chip with every badge and full words
   (clearer, 56 px: it leaves almost nothing for the pending card in RC);
   (d) only a stronger card pulse (a card scrolled away still cannot be
   seen).
2. **Sound**: one scheduler in `useTaskConfirmation` (App owns it), named
   constants, reading state through refs (image and status updates do not
   restart it):
   - Arrival (in the request handler, so also for a zero wait): one beep at
     once (880 Hz, 0.15 s), per new round (key: sys_id plus
     `confirmRoundKey`, so a re-ask of the same POI counts; a decision on an
     older round does not silence it). Several arrivals in one tick: one beep.
   - A 1 s tick on the shared clock runs while any live round exists:
     soonest live above 10 s (`URGENT_LEFT_SEC`): one beep when 5 s
     (`ALERT_REPEAT_MS`) have passed since the last sound; 10 s or less: a
     double beep (1320 Hz, 2 x 0.1 s, 0.1 s gap) when 2 s
     (`URGENT_REPEAT_MS`) have passed since the last sound, including a round
     that arrives already under 10 s; back to the 5 s beep when that round
     ends and the next soonest has more. One pattern per tick; when both an
     arrival and an urgent pattern are due, the urgent one plays.
   - Nothing is queued while audio is not running. Stop (no live round)
     cancels the tick and any pattern not yet started; a pattern already
     playing (at most 0.3 s, e.g. a zero-wait arrival beep) finishes.
     Unmount stops every oscillator and closes the context.
   - Unlock: `pointerdown` and `keydown` listeners (capture, passive) create
     the context if needed and `resume()` it; a chip tap does the same. They
     are removed once the context is `running` and put back whenever its
     `statechange` reports `suspended` again; a failed resume is retried on
     the next gesture. `soundState` ('running', 'suspended', 'unavailable')
     is exported for the chip.
   `App.jsx` passes `waitSecFor(sysId)` (the raw cached NAV_CWT, or
   undefined) into the hook.
3. **Order**: one pure selector `orderedConfirmEntries(state, waitSecFor,
   fired)` in `taskConfirmationState.js`, used by `ConfirmCardList` (both
   hosts): pending first: fired ones by fire time, then due ones by deadline,
   then live ones by
   deadline, soonest first, ties by numeric sys_id (2 before 10); then approved; then
   denied or canceled; each decided group by numeric sys_id. Pinned: chip,
   approved rows, denied/canceled rows (TTL unchanged). It replaces the
   unused `visibleConfirmEntries` (its tests and export guard move to the new
   selector). Unchanged: component keys per round; the forced mark per
   sys_id and task id (`forcedConfirms`, so a forced re-ask of the same POI
   stays forced, as today); the approve hold reset per round.
4. **Deadline and clock**: `utils/confirmClock.js` exports `confirmNow()`
   (today `Date.now()`), the one clock for receipt time, the card, the chip,
   the order and the sound. Pure helpers in `taskConfirmationState.js`:
   `confirmWaitSec(waitSec)` (the card's rule: a finite wait 0 or more, else
   30 s, flagged estimated), `confirmDeadlineMs(entry, waitSec)`,
   `secondsLeft(deadline, now)` (whole seconds, rounded up). The card's tick
   uses them with the same results (auto-fire unchanged). `ConfirmCardList`
   samples `confirmNow()` once a second and passes `now` to the chip and to
   each card's label, so both show the same second; a card alone (its tests)
   keeps its own tick for the label.
   **Fired latch**: the card's existing fire path (the place it calls
   onApprove or onDeny at zero) also calls a new `onAutoFire(sysId,
   roundKey)`; its behaviour is otherwise unchanged. The hook records the
   round as fired (by sys_id and round key, with the time). A fired round
   stays "time up" for the cue, the order, the sound and the card label even
   if a late param load gives a longer wait or the wall clock jumps back; it
   is never decided by the latch. Before the card fires, everything follows
   the arithmetic, as the card does: a round at 0 that has not fired yet is
   not live (no sound), and if a longer wait loads before the card's next
   tick, it is live again, the card counts on and fires later, exactly as
   today. A remount that fires again (existing) calls `onAutoFire` again;
   the latch keeps the first time. Late params before zero recompute the
   deadline from `receivedAt`, as today, and drop the "estimated" mark. A
   forward jump fires early, as today. `deterministic_dev_sim.md` swaps `confirmNow` for
   sim time in one place.
   Follow-up (new todo): the vehicle sends its deadline with the request,
   mapped into the GCS clock (not vehicle monotonic time compared with
   `Date.now()`).
5. **Wording: the GCS's own zero action**, which the GCS knows, from the raw
   cached values (no `AAS_DEFAULTS` for the label). First match wins:
   1. fired, still pending: "Time up; answer not confirmed";
   2. NAV_CWT 0 loaded: "No wait: GCS denies now" (NAV_CM_FL 0 or not
      loaded; decision 7) or "No wait: GCS approves now" (NAV_CM_FL 1).
      Implementation fix: v5 listed "due" before this, but a zero wait is
      due from the start, so these words could never show; decision 7 needs
      them, so they come before "due";
   2a. due (at 0, not fired yet): "Time up; sending answer";
   3. NAV_CM_FL not loaded: "Auto-deny in 0:21 (policy not loaded)";
   4. NAV_CM_FL 0: "Auto-deny in 0:21";
   5. NAV_CM_FL 1: "Auto-approve in 0:21".
   NAV_CWT not loaded or invalid: the time reads "~0:21" (the 30 s fallback).
   m:ss throughout. The chip shows the time, then the same words without
   "in 0:21" ("Auto-deny"). "Auto-deny" / "Auto-approve" name only what the
   GCS sends at zero. They promise neither delivery (the link is
   unacknowledged) nor that the vehicle still has an open round to apply it
   to. The vehicle's own timeout rule is not on the card;
   it is the cached estimate described above. Follow-up (new todo): send the
   policy with the request (the TASK_CONFIRM_REQUEST dialect bump, with todos
   88 and 71), after which the card can name the vehicle's outcome.
   Alternative (the brief's wording): "Rejects in 0:21" / "Approves in 0:21
   if still tracked". Not recommended: both state a vehicle outcome the GCS
   cannot know (delivery and the vehicle's round are unconfirmed), and "if
   still tracked" is wrong for the GCS's Approve, which confirms an open round
   without the freshness check.
6. **Demo policy**: keep reject; the demo card reads "Auto-deny in 0:30"
   ("Rejects in 0:30" with the alternative wording).
7. **NAV_CWT 0 with NAV_CM_FL 0 or not loaded**: the card sends Deny at
   once, which can reject the open round or recall its confirmation
   (`confirmation_response_state.py:37-49`). Not reachable in the demo
   (NAV_CWT 30). Recommended: a separate small bug (new todo); the label says
   what the GCS does until then.
8. **Recording the timeout outcome** (timeout_approve / timeout_deny as the
   decider): a separate task under the deferred "who decided each confirm"
   audit (new todo).

The card's auto-fire at zero is unchanged (it still uses
`AAS_DEFAULTS.nav_cm_fl` when not loaded, and its local fired guard).

## Change (after approval)

Under `src/gcs/frontend/src`:

- `utils/confirmClock.js` (new): `confirmNow`.
- `hooks/taskConfirmationState.js`: `confirmWaitSec`, `confirmDeadlineMs`,
  `secondsLeft`, `orderedConfirmEntries`, `pendingSummary(state, waitSecFor,
  now, fired)` (count, the shown UAV and deadline, "+N", fired
  count; null with none pending), `confirmZeroLabel(...)` (decision 5's
  locale key and args).
- `hooks/useTaskConfirmation.js`: the scheduler, unlock, `soundState`, the
  fired latch.
- `components/TaskConfirmCard.jsx`: the label from a new `labelPolicy` prop
  (raw cached NAV_CM_FL and NAV_CWT, undefined when not loaded) plus
  optional `now` and `fired`; the existing `autoApprove` and `timeoutSec`
  props (with their `AAS_DEFAULTS` fallbacks) still drive auto-fire;
  `onAutoFire` called from the existing fire path.
- `components/ConfirmCardList.jsx`: the order, the shared tick, the raw
  `labelPolicy` per card beside today's effective props, card refs (by
  sys_id and round key) for the chip's scroll, mounts the chip.
- `components/ConfirmSummary.jsx` (new): the chip.
- `App.jsx`: pass `waitSecFor`, `soundState`, the latch and the unlock only.
- `locales/en.json`, `locales/hy.json` (hy for the owner to check).

Tests (vitest; Node for the pure module):

- Order: UAV 3 NAV_CWT 10 received 5 s after UAV 1 NAV_CWT 30 sorts first;
  then approved, then denied/canceled; ties by numeric sys_id (2 vs 10);
  fired pending first; unknown wait uses 30 s; decided TTL unchanged.
- Wiring: both hosts render `ConfirmCardList`, and it orders with
  `orderedConfirmEntries` (no `Object.entries` order left).
- Summary: count, shown UAV and deadline, badge order with mixed expired and
  live rounds; null with none pending; all expired.
- The card and the chip show the same whole second at offset ticks.
- Chip: renders, updates each second, scrolls the shown UAV's card into view
  with a still open, keyboard activation, "+N" (4 or more pending), long
  names, stays with only expired pending rounds, goes when the last pending
  round is decided, reset or removed by disarm; planning, monitor and RC.
- Latch: failed POST then a longer wait loads; a POST that never resolves;
  backward jump after firing; a longer wait loading between the hook's zero
  and the card's fire tick (no latch, card fires later, cue live again);
  ordering of a fired round after a wait extension next to a live round;
  the chip at offset ticks: due before the fire, fired after it, live again
  when a wait extension revives an unfired due round;
  host switch with a fired pending round (the existing re-fire on remount
  documented, not changed).
- Host labels: `ConfirmCardList` with NAV_CM_FL and NAV_CWT not loaded shows
  the not-loaded label and "~", while the card still auto-denies at 30 s.
- Labels: each rule in decision 5, the "~" mark, late valid and invalid
  params; the existing auto-timeout and forced-hold tests pass unchanged
  (display-text assertions such as "6s" in `ConfirmCardList.test.jsx`
  change to the new label with the same timing).
- Fake AudioContext: arrival beep per round (re-ask included; a zero-wait
  arrival beep plays its full 0.15 s, while unmount stops it at once); 5 s repeat; double beep at 10 s, on arrival under 10 s, and
  back; one pattern per tick; stop on decide, expire, reset, disarm and
  unmount (oscillators stopped, context closed on unmount); unlock on the
  first pointerdown, retry after a failed resume, listeners back after a
  later suspend; 'suspended' and 'unavailable' on the chip; image updates do
  not restart the period; a stale response during a re-ask does not silence
  the new round.
- `ConfirmCardList.test.jsx`'s RC harness uses the as-built 173 px cap (it
  still has todo 69's first 207 px).
- npm test, npm run build, `python -m pytest tests/gcs`.
- Live (live-tester): 3 SITL UAVs on the demo config with NAV_CWT 30, 20 and
  10 (one with NAV_CM_FL 1), then one with NAV_CWT 0; desktop, 1280 x 648 and
  960 x 448; en and hy; chip visible with the sidebar scrolled, the vehicle
  window maximized, in planning and under RC (RC with two decided rows,
  sound blocked and hy: the chip and every CANCEL TASK in view, and after a
  chip tap the pending card's APPROVE and DENY reachable by scrolling the
  list and hit-testable); the chip's time never cut with three 12-character
  UAV names, 4 pending rounds ("+N") and the longest hy words; order by time
  left in both hosts; a demo card reads "Auto-deny in 0:30"; with stills, an
  open still; sound by instrumentation (AudioContext.state
  'running' after one tap, an oscillator on each request). Injected requests
  only with the response POSTs intercepted, so no injected answer reaches a
  vehicle.
