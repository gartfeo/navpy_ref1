# GCS: a press guard on Approve and Deny (todo 166)

Status: built. Codex rounds 1 and 2 said "rework"; every point answered (`.logs/press-guard`). The owner kept `EXPIRY_OPEN_MS` at 2 s (2026-11-02).

## Problem

A pending card's Approve and Deny fire on a plain click. The list can move while a press
is on its way: a card above loses its camera row, a pinned row expires, a sooner round is
inserted above, a card above is decided, a header wraps at zero. The press then lands on
another card's Approve or Deny and sends a real command. Measured cases:
`gcs_still_age.md`, "The list can move under a press". The list has no single event for
"this button moved", and the triggers measured are not a complete list, so the fix must
not depend on naming each trigger.

## Established approaches

- **Chromium's input protector** (`ui/views/input_event_activation_protector`): dialogs
  ignore mouse, touch and gesture input for one double-click interval (500 ms by default)
  after the view is shown or its window moves. Key events go past it unless a dialog asks
  otherwise. This is the same problem, and the design below takes its rule and its length.
- **Floating UI `autoUpdate`** follows an element's position with observers by default and
  has an `animationFrame` option that reads it each frame. We need a time of the move, not a
  re-position, and it is not a dependency here, so only the method is taken.
- **Layout Instability API** (`layout-shift` entries): rejected. It lists at most five
  sources, and any other element's shift would trigger the guard.
- **Keeping the layout still** (keep the row for the round, more margin): each covers one
  trigger only (`gcs_still_age.md`, Options). One fixed decision area bound to a selected
  round would keep a position still, but is a larger redesign of a list the owner approved,
  and changing the selected round would still need a guard.
- **React Aria `usePress`**: not a dependency, and it does not cover a button appearing under
  a finger. Only its rule for virtual clicks is taken.

## Design

One hook, `hooks/usePressGuard.js`, used by the pending card.

**What is a move.** While the card is pending, the hook reads the actions row's rectangle and
the list's `scrollTop` and `scrollHeight` once per animation frame, and again at each
`pointerdown` and click. A frame read is the only method here that sees every cause without
naming it. Against a kept baseline it is a move when left, width or height differs, or when
top differs and top + scrollTop or scrollHeight differs too. The baseline stays until a
difference reaches `MOVE_EPSILON_PX = 1`, so slow drift adds up; that value absorbs the
fractions in rectangles and scrollTop during a scroll. So:

- the operator's own scrolling is not a move (top changes, top + scrollTop and scrollHeight
  do not), and neither is the chip's scroll to a card, which the operator also asked for;
- scroll anchoring holding the button still is not a move (top does not change);
- content leaving, above or below, with the scroll clamped, is a move (top and scrollHeight
  change).

Mounting and a new round on the same card (`roundKey`) count as a move ("appears"). Move times
are `performance.now()`, the clock of `event.timeStamp` in Chromium; deadlines stay on
`confirmNow()`.

**The two rules for a pointer click.** One `pointerdown` listener on the document, in the capture
phase, keeps the gesture's record: the Approve or Deny button of this card it landed on and its
`event.timeStamp`, or nothing when it landed anywhere else. So every new press replaces the record,
and one that began elsewhere cannot inherit an old one. A click consumes the record, and
`pointercancel` clears it. A second finger elsewhere also clears it; that press needs repeating.

1. **Same gesture, always on.** A click with no `pointerdown` recorded on that same button, or
   with a move since that `pointerdown`, is refused. This press did not start on this button or
   the button moved under it, so it was not aimed here. Measured cases: a finger on View camera
   when a card slides under it, a sooner round's new button under a finger.
2. **Cooldown.** Otherwise a click whose `pointerdown` began under `PRESS_GUARD_MS = 500` after
   the last move is refused. So the limit is on when a press begins: one begun at 400 ms and
   released at 1500 ms is refused, and the buttons were dim when it began.

**An expiring card has no cooldown.** A refused press has to be noticed and pressed again;
if time runs out first, the timer sends its action instead and the refusal has changed what the
GCS sends. So rule 2 is off, for both buttons and every `NAV_CM_FL`, when
`msLeft(deadlineMs, confirmNow()) <= EXPIRY_OPEN_MS` (2000: the guard plus about 1.5 s to see
it and press again; a press refused with more time left can still be repeated). This includes an
overdue card still pending, as on main today. Cost, stated plainly: in a round's last 2 s
a press landing just after a move still acts, the same hazard as today, there only. Rule 1 stays
on, except that a press begun on this button and moved under stays valid inside the 2 s: it
refuses only a press that did not begin on this button.

A press refused with more time left is not lost silently: the pressed button's label reads
"Press again" for `REFUSED_LABEL_MS = 1500`, in the same space. Without another press the timer
still acts at zero, so a cooldown cannot be absolute on this point; the owner accepted `EXPIRY_OPEN_MS`
(2026-11-02) (Codex's round 2 point 1).

**What is not refused.**

- Virtual clicks: `event.detail === 0` (keyboard, most assistive input) or `event.buttons === 1`
  on a click (TalkBack, React Aria's `isVirtualEvent`). A move cannot change which button they act on.
- Auto-fire: `fire()` calls `onApprove`/`onDeny` directly and never reads the guard, so what the
  GCS sends at zero, and when, is unchanged.

**Forced cards.** The hold Approve is not click-based, and a hold can begin only on this button,
for this round, and takes 1 s with its progress showing, so it is not guarded. Todo 73
(`gcs_handheld_layout.md`) since ends a pointer hold when the press point lies more than 8 px
outside the button, as the finger or the button moved; a keyboard hold has no press point, so a move
does not end it (Codex's round 2 point 3). Either ends when the button's box is gone. One fix: `useLongPress` has no
`touchcancel` handler, so a touch the browser takes for scrolling still completes a hold (review 1,
by an in-memory check). `onTouchCancel` now cancels it; this also fixes the launch holds that use
the hook. The forced card's Deny follows both rules.

**What the operator sees.** While rule 2 applies, the plain Approve and Deny dim to 0.6 opacity;
the layout does not change. The dim's end is a timer and its expiry test is read at render, so it
can lag by up to a list tick; the press decides by the rules above, not by the dim. No aria change:
a 500 ms flicker read aloud would be noise and virtual clicks pass. The native `disabled`
attribute is not used, because it would also take keyboard activation away.

**Where it hooks in.** The document listener above, and `onClickCapture` on Approve and Deny; a
refusal calls `stopPropagation`, so the buttons' `onClick` stay as they are
(Codex checked this on the installed React). `ConfirmCardList` passes pending cards one more prop,
`scrollRef`. `data-press-guard` on the actions row and `data-press-moves` (the count) are for
the tests and the live check.

## Limits

- Other controls that move (View camera, Map link, Cancel task, the chip) are not guarded: none
  approves or denies, and Cancel task must never be delayed.
- A deliberate press inside 500 ms of a move needs a second press. Moves in a row lengthen that;
  only real moves count, and the live check records how many a round has. How often an operator
  meets it is not measured, as with the hazard.
- A press begun 500 ms or more after a move is treated as aimed. Nothing in the GCS can tell.
- The actions row is measured, not each button; the two share it and split it evenly.
- Existing race, not changed: a pending card whose timeout response is on its way, or has failed,
  still takes a press, and a Deny then can follow an approve already sent.
- An outer scroller other than the list's own, if a host adds one, reads as moves: safe, a short guard.

## Tests

- vitest, real card with stubbed rectangles and event times (jsdom has no layout), each failing
  without the fix: rule 2 for a move, an appearance and a new round, with exact 499 and 500 ms
  boundaries; rule 1 for a press begun elsewhere at any age and a press held across a move;
  a click with its record consumed; delayed delivery (old `timeStamp`, late handler); a move seen
  only at the press (frames suspended); scroll, anchoring (no refusal); clamped scroll
  (refusal); drift of 0.4 px per frame adding up.
- vitest: virtual clicks pass inside the guard; auto-fire inside the guard, same action; inside
  `EXPIRY_OPEN_MS` both buttons pass rule 2 and rule 1 still refuses; a stale record after a drag-out
  is replaced; overdue pending; touchcancel ends a hold, a keyboard or clean hold across a move
  completes; "Press again" on and off; dim on and off; the frame loop stops after a
  decision and on unmount.
- Browser, no stack: the saved harness (real list, headless Chrome, 320 px), real events by CDP, on
  253f7af9 (main, with PR 48) and this branch: a lost camera on a viewed card above, on an unviewed
  card above and on the card itself; one and two pinned rows leaving; a sooner round; a card above
  decided; press inside the guard and a deliberate press after it; mouse, emulated tap, finger
  already down; en and hy. The "larger" comparison in `gcs_still_age.md` stays pinned to 5179baab
  and c9d38855. Incremental frame time with three pending cards, guard on and off (the timer bar's
  width transition already runs layout each frame while a card is up).
- Android WebView (the repo's emulator and WebView image, if free): a real `adb` tap on a test
  page or the app: `detail`, `pointerdown` target and `timeStamp` clock are as assumed.
- Live (`live-tester`, sim demo): deliberate Approve and Deny work; moves counted over a round match
  the visible changes; a list move with the real backend, using unequal `AAS_NAV_CWT` for the
  insertion; View camera's width holds and Approve and Deny do not move between View camera and Viewing.

## Results (2026-11-02, this branch against main 253f7af9)

Evidence: `.logs/press-guard` of this checkout, not in git.

- Browser, no stack (real list, headless Chrome, 320 px, stubbed fetch, real CDP input): seven list
  moves (a lost camera on a viewed card above, on an unviewed card above, on the card itself; one and
  two rows leaving; a sooner round; a card above decided), en and hy. Mouse, tap and finger already down,
  under about 100 ms after the move: 42 of 42 post Approve on main, 0 on this branch. Mouse and tap
  650 ms after: all 28 post once on both. A press right after a wheel scroll, or on a button scroll
  anchoring held, posts. A Deny right after a move in the last 2 s posts. Auto-fire: same actions and times.
- Cost: three pending cards, one 10 s sample each, two runs: script 29 and 25 ms on main, 66 and 77 ms here;
  layout, style and task time within the two runs' spread. No device.
- Android WebView 101 in the repo's shell, on an emulator, real `adb` taps on a test page: a tap's click has
  `detail` 1 and `buttons` 0; `timeStamp` is on the `performance.now()` clock; with the buttons swapped
  during a press, `pointerdown` was on one button and the click on the other, so rule 1 applies. The guard itself was
  not run there.
- Live (`live-tester`, real stack and SITL, headless Chrome, real input): View camera to Viewing moves nothing
  in en and hy (button 110.6 and 163.5 px wide before and after; Approve and Deny unmoved; no move counted). Deliberate
  mouse and touch Approve and Deny each post once. A press 45 ms after a real card above decided: no POST, PRESS AGAIN, dim;
  a press at 733 and 760 ms posts once. A Deny 170 ms after a move at 0:02 posts. 54 timeouts: each a Deny at 29.8 to 30.8 s,
  as labelled. 90 images: none moved anything by itself. Moves per round at most 8. No console or backend errors.
  Not reached live: a denied row expiring under a press and an inserted sooner round (both covered in the browser run above).
- Moves with no rectangle change in the live recorder: 114 of 177 by its count. The saved runs hold 41; 34 come 88 to 294 ms after a
  visible change. A run with 150 ms frames reproduces this: the rectangle changes at once and the count rises one frame later. So
  on a busy page a move is seen up to a frame late and the guard runs that much longer, never shorter (a press re-reads the
  position). The other 7 were not separated; each adds at most one more guard time.
- vitest: 21 of the 31 new tests fail without the fix; the other 10 check what must pass.
