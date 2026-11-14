# GCS: the confirm still's age and POI (todo 71)

Status: v3. Codex round 1 "rework", round 2 "approve with changes" (both round 2
points taken in here). The owner approved all six decisions (2026-11-01), before the
press paths in todo 166 (a lost camera, an expiring pinned row, a sooner round inserted
above) were measured; todo 166 has since fixed those with a press guard (`gcs_press_guard.md`).
The header wrap is separate: its words were in the text recorded with the approval at
7aa91c52, without numbers; whether the owner saw them is not recorded. The final reviews
added View camera's one width (under Change) and the section on press paths.
Mock: `docs/design/gcs_still_age_mock.html` (static drawing at the real host
sizes). Fit is checked live; jsdom has no layout.

## Problem (code on origin/main d5394e80)

- **The still has no age.** A pending card shows the header, the position, the
  still and View camera (`TaskConfirmCard.jsx:171-257`). To compare the still
  with live video, the operator needs to know how old it is.
- **The chosen frame has no time.** Frames have capture estimates upstream
  (`real_detected_object_builder.py:102-115`), but none stays with the frame kept
  for confirmation: `ConfirmationFrameStore.put` stores frame, box and score
  (`real_detector_state.py:81-90`) and `ConfirmationEvidence` has no time field
  (`detection_components.py:44-50`). Neither the request nor the image broadcast
  has one (`task_confirm_listener.py:106-115`, `task_confirm_images.py:106-112`).
  `TASK_CONFIRM_REQUEST.time_ms` is when the request was built.
- **What the GCS does know.** `receivedAt`: the browser clock (`confirmNow`) when
  the request reached this browser. The countdown starts from it. It is local:
  a reload or a second browser does not get earlier cards or stills
  (`useTaskConfirmation.js:45`), so there is no older receipt time to restore.
- **How the picture relates to the request.** A round keeps the POI object it
  began with (`confirmation_round_state.py:65`, `:138`), and detector updates build
  new objects (`real_detected_object_builder.py:80`). So both the first image and
  a resend use evidence retained from before the request
  (`confirmation_media.py:74`, `confirmation_inbound.py:54-65`). That evidence is
  the best-ranked frame over the track, so it can be much older. The time since
  receipt is therefore a low estimate of the picture's age, by an unknown amount.
- **The still comes late, or never.** Paced images (todo 116) mean the card usually
  has no still for the first seconds, and under heavy loss it may never come whole
  (todo 119). A POI with no frame sends no image at all
  (`confirmation_media.py:76-77`), so the card cannot promise that one is coming.
- **The card jumps.** The empty box is 60 px; the still is up to 160 px
  (`maxHeight`, `TaskConfirmCard.jsx:209-237`). When the image arrives, Approve and
  Deny move down by up to 104 px (measured at width 320), and every card below moves
  too. In the 7 sim rounds it came 8.3 to 8.8 s after the request; how often a press
  meets it is not measured.
- **Identity.** The header says "POI #N" and the burned-in thumbnail label says
  "D{poi_id}" (`thumbnail_rendering.py:112`). View camera's note says only "The
  countdown keeps running": the live camera may have moved on from this POI, and
  after zero the sentence is not even true.

## Proposal

Decisions for the owner, with the recommendation first.

1. **What time to show.** Request receipt time now, worded honestly (frontend
   only). Other choice: true capture time end to end (NavPy, the wire, a pymavlink
   pin bump, the backend; size M; a separate session). The demo comes first, so log
   true capture time as a follow-up, in the same TASK_CONFIRM_REQUEST dialect bump
   as todos 88 and 158.
2. **How the age shows.** A ticking relative age in m:ss, on two lines under the
   still:
   - "POI #3 · request received 0:07 ago"
   - "Taken before the request; exact time unknown"

   "Request received" and not "requested", because the GCS knows only when the
   request reached this browser; it costs seven characters. No clock time: nothing on
   the card can be compared with one, and the browser clock is not the vehicle's.
   The word "captured" never appears without a capture time. Other choices:
   "requested 0:07 ago" (shorter, a little less exact); or only the first line, with
   the second in a tooltip (saves 15 px, but touch screens have no tooltip).
3. **If true capture time is built later.** The frame's age, measured on the
   companion's clock when the request is built, so no clock sync is needed. The GCS
   adds its own time since receipt, so the transmission delay stays unknown and the
   result must still read as an estimate. Other choice: an absolute capture timestamp
   (needs synced clocks).
4. **Identity.** The caption reuses the header's "POI #N" (the burned-in "D3" is the
   same number). View camera's note becomes "Live view may no longer show POI #3. The
   countdown keeps running." Once the round is due or fired, or has no wait, the second
   sentence is hidden but keeps its space, so the note's height does not change.
5. **The empty period.** The box says "Waiting for image" (today's words, which
   promise nothing), with the age on the caption line. Once the countdown reaches
   zero with no image (a due or fired round that is still pending), it says "No image
   received". An image arriving later still attaches. A zero-wait round has no live
   period, so it shows "No image received" until an image arrives. An approved round
   with no image keeps today's row with no Still button.
6. **Keep the buttons still (new, not in the brief).** The picture area and both
   caption lines keep one size in every state, so nothing on the card moves when the
   image arrives, and no card below moves either: the box and the image have a fixed
   160 px height (the image `object-fit: contain`, which also covers a thumbnail of
   another shape or a broken one), and the second line stays reserved (hidden, so it
   also reserves its wrapped height in Armenian) until the image is in. Cost over
   origin/main, measured with one font at width 320: a card is 112 to 164 px taller
   while it has no image and still 8 to 60 px taller with one; in the 173 px RC list
   with two approved rows, about 50 px is left to scroll in, as today, so Approve and
   Deny are further down that scroll. Other choice: keep the 60 px box and the jump.

   The list can still move: see "The list can move under a press" below. The press guard
   (`gcs_press_guard.md`) now refuses a press that movement would misdirect.

The caption also appears on the opened still of an approved round (`ConfirmStill`),
where the same question comes up before a Cancel task. `ConfirmCardList` is not
edited: its tick runs only while a round is pending, so `ConfirmStill` runs its own
1 s tick on `confirmNow`, only while it is open.

### Clock and rules

- Age = `floor(max(0, now - receivedAt) / 1000)`, as m:ss with `formatClock`. `now`
  is the card's existing `clockNow`: the list's shared sample, or `confirmNow()` when
  the card is alone.
- With an integer wait and an unfired round between receipt and its deadline, the age
  and the countdown add up to the wait (floor plus the countdown's ceiling). Outside
  that interval they do not, by design: after zero the countdown holds at 0 and the
  age keeps rising, since the picture keeps ageing. A late NAV_CWT load moves the
  countdown, not the age. A clock step moves the age with it, as it does the
  countdown; before `receivedAt` the age reads 0:00, never negative.
- A card is only ever hosted by `ConfirmCardList`, whose tick runs while any round is
  pending, so a fired card keeps ticking. A card alone (tests only) stops ticking when
  it fires; no timer is added for that.
- Each new request event that reaches this browser is a new `confirmRequest`, which
  sets a new `receivedAt` and clears the image, so the age restarts. An image with a
  different round uid does not attach (`isOtherRound`). Three existing limits stay: with
  either uid missing, only the task id is compared; the backend gives an image the
  round open at its handshake (`task_confirm_images.py:74`), so a delayed old handshake
  can carry the new round's uid; and a sender without metadata gets one constant legacy
  uid (`task_confirm_rounds.py:37`), so within the retention time its re-ask is a repeat
  or is answered from the retained decision, with no new event (`task_confirm_listener.py:93-103`)
  and no restart. Transport repair stays with todo 119.
- `confirmImage` records `imageAt` (the same clock) for the matching task and round
  only, updated if a resent image replaces the picture, and returns the same state
  reference otherwise. The card exposes `imageAt - receivedAt` as
  `data-image-delay-ms`, so the live check can record it. It is not shown to the
  operator: it is the delay between two receipts (image generation, queueing, retries
  and link), and says nothing about the picture's age.

## The list can move under a press (todo 166)

This is the record of the hazard without a guard, before `gcs_press_guard.md`. The sizes still
describe where the buttons land; "posts" no longer holds for a press inside the guard.

All numbers: width 320 (the App's only width, `styles.js:92`), the real list and hooks in
headless Chrome on this PC, fetch stubbed, the app's font unless named, the list unscrolled
in a tall window unless named; not the real App, no device, no backend or vehicle. "This
PR" is c9d38855; origin/main is 5179baab. Approve fires on a plain click with no check for
a recent move (by reading, a forced card needs a hold), against "the tap never approves,
denies or pauses" (`gcs_video_streams.md:212`). "Share" is how much of the pressed button
lands on Approve: 22.7 %, 70.5 %, 75 % and 76 % are exact rectangles (sampled grids read
25 %, 67 to 73 % and 77 %), the others are sampled. "Posts" means the real handler sent
task_confirm approve to the stubbed fetch. How often any of this happens is not measured,
so these are changes in size, not in likelihood. A result that comes only from a reviewer's
report, with no saved output, says so; saved results are in `.logs/still-age/final-review`
of the main checkout, not in git. These are the triggers measured, not a complete list.

### Larger: a viewed card above loses its camera

View camera's row shows only while the UAV has a stream (`App.jsx:578`,
`TaskConfirmCard.jsx:254`; a camera is lost after 5 s without heartbeats,
`camera_protocol_timing.py`). The upper card shows Viewing, its last stream goes, and a
press lands at the centre of the next card's View camera.

- origin/main: rows 44 and 67 px, Approve moved 56 px, share 22.7 % (the button's lowest
  10 px), centre off it, posts nothing. This PR: rows 45 and 45 px, moved 57 px, share
  70.5 %, centre on Approve, posts approve for that UAV.
- New at the app's font is a press at the centre in English, with a mouse or a contact
  radius up to 12 px. 70.5 % is not a new maximum: origin/main has it with an unviewed card
  above, and this PR adds this state to it.
- Cause, in English at the app's font: the longer note removes the 23 px row difference
  that kept Approve away by accident. It is not the hidden label: 80580752, without it,
  posts (rows 44 and 45), and c9d38855 without the note's flex basis also posts (rows 82
  and 82, a reviewer's report). So keeping the note under the button would not bring back
  22.7 %.
- Armenian at the app's font: no change, posts on both (rows 67 and 67, 82 and 82). There
  the hidden label does matter: 80580752 posts nothing (rows 60 and 82).
- Other fonts at width 320 (image in, no scrollbar, each font forced on every element; hit
  test, no press): the centre is on Approve, never Deny, in 4 of 7 fonts in English (off:
  the app's font, Arial, Tahoma) and 5 of 7 in Armenian (off: Calibri, Times New Roman) on
  origin/main, in all 7 in both on this PR. With widths 280, 360 and 400 too, which the App
  does not use: 23 and 18 of 28 against 28. Which change gives all of them was not
  separated; a reviewer ties it to the equal rows.
- Roboto in English (Android Studio's font file in Chrome on this PC, no device) depends on
  the scrollbar, and which one the handheld has is not determined. Overlay (Chrome's mobile
  emulation; under 1 px of margin, not settled): origin/main rows 44 and 44, already posts,
  75 % against 76 %. In layout (10 px): origin/main's row 67 and gap 46, this PR's row 45
  and gap 23.5, so by arithmetic a similar rise to Windows' (22.7 % against 76 %, with
  Roboto's taller Approve); not pressed.
- Touch (emulated, English, tap after the loss): origin/main posts nothing at radius up to 12 px and
  posts at 14, 16 and 20 px, the largest tried; this PR posts at every radius tried. Finger
  held through the loss: origin/main nothing at 14 and 20 px, this PR posts (a reviewer's
  report). Real finger size not measured.
- List and scroll. The rows above are from the top of the list in a tall window. Over
  scroll positions with the pressed button visible (a sweep, not a model of behaviour), in
  English the centre is on Approve at 12 of 27 positions at 800 px and 5 of 20 at 600 px on
  this PR, 0 on origin/main. Armenian has no such rise: at 800 px origin/main 7 of 18 with no
  image and 11 of 27 with it, this PR 11 of 28; at 600 px 7 of 18 and 5 of 21, this PR 3 of
  20. In most scrolled states measured the next card holds still once the upper row is out
  of view, attributed to scroll anchoring (no run turned it off). With the upper card viewed,
  the saved per-state records (`refute48/run2.json`, a few states): in a 352 px window (list
  303 px) Roboto's next card holds; in a 600 px window (list 551 px, scrolled) it moves, and
  at the app's font origin/main posts nothing and this PR posts. A reviewer's table for a
  503 px list, not saved, has the next card moving at 18 of 77 and 26 of 65 positions on
  origin/main and 14 of 77 on this PR, with the centre on Approve at none and 14. At 960 x 448 with the button mid-list and an unviewed card above, the next card is
  held on both. By reading, View camera shows only in the monitor phase with manual control
  off (`App.jsx:936`, `:927`), so this case does not arise in the RC list; that list was not
  measured for any path.

### Larger in English: a pinned row leaves, with no camera loss

A denied row, also one denied by timeout (measured), goes after 8 s
(`useTaskConfirmation.js:18`) and everything below moves up 36 px per row. By reading, the
same for a canceled row and for an approved row removed by disarm or reset, at no fixed
time (`taskConfirmationState.js:122-138`, `:155-167`); reset on a denied row was measured
(36 and 72 px, same shares), the rest not run.

- One row, English, pressed card showing View camera: the note beside the button (flex
  basis, `TaskConfirmCard.jsx:276`) takes the gap from View camera to Approve from 46 to
  23.5 px and the share from 0 % to about 30 % (the button's lowest 12.5 of 44 px), centre
  off it by 9.5 px. A press 6 px from the bottom posts approve on this PR, nothing on
  origin/main. An emulated finger down about 75 ms before the row goes, lifted after: posts
  on this PR, nothing on origin/main. A mouse held through on this PR: posts nothing. Touch
  radius not measured here.
- A card already showing Viewing (a disabled button): origin/main already has gap 23, so
  31.8 % against 29.5 %.
- Armenian, one row: 0 % on both.
- Roboto in desktop mode with hidden scrollbars, English only: origin/main's gap is already
  23 px and the same press already posts (31.8 % against 29.5 %), so not larger there. With
  a 10 px scrollbar, and in Armenian, origin/main's gap stays 46.
- Two rows at once, English: about 60 % with the centre on Approve on both.
- Measured at the top of the list only. By reading anchoring cannot hold this one, since the
  pinned part is outside the scrolling part (`ConfirmCardList.jsx:133-164`); at the scroll
  end the move could be smaller; not run.

### Larger: a sooner round is inserted above

It needs different waits as the GCS sees them: equal waits sort below
(`taskConfirmationState.js:285-296`), the default is 30, and an unloaded NAV_CWT counts as
30 (`:210-219`). A sidebar edit
writes to all connected UAVs (`ConfirmSection.jsx:26-36`) and the demo eval config sets 30
for all (`eval_gcs_demo_config.py:165`), so default demo settings do not reach it; mixed
values are still supported. By reading, a value that loads or changes while cards are up
re-sorts them with no new round; not run.

- The fixed picture height puts the new card's Approve exactly under the old one's, so a
  press approves the new round while it has no image; Deny lands on the new Deny the same
  way.
- With View camera on both: English 392 and 392 px, Armenian 463 and 463 px with a
  scrollbar. If the scrollbar appears on insertion, Armenian's saved positions are 19 px
  apart (444 and 463 on this PR, 280 and 299 on origin/main); a reviewer's 38.1 %, on
  origin/main with the old card empty, fits that, a verifier's 96 %, given for both trees,
  does not, and neither is saved. With View camera on neither: 335 and 335, 369 and 369 px,
  so it also applies in planning; the RC list was not measured. One card with the row and one without: not run; by positions
  57 and 94 px apart.
- origin/main coincides only when both cards are in the same image state: both empty (280
  and 280, 299 and 299) or both with the image in (384 and 384, 403 and 403). Old card's
  image in and new card's not: 384 against 280, 403 against 299, posts nothing. English, old
  card empty and viewed: 257 against 280, 25.8 %, posts nothing; this PR posts, because the
  held width removes the height difference. So this PR adds the state with the old card's
  image in and the new card's not: 8.3 to 8.8 s in the live check, by reading the whole
  round if no image comes.
- On this PR a touch tap and a finger already down post, a mouse held through does not;
  origin/main posts on none of these with the old card's image in.
- A list already scrolled was held on both at panels of 352 and 400 px, attributed to scroll
  anchoring. A list that can scroll but sits at its top is not held: 400 px with no View
  camera still hits in English on this PR.
- By reading, neither the frontend path nor the route checks for an image or the card's age
  (`useTaskConfirmation.js:154-158`, `task_confirm.py:55-100`).
- The press results and scrolled-list results in this case come from a verifier's report
  with no saved output; the positions agree with saved files.

### Same size where the card moves: a lost camera on the card itself or on an unviewed card above

"Where the card moves" means the top of the list, or the upper row in view. Row height
cancels out. Approve moves 79 px on origin/main and 57 px on this PR in English, 79 and 94 px
in Armenian, and lands at the same offset under the old button (11 and 10.5 px, 11 px).
English: share 70.5 % on both. Armenian, on a 480-point grid: 240 Approve and 64 Deny with a
scrollbar, 248 and 56 without, on both; if the loss itself removes origin/main's scrollbar,
Armenian there moves 98 px and drops to 186 and 42. A mouse press or emulated tap at the old
centre posts approve, also with the finger already down; a mouse held through the loss on the
card itself does not (next card not measured). On the card itself the share is the same at
each height, including 960 x 448. Over scroll positions with an unviewed card above there is
no single direction (a sweep, not behaviour): mean share, English no image, .235 on
origin/main and .296 on this PR at 800 px, .235 and .167 at 600 px; Armenian no image at
600 px, .194 and .075.

### Smaller or gone on this PR

Other paths, which do not reduce the three larger cases.

- Two rows leaving at once, Armenian: no scrollbar, origin/main 47.5 % Approve and 10.7 %
  Deny, centre on Approve; this PR 21.1 % and 4.7 %, centre off it. 10 px scrollbar: 46 and
  12.2, 20.4 and 5.4.
- origin/main's 104 px image jump. English, with only the top card's image in (next and third
  cards empty): deciding the top card puts 52.3 % of the next card's View camera, centre
  included, on the third card's Approve, and a press 6 px from the bottom posts. This PR in
  that state: 0 % on Approve, 20.3 % on the third card's View camera, posts nothing. No
  Armenian run saved. When an image arrives, part of the next card's Map link becomes Approve
  on origin/main (13 % in English; 7 % with 9 % Deny in Armenian; centre on neither; a centre
  press posts nothing); gone on this PR.
- A first stream arriving on the upper card puts the next card's Map link in English 66 % on
  Approve on origin/main, centre on it; none on this PR. In Armenian about the same on both
  (37 % Approve and 48 % Deny; 32 % and 41 %; centre on neither). Hit test, no press; the
  link's position depends on the coordinates' digits.
- The 23 px English movement when View camera is pressed: a movement only, with no press
  measured reaching Approve through it, and the same row difference whose removal makes the
  first case larger.

### Other movement and limits

- The header wraps at zero, from the merged cue design: the countdown words can take a second
  line at zero (app's font, first card, 19 px once fired in English and 36 px from zero in
  Armenian; at most 19 px and 36 px over the fonts and widths measured; a second card moves
  38 px and 72 px when both headers wrap). The time is over for the wrapping card only: it
  stays pending with its buttons live, and by reasoning a card below with time left also moves
  19 and 36 px. A press during this move was not measured.
- An approved row offers Cancel while it stays; whether the vehicle's action can still be
  undone then was not checked.
- Other list moves, such as a card above being decided: saved results for one state; a
  reviewer's script covered more image states and reported that a second press after deciding
  reaches no decision button of the next card (no saved output).

### Options at the time (the press guard was built)

- Once a pending card has shown the row, keep it for the round, with the button disabled while
  there is no stream. Camera-loss paths only: not a first stream's arrival, row expiry or
  insertion.
- A short press guard on Approve and Deny after the button moves or appears: built,
  `gcs_press_guard.md`.
- Keep the note under the button. The one-row gap only: by arithmetic English row 82 px and gap
  about 61 px, so one row clears and two rows overlap about 11 px; the card is about 37 px
  taller; the first case still posts.
- About 13 px more margin under the row. The one-row gap only; by reasoning for a mouse press
  only, since it leaves 0.5 px.

## Change (after approval)

Under `src/gcs/frontend/src`:

- `hooks/taskConfirmationState.js`: `imageAt` in `confirmRequest` and `confirmImage`;
  `confirmAgeSec(entry, now)`.
- `hooks/useTaskConfirmation.js`: pass `receivedAt: confirmNow()` to `confirmImage`.
- `components/TaskConfirmCard.jsx`: a `StillCaption`, the box's words by phase, the
  fixed picture height (decision 6), the note's POI sentence and its hidden countdown
  sentence by phase (with a 120 px flex basis, so the note sits beside the button when
  it fits and under it when not), `ConfirmStill`'s caption and tick. The age line is
  `colors.text` and the note under it `colors.textDim`, as in the mock. View camera
  holds one width for both labels: the other label stays inside the button, hidden,
  with no fixed pixel width. Otherwise the button's width decides the note's side and
  line count, so the label switch moves Approve, Deny and the cards below. It holds the
  width, not the height: with fonts scaled 1.7 to 2 times (at 1.7 only in a list with a
  scrollbar), the Armenian label wraps past the button's 44 px minimum at width 320 and a
  press moves Approve 14-22 px.
- `locales/en.json`, `locales/hy.json`: add `task.stillCaption`, `task.stillAgeNote`,
  `task.noImage`, `task.viewCameraPoiNote`; `task.viewCameraNote` ends its sentence now
  that it follows another; drop the dots from `task.waitingForImage`. hy for the owner
  to check.
- `docs/design/architecture_todo.md`: mark 71 done, add the true-capture-time
  follow-up.

Not edited: `App.jsx`, `TopBar.jsx`, `MonitoringSidebar.jsx`, `styles.js`,
`ConfirmCardList.jsx`, map and mission-status files, backend, NavPy.

## Tests

- vitest: a pending card shows "Waiting for image" with the age before its image, and
  the caption with the POI id and age after; the age advances with fake timers; no
  "captured" wording; "No image received" at zero, when fired and with a zero wait; a
  late image replaces it.
- vitest: the age and the countdown add up to the wait at offset ticks for an unfired
  round; after expiry the age keeps rising; backward (partial and full) and forward
  clock steps; a late NAV_CWT load.
- vitest: a re-ask resets the age; an image with a different uid does not attach or
  change the caption, and one with a missing uid falls back to the task id.
- vitest: the picture area has the same height style with and without the image, and
  both caption lines exist in both states.
- vitest: View camera's note names the POI, and its countdown sentence is hidden but
  still rendered once the round is due or fired.
- vitest: View camera keeps the other label inside the button, hidden, in both states,
  with no fixed width; the age line is brighter than the note. The presence check and the
  two clicks find the button by role, since `getByText` also matches the hidden one; two
  checks that keep `getByText` fail when the labels swap.
- vitest: `ConfirmStill` shows the caption, ticks on its own, and stops its timer when
  closed.
- Reducer: `confirmImage` records `imageAt` only for the matching task and round, and
  returns the same reference otherwise.
- `tests/gcs/frontend/test_locale_translations.py`; npm test; npm run build; `python -m
  pytest tests/gcs`. The source-text tests on `TaskConfirmCard.jsx` keep passing.
- Live (live-tester, sim delivery demo): each card shows the empty state, then the
  caption; record each image's delay after its request; the age ticks; the identity
  matches the header; the countdown and View camera behave as before; no button moves
  when an image arrives or across live, due and fired, other than by a header wrap
  (`getBoundingClientRect` before and after). At 960 x 448 with
  three stacked cards, en and hy: E-STOP and every Cancel task reachable with no
  scroll. In the 173 px RC list with two approved rows, one pending round and an opened
  still, and through an image arrival: pending Approve and Deny reachable by scrolling
  and hit-testable.
- Browser, no stack (the real `ConfirmCardList` in headless Chromium; en and hy, 13 font
  settings, six widths from 280 to 400, tall and short lists, two cards): Approve on both
  cards, the button's width and the row's height stay the same within a phase, with no
  UAV or the first card's UAV being viewed (also the second card's UAV at live and
  fired). The image's arrival was measured in the live phase only. Result 2026-11-01:
  0 px in every measured case; before the one-width rule, Approve moved by up to 37 px.
