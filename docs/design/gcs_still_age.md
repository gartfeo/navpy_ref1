# GCS: the confirm still's age and POI (todo 71)

Status: v3. Codex round 1 "rework", round 2 "approve with changes" (both round 2
points taken in here). The owner approved all six decisions (2026-11-01). The final
review added View camera's one width (under Change) and the movements that stay
(decision 6).
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
  began with (`confirmation_round_state.py:62`, `:133`), and detector updates build
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
  Deny move down by up to 100 px, and every card below moves too. With paced images
  this now happens seconds into every round, while the operator may be reaching for
  a button.
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
   request reached this browser; it costs nine characters. No clock time: nothing on
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
   also reserves its wrapped height in Armenian) until the image is in. Cost: about 100
   px per card during the empty period, the size the card reaches anyway once the
   image is in; in the 173 px RC list with two approved rows, about 50 px is left to
   scroll in, as today, so Approve and Deny are further down that scroll. Other choice:
   keep the 60 px box and the jump. Two movements stay. View camera's row shows only
   while the UAV has a stream, so when its last stream goes or a first one comes during
   a round, the row goes or comes and moves Approve, Deny and the cards below: 57 px in
   English and 94 px in Armenian at width 320 (todo 166). The other, from the merged cue
   design: the header's countdown words can wrap onto a second line at zero, when the
   operator's time is over anyway (with the app's font at width 320, on the first card,
   19 px once fired in English and 36 px from zero in Armenian; at most 19 px and 36 px
   over the fonts and widths measured; a second card moves 38 px and 72 px).

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
  with no fixed width; the age line is brighter than the note.
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
