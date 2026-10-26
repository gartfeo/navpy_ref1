# GCS: E-STOP and Cancel task always in reach (todo 69)

Status: approved by the owner 2026-10-25 after Codex review rounds 1-4:
1(a), 2(a), 3 no, 4(c), 5 (32 px now; safety controls not an exception for
now), 6, 7 keep. Implemented 2026-10-25 (branch
`claude/nice-archimedes-582617`); the RC list's top moved to 94 px (4(c),
"As built").
Mock: `docs/design/gcs_estop_reach_mock.html` (static, drawn over
screenshots of the running app at 960 x 448, the UniRC emulator's
density-320 landscape viewport).

## Problem (measured)

Screenshots and element positions from the running app on origin/main
3c3034ba (3 SITL UAVs, s1-u7..u9; three confirm rounds injected into the
page's WebSocket only: s1-u7 pending with a still and View camera, s1-u8
and s1-u9 approved; `.logs/gcs-estop-reach/current`):

| Viewport | Sidebar (content) | E-STOP at scroll 0 | CANCEL TASK at scroll 0 | Scrolled to the end |
|---|---|---|---|---|
| 960 x 448 | 352 px (1618) | y 1010, fold at 400 | y 667 and 953, both below | E-STOP and cards above the view |
| 1280 x 648 | 552 px (1618) | y 1010, fold at 600 | y 667 and 953, both below | E-STOP and cards above the view |
| 1920 x 1080 | 984 px (1618) | y 1010-1032, just in view | both in view | s1-u8's CANCEL TASK cut at the top edge (y 33-66, view from 48) |

A pending card is 362 px tall and an approved card 274 px. At 960 x 448 a
pending card is taller than the sidebar: its APPROVE/DENY (y 383-414) are
already cut at the fold.

- The sidebar is one scroll area (`App.jsx:916-926`). Every confirm card
  comes first in it, in sys_id order (`MonitoringSidebar.jsx:73-97`).
  E-STOP comes after them, in the UAV Status header (`:431`). Nothing is
  pinned.
- Approved cards stay until disarm, cancel or reset
  (`taskConfirmationState.js:148-164`), each with its still (up to 160 px,
  `TaskConfirmCard.jsx:178-190`). Cancel task is only at the bottom of each
  approved card (`:288-298`).
- Planning hides every card. Edit plan is always offered, also while UAVs
  fly (`MonitorMapOverlay.jsx:74-81`); it switches to planning, where
  `MonitoringSidebar` is not mounted (`App.jsx:970`). A pending round then
  times out unseen and no Cancel task is shown. E-STOP shows (the TopBar
  fallback).
- Under manual control (RC) the cards sit in an overlay at top 60, width 260,
  with no height limit and no scroll (`App.jsx:853-883`), in a row that
  clips (`App.jsx:606`). Measured: the overlay is 903 px tall from y 108;
  both CANCEL TASK (y 650, 954) are below the map's bottom edge at 960 x 448
  and 1280 x 648, so they cannot be reached. The overlay (z-index 20) also
  lies over the right stick (150 px at bottom 24, z-index 10,
  `ManualControlOverlay.jsx:22-30`) when its cards are taller than 110 px
  at 960 x 448, as a pending or approved card with its still is. Pitch and
  roll then take no touch input.
- A dialog covers the whole screen, E-STOP and Cancel task included, until
  it is closed. None of them confirms its action when closed this way:
  - the app's confirm dialog (container launch, reboot, compass; E-STOP's
    own) closes on a tap outside it (`ConfirmModal.jsx:33`): one extra tap;
  - the browser's own confirms (`window.confirm`: arm/disarm, set all UAVs'
    mode, force launch or start, leaving RC, switching UAV, plan and
    settings changes; e.g. `FlightModeColumn.jsx:97`,
    `useManualControlWiring.js:42`) block the page until answered: one extra
    tap (Cancel);
  - settings, preflight and the panels inside them (pitot confirm,
    waypoint editor) do not close on an outside tap: their check reads
    `e.currentPoi`, a 2026-10-07 rename slip (e7d2c7f3,
    `PreflightReadiness.jsx:151`, `:507`, `SettingsModal.jsx:91`,
    `WaypointEditorOverlay.jsx:19`), so each needs its own Cancel or close.
    A dialog inside one of them adds a tap: up to three extra taps (a
    browser confirm in the waypoint editor in settings). That bug, which
    also breaks the parameter grid's scrolling (`VirtualParamGrid.jsx:92`),
    is a separate task.
- E-STOP's scope menu opens downward under the button (`EstopDropdown.jsx:88`)
  and closes on a tap outside it.

## Owner ruling to weigh

Owner, 2026-10-20 (`gcs_video_streams.md:617-624`): "our panel from right
always keep design same as we have". Options 1(a) and 2(a) change the right
panel: 1(a) takes E-STOP out of it, 2(a) turns an approved card into a row.
Showing cards in planning (below) also changes the planning panel while a
round exists. Please weigh them against that ruling.

## Proposal (recommended: 1a, 2a, 3 no, 4c, 5, 6, 7 keep)

One list for all card hosts, a new `ConfirmCardList`:

- **Pinned part**, never scrolled: the todo 67 summary line while a round is
  pending (count, UAVs, soonest deadline; this change only reserves the
  place, the confirm-cue-countdown session builds it), then one row per
  decided round:
  - approved: UAV badge, "Approval sent", a Still button, CANCEL TASK
    (32 px);
  - cancel requested: badge, "Cancellation requested", no button;
  - denied: badge, "DENIED", no button.
  As today, the cancel-requested and denied rows leave after 8 s. A sent
  cancel is not confirmed by the vehicle, so the row can leave while the UAV
  still flies its task. That is unchanged here; it is a separate task.
- **Scrolling part**: the still a row's Still button opened (one at a time,
  up to 120 px, with its badge and POI line), then the full pending cards as
  today (still, View camera, countdown, Approve/Deny). The still opens here,
  not under its row, so it can never push a CANCEL TASK out of view.
- **Hosts**: the right panel in planning and in monitor (the list on top,
  then today's planning or monitor sidebar in the same scrolling part), and
  the RC overlay (decision 4). The sidebars keep their look: same width,
  same cards for pending rounds, same UAV Status list. Cards move out of
  `MonitoringSidebar` into the panel, so they also show in planning. There
  the pending card has no View camera: the vehicle window exists only in
  monitor (`videoStreams.js:80`).

Height budget: pinned padding 18 px, summary 34 px, 36 px per row. Every
CANCEL TASK stays in view with up to 8 approved UAVs in the 352 px sidebar
at 960 x 448. Larger fleets on this screen belong to todo 73.

Cost: the pinned rows take room from the scrolling part. With two approved
rows and the summary (124 px) the scrolling part is 228 px at 960 x 448
(428 and 860 px on the larger sizes), so the pending card's APPROVE/DENY
need about 150 px of scroll on that screen instead of 15 px today. Reaching
a pending round without scrolling is todo 67's (summary, order) and todo
73's (handheld layout) work.

## Decisions for the owner

1. **Where E-STOP lives.**
   - (a) Recommended: always in the TopBar, in every phase (planning,
     monitor, RC). One place that is never inside a scroll. The fallback
     rule at `TopBar.jsx:63-70` goes away, and the sidebar header keeps only
     its title. It costs no sidebar height. Fit at 960 x 448, measured in
     RC (where the top bar already shows E-STOP): left group 280 px with the
     dev-mode branch badge, right group 287 px with E-STOP, 361 px free.
     Its scope menu keeps opening downward: while open it lies over the top
     rows of the right panel, and a tap outside closes it. As built, the
     menu ends at least 8 px above the screen's bottom edge, measured on
     opening and on every resize, and scrolls inside when the fleet is long
     (the app itself never scrolls).
   - (b) A pinned strip at the top of the right panel in planning and
     monitor (E-STOP right, the todo 67 summary left). E-STOP stays in the
     right panel, but the strip costs 36 px of panel height when no summary
     shows, and RC still needs the TopBar copy, so E-STOP keeps two homes.
   - (c) A sticky UAV Status header: it sticks to the sidebar's bottom edge
     while its place is below the view, and to the top edge after. Keeps
     today's spot (monitor only); the header floats over the cards or the
     UAV list, its scope menu must open upward at the bottom edge (it opens
     downward today and would be clipped), and planning and RC still need
     the TopBar copy.
   With no vehicles there is nothing to stop and no E-STOP, as today.
2. **How each approved UAV's Cancel task stays reachable.**
   - (a) Recommended: the compact row above. It removes the cause: approved
     cards no longer stack. Its cost to the pending card is stated above.
   - (b) Full cards as today, plus a pinned line of per-UAV chips
     ("Cancel task: [s1-u8] [s1-u9]"). The card design stays, but cards
     still stack and push the UAV list down, and each Cancel task shows
     twice.
   - (c) Cancel task on that UAV's card in the UAV Status list. That list is
     in the scrolling part, so it scrolls away on a short screen: it does
     not meet the goal. `test_estop_placement_js.py` bans only E-STOP
     strings and a ConfirmModal on that card, but its "no stop control"
     intent would need review.
3. **Cancel task in the video window.** Recommended: no. Keep the v12 rule
   of no flight actions in the window (`gcs_video_streams.md:231`).
4. **RC overlay.** The same `ConfirmCardList` at the sidebar's width
   (320 px; at today's 260 px a row does not fit), from top 60. Heights at
   960 x 448, 1280 x 648 and 1920 x 1080:
   - (a) At the right edge, down to 8 px above the map's bottom edge:
     284 / 484 / 916 px. At 960 x 448 the summary and two rows leave 160 px
     of scroll for the pending card (16 px with six rows). It lies over the
     right stick, so pitch and roll take no touch input, while a round is
     pending or a still is open, and with three or more rows.
   - (b) At the right edge, down to 8 px above the right stick:
     110 / 310 / 742 px. At 960 x 448 only two rows fit: it fails with three
     approved UAVs, and has no room for the summary or a pending card.
   - (c) Recommended: beside the right stick (its right edge 8 px left of
     the stick), down to 8 px above the UAV selector: 207 / 407 / 839 px
     (selector with its "Select UAV" line). At 960 x 448 the summary and two
     rows leave 83 px of scroll for the pending card; it holds up to four
     rows with the summary, five without. It never lies over a stick or the
     altitude tape. It covers the middle-right of the first-person view.
     **As built:** the list starts at top 94, 8 px below the first-person
     tag (top 54, 32 px tall, centred), not at 60: at 960 and 1280 px wide
     the centred tag (about 366 px) reaches into the list's columns, so a
     list from 60 would hide "not camera" or the first row's badge. Heights
     are then 173 / 373 / 805 px; at 960 x 448 the summary and two rows
     leave about 50 px of scroll, and it holds three rows with the summary,
     four without (enough for three UAVs). While the list shows, the
     full-width "View held" banner (top 98) is narrowed to the room
     between the flight-mode buttons and the list, 8 px from each (x 100 to
     the list's left edge minus 8), so neither hides its text (the live
     check found 60% of it hidden under the list at 960 x 448, and the
     narrowed banner's centred Armenian text would start under the buttons
     at x 41). The
     HUD's warning strip (bottom centre, up to five lines, about 85 px, takes
     no taps; `TelemetryHud.jsx:84`, `StatusStrip.jsx:35`) stays drawn above
     the list so warnings stay readable; at 960 x 448 and 1280 x 648 long
     warnings can lie over the pending card and, with four or more rows, the
     lowest rows, whose buttons still work. The fit is checked live with a
     UAV selected.
5. **Touch size.** Today E-STOP is 22 px tall in the sidebar (27 px in the
   top bar) and CANCEL TASK 33 px (todo 94). On 2026-10-20 the owner ruled
   that window controls use the app's 32 px map button "with no larger touch
   size" (`gcs_video_streams.md:617-624`). Recommended: E-STOP and the row's
   CANCEL TASK at the app's 32 px now; any larger size waits for todo 94's
   gloved measurement on the device. Question: should safety controls be an
   exception to that ruling?
6. **Gate for "problem seen in the video" to that UAV's Cancel task.**
   Recommended here: no scroll and one tap, at 960 x 448, 1280 x 648 and
   desktop, with three cards. The gloved time in seconds moves to todo 94's
   device session; entry 69 is then "done except the gloved timing, moved
   to 94".
7. **E-STOP and Cancel task while a dialog is open.** Recommended: keep, and
   accept the extra taps listed under Problem (one per open dialog, so up to
   three for a browser confirm in the waypoint editor in settings; fixing
   the `currentPoi` bug changes where a tap closes them, not how many).
   The other choice: the top bar stays above the app's own dialogs and
   opening E-STOP closes them unconfirmed. E-STOP is then one tap except
   under a browser confirm, which nothing can sit above; Cancel task stays
   under the dialog; and it touches preflight and settings, which other
   sessions own.

Not in this change: START MISSION below the fold on the UniRC (todo 73,
handheld-layout session), unless the owner asks to fold it in.

## Change (after approval)

Files under `src/gcs/frontend/src`:

- `components/ConfirmCardList.jsx` (new): the pinned part and the top of the
  scrolling part, used by the right panel and the RC overlay in `App.jsx`,
  replacing both `Object.entries` maps.
- `components/TaskConfirmCard.jsx`: the decided-row layout; no View camera
  in planning.
- `components/TopBar.jsx`, `components/EstopDropdown.jsx`: E-STOP always in
  the TopBar, 32 px. `MonitoringSidebar.jsx` drops its cards and E-STOP.
- `App.jsx`: the right panel becomes the pinned part over one scrolling
  part (pending cards, then the planning or monitor sidebar); the RC overlay
  uses `ConfirmCardList` with its placement and cap.
- `components/ManualControlOverlay.jsx`, `components/VehicleSelector.jsx`:
  export the sizes the RC cap is derived from (stick size and inset; the
  selector's line and chip heights), for 4(b) or 4(c).
- Locale strings for the Still button and the row labels.

Tests (vitest unless noted):

- E-STOP renders exactly once, in the TopBar and outside any scroll
  container, in planning, monitor and RC, with and without cards; none with
  no vehicles.
- Each approved UAV's CANCEL TASK renders in the pinned part with its badge,
  in planning, monitor and RC, and sends cancel for that sys_id only (sys_ids
  7, 8, 9; roster order changed; a new round for the same UAV; a failed
  POST keeps the button).
- Rounds survive monitor -> planning -> RC -> monitor, including a pending
  round that expires meanwhile and a forced-approval hold.
- Opening a still never moves a row; one still at a time; the still closes
  when its round is replaced; opening it while scrolled to the end.
- No View camera on a pending card in planning.
- The RC list has its placement and height cap, and its scrolling part
  scrolls.
- `test_estop_placement_js.py` changes from "sidebar primary, TopBar
  fallback" to "TopBar only"; the scope menu and ConfirmModal tests stay.
  `test_confirm_cancel_wiring_js.py` and `test_confirm_override_wiring_js.py`
  follow the wiring to `ConfirmCardList`, keeping forced approval, round
  identity, timeout and cancel guards.
- Live (live-tester; jsdom has no layout): at 960 x 448, 1280 x 648 and
  desktop, three cards, the full button box of E-STOP and each CANCEL TASK
  inside the viewport and hit-testable (`elementFromPoint`) next to its
  badge, at both ends of the scroll and with a still open; the scope menu
  open (its items hit-testable; the rows again after a tap outside closes
  it); planning with approved rows; all rounds approved; the row capacity
  limits stated above; RC with and without a UAV selected (no stick under
  the list; five long warnings readable over it); each dialog kind (app
  confirm, browser confirm, settings and preflight panels, nested), closed
  without confirming its action; video window to that UAV's Cancel task in
  no scroll and one tap.

Docs: entry 69 (decision 6), entry 94 (the moved gloved check),
`gcs_video_streams.md:257` (E-STOP is in the TopBar, cards in the right
panel); new todo entries for the `currentPoi` bug and for a cancel row that
leaves before the vehicle confirms.
