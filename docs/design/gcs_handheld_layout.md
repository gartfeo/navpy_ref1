# GCS: handheld layout, collapsible right panel (todo 73)

Status: the owner approved all seven decisions below on 2026-11-01; built under
todo 73 and checked in the running app and the UniRC emulator, not on the device (Live
results, at the end). Codex reviewed this design and the finished diff in rounds; every
finding is answered at the end.
Mock: `docs/design/gcs_handheld_layout_mock.html` (static, drawn over
screenshots of the running app). It shows the collapsed list rising from the bottom,
as approved; the list now grows down from the top of its box (Collapsed list box, below).

## Problem (measured)

Running app on origin/main d5394e80, 3 SITL UAVs, headless Chrome, confirm
rounds injected into the page only (`.logs/handheld/baseline`, `geometry.json`).

| Viewport | Device | Map pane | E-STOP x | START MISSION y | Fold |
|---|---|---|---|---|---|
| 375 x 812 | phone | 55 x 716 | 389-475, **off-screen** | 657 | 764 |
| 695 x 812 | | 375 x 716 | 487-583 | 657 | 764 |
| 960 x 448 | UniRC density 320, Chrome | 640 x 352 | 752-848 | 657, **257 px below** | 400 |
| 960 x 600 | UniRC density 320, full screen | 640 x 504 | 752-848 | 657, **105 px below** | 552 |
| 1280 x 648 | UniRC density 240, Chrome | 960 x 552 | 1072-1168 | 657, **57 px below** | 600 |
| 1280 x 800 | UniRC density 240, full screen | 960 x 704 | 1072-1168 | 657 | 752 |
| 800 x 1128 | UniRC density 240, portrait | 480 x 1032 | 592-688 | 657 | 1080 |
| 1920 x 1080 | desktop | 1600 x 984 | 1712-1808 | 657 | 1032 |

- **E-STOP is unreachable at 375 px**, and so are fullscreen and settings. The
  top bar has no shrink rule, and the root clips it (`overflow: hidden`), so
  nothing can scroll to them. Planning is worse (E-STOP x 474-559): the
  PLANNING chip pushes it further out.
- The panel is 320 px with `flexShrink: 0` (`App.jsx`, the host of
  `ConfirmCardList`), so the map gets what is left. There is no breakpoint.
- At 375 px, in planning, the bottom bar's stats wrap and clip, and Upload
  sits on top of them.
- The RC confirm list at 375 px is 320 px wide and 182 px from the right, so
  its left 127 px is clipped.
- Pending cards are mounted only in the panel, and a card owns its auto-fire
  (`onAutoFire`, `TaskConfirmCard`). So a collapsed panel must keep the list
  mounted, or a round would never fire its auto-approve or auto-deny.

## Options

**A. Collapsible panel, drawer on narrow screens (recommended).** The panel
keeps its 320 px design (owner ruling 2026-10-20, `gcs_video_streams.md`).
It has three states:

- docked: exactly today's layout;
- overlay: the same panel over the map, the map does not resize;
- collapsed: the map takes the full width.

**B. Stacked panes below the breakpoint.** Map above, panel below. Each gets
about half the height on a phone, the panel needs a wider layout (a second
design to keep), the vehicle window's contract loses its 716 px pane, and it
does nothing for the UniRC. Not recommended.

## Design (option A)

**Rule.** `utils/appLayout.js`, pure, from the viewport width and one flag:

```
DOCKED_MIN_VIEWPORT_PX = sidebarWidth + 375 = 695
open   and width >= 695: 'docked'
open   and width <  695: 'overlay'
not open:                'collapsed'
```

375 is the map pane the vehicle window's layout contract assumes. `open` starts
as width >= 695 on each load, and nothing persists, so a reload never hides the
panel on a wide screen. Two transitions only:

- the toggle flips it;
- crossing up to 695 or wider sets it to open (a phone rotated from portrait
  docks, as a wide screen does by default).

Crossing down keeps it, so **a resize or rotation never hides the panel**: only
the operator's own tap does. View camera on a card in the drawer also closes the
drawer, through the same toggle: the picture opens under the drawer, and that tap
asked to see it. The width comes from the root element through the
existing `useElementSize`, starting from `window.innerWidth` so that Cesium never
mounts in a 55 px pane.

**Toggle.** One icon button at the right end of the top bar, above the panel,
at every width, with `aria-expanded` and a pending-count badge
(`isPending` entries, the same set `pendingSummary` counts). Hidden in RC,
where the panel is already gone. Its place stays empty there, so E-STOP does not
move as RC starts or ends, which can happen with no tap (its UAV lost, phase change).
The toggle takes 48 px at the right end, so at 695 px and wider E-STOP and the controls to
its right sit 48 px left of origin/main (E-STOP x 1712 to 1664 at 1920 x 1080, 752 to 704 at
960 x 448, 487 to 439 at 695 x 812). The map pane, Reset view and START MISSION do not move.

**Never remount.** `ConfirmCardList` stays the same element in all three
states, only its `style` changes, so a card's auto-fire and the still survive
a toggle. The sidebars stay mounted too, in a wrapper with `display: none`
while collapsed: their form state survives. (Entering RC still swaps hosts, as
it does today.)

| State | List host | z-index | Sidebar child |
|---|---|---|---|
| docked | today's style | none | shown |
| overlay | absolute, right 0, top 0, bottom 0, width 320, max-width 100% | 30 | shown |
| collapsed, any round | absolute over the map, box below | 11 | hidden |
| collapsed, no round | `display: none` | | hidden |

The hosts sit in the map row, which becomes `position: relative`. No transform
or filter is used on them, so a `position: fixed` modal inside still means the
viewport. No animation, in line with the vehicle window.

- Overlay at 30 is above the map controls (10), and under the notification
  banner (50) and the top bar's menus (1000). While the drawer is open, it
  covers what lies under it, including the controls' menus; a tap on the
  toggle closes it.
- Collapsed at 11 is **above** the map controls and their labels and menus (10)
  and the vehicle window (9). The rule is: **a round outranks the map**. Approve,
  Deny and Cancel task are never under anything but the toast. The list is
  clear of the fixed controls by its box; the fence-source label and an open
  menu are what it can cover, only where it reaches them, and only until the
  round is decided. A decided-only list is a 36 px row per round, with its margin.
- An open launch confirmation closes when the panel collapses (`MonitoringSidebar`
  gets a `hidden` prop). A backdrop stops a tap on the toggle, but not the keyboard
  (`ConfirmModal` does not trap focus), and reopening must not bring back an
  unfinished confirmation. Form data and pending rounds stay.

**Collapsed list box.** A pure function beside the rule, tested for overlaps
in the style of `vehicleWindowLayout.test.js`:

- left 12;
- width = min(320, pane - 2 x 12 - 32 - 12): clear of the 32 px column on the
  right (reset view, planning toolbar). 307 px at 375, drawn at that real
  width in the mock;
- top: under the top controls. When the vehicle window is shown at
  normal size and cannot sit beside the list (pane < 12 + list + 12 + window
  + 12), under the window instead: under its one-line bar and a whole 16:9 picture.
  If less than a named minimum (160 px, a chip, a decided row and the start of a
  card) then lies between there and the box's end, the top falls back to the
  top controls: the cards win over the picture. A maximized window also falls back;
- height: content, growing down, and ending 56 px above the pane's bottom, level with
  the reset view's reserve. It does not read the credit line, whose height changes with
  no tap. The line sits 3 px above the pane's bottom, so 56 px clears a line up to 44 px tall by
  9 px (measured: 28 px tall at 360 and 375 px wide, 812 x 375 and 960 x 448, clear by 25 px; 46 px
  on a 320 px pane, clear by 7 px). A single decided row takes 36 px with its margin and hides
  almost nothing.

It grows down like the docked list: a row arriving or leaving moves only the rows after
it, as the docked list does (Limits). As first approved it rose from the bottom: a new
round then lifted every row, and put its own Approve and Deny under a finger aimed at the
old ones, with no confirmation (review of the finished diff, below). Where the two cannot
sit side by side, the top moves when the operator opens, closes, maximizes or restores the
window. A warning row and the credit line come with no tap during a flight, so neither
reaches the box: the window grows under the list. What still moves the top with no tap is
under Limits.

So on a phone the operator sees the video above the card, and on a wide screen
the two sit side by side. In planning the display toggles' row stays clear above
the list. The vehicle window's UAV menu stays inside the window's own height for
the 3-UAV fleet; a larger fleet's menu can run under the list.

**The notification toast.** It sits at bottom 60 (3 s, no taps), where the list can
reach it and a card's Approve and Deny can lie under it. While the collapsed list
shows, it moves to top 12, over the top controls, which still take taps through it;
the list starts below them. Its text wraps instead of being clipped on a narrow
pane (`width: max-content`, `max-width: calc(100% - 24px)`), and a wrapped one can reach
the list's top for its 3 s (Limits).

**Short boxes: Cancel task is never clipped.** Today the pinned part cannot
shrink and the host clips, so a box shorter than the pinned rows loses Cancel
task. Measured from the code: RC at 812 x 375 leaves 100 px, and three decided
rows need 123 px. `ConfirmCardList` gets a host prop `heightPx` (the box's
height). The pinned part gets `overflow-y: auto` and a `max-height` of
`heightPx - 48` (`SCROLL_MIN_PX`, a control row and its padding) while the
scrolling part has content, so its cards, sidebar and footer keep room. With nothing
to scroll (no card or still, and the sidebar absent or hidden) the rows get the
whole box.
The pinned part only yields what the scrolling part needs, so for the 3-UAV demo
fleet nothing changes at any UniRC size: RC at 960 x 448 leaves 173 px, and a chip
and two decided rows need 121.

**Sticky pre-launch footer.** A sticky element only moves inside its containing
block, and the sidebar's block starts below the pending cards. So the footer is
an empty slot `<div>`, a direct child of the list's scrolling part after the
sidebars: `display: contents`, or `none` while collapsed. App passes the slot
element to `MonitoringSidebar`, which renders the phase's action into it through
a React portal, so the action keeps its state and hooks in `ActiveMonitoring`. The
slot has no box, so the action's wrapper, `position: sticky; bottom: 0`, has the
whole scroller as its containing block and stays in view over the cards. Empty,
the slot takes no height. No `ConfirmCardList` change for this. The wrapper has
the panel's background, `padding: 0 16px 8px` and `margin-top: -16px`: it takes
back the sidebar's 16 px bottom padding and keeps the padded width, and the abort
row (47 px) fits the 48 px the scrolling part keeps. When the content fits, it then
sits right after the content, as today (START MISSION stays at y 657 and 287 px
wide at 1920 x 1080, measured).

**A timed round outranks the start action, never the abort.** At 960 x 448 the
pinned START MISSION covered a pending card's Approve and Deny, and a card decides
by itself when its time ends. So while any round exists, pending or decided, the
start action is not pinned: it lies after the sidebar, where it is today. It is
not pinned again the moment a card is decided, because it would then come back
under the finger that just tapped Approve, and a short click on START MISSION
starts the mission. The abort stays pinned in every state. The footer holds:

- bungee: the pitot acknowledgement and `ForceStartButton` (START MISSION), or
  the download or start planning button when there is no GCS plan;
- container: the pitot acknowledgement and `ForceStartButton` (HOLD TO LAUNCH /
  START), or the same button when there is no GCS plan. The `LaunchPanel` list
  stays in the flow;
- container launching: the abort button (decision 7), which `LaunchPanel` renders
  into the slot. It is then in reach also while a pending card from an airborne
  UAV fills the panel (`CONTAINER_LAUNCHING` outranks `IN_FLIGHT` in a staggered
  launch).

Restart mission (in flight) stays in the flow: it has no confirmation, and it
should not sit one stray tap away from the map.

**Holds cancel when their button goes away.** `useLongPress` runs on animation
frames and cancels on release, disable or unmount. It has no `onTouchCancel`,
and it keeps running if an ancestor turns `display: none`. So: add
`onTouchCancel: cancel` (a system gesture no longer completes a hold), and at each
frame cancel if the pressed element had a layout box at the start and has none
now. A second finger on the toggle therefore cannot complete a launch hold
unseen. A hold also ends when the finger that pressed is no longer on its button, whichever
moved: a round arriving, a scroll, or the finger. The hook follows that finger by its
identifier (`changedTouches`), since a thumb may already rest on a stick, and allows
Android's 8 px touch slop around the button, as Android's own views do. A key hold has no
press point. New for touch: on origin/main a finger could slide off a button and still
complete its hold; a mouse could not. This covers START, LAUNCH on a UAV card, the mode
buttons and Approve holds, in one place.

**Top bar.** At every width: the left group gets `minWidth: 0` and clips, and
the right group never shrinks, so nothing can push E-STOP out again. Below 695
px: padding 8, gaps 6, no "GCS" label or branch badge, the PLANNING chip
replaces the logo, and the connection button shows the dot and the count (full
text in its title). The sum at 375 px is 8 + 80 + 40 + 86 + 3 x 32 + 5 x 6 + 8
= 348, so it fits; a longer translated chip is what clips. The connection panel
drops from the top bar's right edge (`position: fixed`, right 8) below 695 px,
as its button no longer has room to its left.

**Bottom bar.** At every width: stats on one line, their strip scrolls
sideways, the buttons never shrink. Upload progress items keep their width in
that strip too. Upload stays reachable, and no stat is lost.

**RC on a phone.** The RC list stays where it is (right of the flight-mode
column, left of the right stick) and only narrows as far as 260 px. With less
room than that between the column and the stick, it moves above the sticks: left
just past the flight-mode column, right 12, bottom above the sticks, and top below
the room kept for the "View held" banner on three lines (top 174), while that banner spans
from the column to the right margin. The room stays with or without a banner, so a banner
coming, going or wrapping does not move Approve and Deny. In that place its z-index is 11: it lies over the altitude tape, which is
at the right edge there, so a card's labels stay readable. The flight-mode column
goes from z-index 10 to 12 only while that raised list shows, so its "Other" menu, a
flight command, opens over the list. Everywhere else it stays at 10, under the speed
tape as today: at 960 x 448 "Other" lies on the airspeed readout, the only airspeed
number in RC. The HUD's warning strip (bottom 60) stays clear, as the list
ends above the sticks. Otherwise RC stays as it is. A selected UAV with held video and HUD warnings is part of the live check.

## Owner decisions

1. **Target:** the UniRC in landscape (960 to 1280 CSS px) is primary, as the
   demo comes first. Phones get one breakpoint, not a separate UI.
   (Recommended.)
2. **Layout:** option A with the drawer below 695 px, not stacked panes.
   (Recommended.)
3. **Manual toggle on wide screens:** yes, one toggle in the top bar, expanded
   by default at 695 px and wider. (Recommended.)
4. **While collapsed:** E-STOP in the top bar, pending cards over the map through
   `ConfirmCardList` with Approve, Deny and Cancel task, and a pending count on
   the toggle. UAV status cards are one tap away, in the drawer. (Recommended.)
5. **UniRC fold:** the sticky footer above. (Recommended. The E-STOP session
   did not fold this in; the measurements show all UniRC landscape sizes except
   full screen at density 240 need it.) Refined while building, after the owner's
   approval: while a round exists the start action is not pinned, see above.
6. **Density and browser are unknown:** test 960 x 600 and 1280 x 800, confirm
   on the device later (todo 94). (Recommended.)
7. **Container abort in the footer (new, not in the brief):** during a container
   launch, the abort button is in `LaunchPanel`, after the UAV cards, so it has
   the same fold problem as START. (Recommended, safety reach.)

## Limits

- UAV status, the phase and fault text are not visible while collapsed on a
  phone; only the top bar's count, the toast and the cards are. That is the
  cost of a full-width map. While collapsed, the abort button is one tap further
  away, in the drawer; E-STOP stays in the top bar.
- In RC under 542 px wide (column 100 + list 260 + stick 182), an open round's list
  covers the altitude tape until it is decided. The UniRC is not affected: at its narrowest,
  600 px, the list narrows to 318 px between the mode column and the stick; from 602 px it is as
  today.
- In RC on a pane from 542 to about 730 px wide (a phone in landscape, UniRC density
  320 in portrait), while the list is open the "View held" banner has little or no
  width between the mode column and the list. Measured at 600 x 808 with a round: the banner is
  26 px wide, on 9 lines, every word cut, behind the list. From 602 to 730 px by calculation: under
  about 640 px the warning that the picture is frozen cannot be read until the round is decided.
  As on origin/main, by reading. Todo 168.
- In RC under a 542 px pane the raised list keeps room for three banner lines with or without
  a banner, so it has 18 px less height than it would with room for two: 215 px at 375 x 667,
  188 px at 360 x 640, 116 px at 320 x 568, where Approve, Deny and a second Cancel task can each
  need a drag and are not all in view at once. Three lines is the most measured at 320, 360 and
  375 px wide, held 10 to 12 s: Armenian "Telemetry disconnected", 8 px above the list; English
  takes at most two. A larger text scale or a long counter could add one, which would lie 10 px
  under the list and not move it. Only a rotation or resize moves this list: crossing 542 px takes
  its top between 174 and 94.
- On a phone with the vehicle window open, a warning row makes the window longer under a
  collapsed list. Every row and its buttons stay above the list while it sits under the
  window. One one-line row (29 px measured) puts 17 px of the picture's bottom under the
  list while a round shows (and a synthetic picture's credit line), two rows about 46 px, the most that
  come with a picture, and each wrapped line about 16 px more. Video that is lost or failed
  removes the normal picture, so its rows and buttons have nothing to push under the list.
- On a pane too narrow for both and under 505 px tall (viewport about 601, at 360 and 375 px wide
  with the top controls on one row; a second row adds 38 px, and a 320 px pane starts from 492, so
  530 with the two rows it always has), the list starts under the top controls
  and lies over an open window while a round shows, over its bar, its warning rows and their
  buttons (Reconnect video, Dismiss, Retry). That is the approved "the cards win over the
  picture"; they return as the rows leave.
- New with this change, on narrow panes: these move the collapsed list's top with no tap, and
  origin/main's lists follow none of them. The vehicle window closing itself on an empty fleet
  (289 to 56 at 375 x 812; no UAV is connected then, and a lost link keeps its entry). A plan upload
  ending with the window open. The top controls rewrapping on a plan label change, with the window
  open or closed (38 px; of 320, 360 and 375 px panes in English and Armenian, only English at 360 px
  rewraps: "Start Planning" takes two rows, "Edit Plan" one). A rotation, or a browser bar resizing the pane, which can also cross the
  boundary above. A tap at that moment can land on another row. Warning rows, the credit line and
  the "View held" banner move nothing. Todo 171.
- A row arriving or leaving moves the rows after it, docked and collapsed, as on
  origin/main: the chip by 34 px, a decided row by 36 px, a card by its height; a still arriving,
  a card's header rewrapping or View camera coming and going move what follows them too (a tap on
  View camera shortens its own card by 23 px at 375 px, as the note beside it stops wrapping). A tap at
  that moment can land on the next row: another UAV's one-tap Cancel task when the row above it
  ends, or another card's Approve or Deny when a round with an earlier deadline arrives. New with
  this change, on a box short enough for the pinned rows to scroll: with them scrolled down, a row
  leaving or the chip going lets the scroll clamp, which moves the rows above too (calculated, not
  measured: RC at 320 x 568, three approved rows, the last row's still open and that UAV disarming:
  55 px, or 36 px with the still on another row; the collapsed list at its 160 px minimum: 11 px).
  The owner chose a separate task on 2026-11-02, so no guard was added
  here. Todo 171.
- A list that first overflows gets a scrollbar with no tap, which narrows its cards and stills, so
  what follows rises about 6 px. Measured collapsed in Chrome, with one round and a second arriving
  with a later deadline: the scrollbar takes 10 px; at 960 x 600 the card 286 to 276 px, its still
  147.3 to 141.6 px; Approve and Deny y 390 to 384.3 at 375 x 812 and 397.3 to 391.6 at 960 x 600. The
  list's top and the order stay. By the rects, a tap at either button's old centre still lands on it.
  Not seen collapsed at 1920 x 1080; docked there, one round already overflows the panel. origin/main
  has the same scroller, scrollbar style and card, so the same by reading, and its baseline shows the
  narrowed widths with cards (START 277 against 287 px); the shift itself was not measured on a
  running main stack. A reserved gutter would change widths at every size, so it stays with todo 171.
- The collapsed box ends 56 px above the pane's bottom, which costs about 16 px of list with a
  28 px credit line, against an end that read the line's height, and clears a line up to 44 px by
  9 px (46 px on a 320 px pane: by 7 px). The list overlapped the line by 0 px at the seven sizes
  measured. A full list can lie on a taller one (80 px: 27 px) until the round is decided.
- A wrapped toast can lie on the collapsed list's top for its 3 s: at 375 px with the window closed,
  a three-line toast (y 60 to 123) lies 19 px on the list's top at y 104 and a two-line one about
  2 px; a one-line toast clears it by 15 px. It takes no taps, so what is under it still does.
- Collapsed by hand at 960 x 448, or on a phone in landscape, a timed card's Approve and
  Deny start below the end of the box: with one round and no decided row, 48 px at 960 x 448 and
  121 px at 812 x 375, and each decided row adds 36 px. They are inside it at 320 x 640, 360 x 740
  and 375 x 812. Docked at 960 x 448 they are inside with no decided row (y 335 to 366, the panel
  ends at 400) and end 2 px past it with one, then 36 px more per row, as on origin/main. The chip
  brings only the card's top into view. One finger drag reaches them, two with a decided row
  (measured); the chip, the countdown and the
  card's top stay visible. A tap on the toggle gives the larger box.
- While a round exists, START MISSION and HOLD TO LAUNCH need a panel scroll on a
  short screen, as they do today.
- When the last round leaves (a denied or cancelled row ends after 8 s, an approved one
  when its UAV disarms), the start
  action pins again with no operator action, over the panel's bottom. A tap at that
  moment can land on it, and START MISSION fires on a short click. On origin/main the
  same row leaving moves START MISSION up wherever it is on screen, so the hazard is
  not new, but it now also applies on short screens. No guard timer was added: the
  rest of the app has none.
- During a container launch on a screen under about 461 px tall (UniRC density 320 in
  Chrome, 960 x 448; phones in landscape), the pinned abort covers part of a round's Approve
  and Deny. It takes an airborne UAV raising a round during a staggered launch. Measured at
  960 x 448 with one round and no decided row: 4 px of each lie under the abort button and
  12 px under its row, and a tap at their centre works with no scroll. Each decided row above
  the card adds 36 px, so with one they lie under the abort's row until the panel is scrolled
  (with no decided row, one scripted panel scroll, scrollTop 178, brought them clear in the live
  check; with a decided row not run at this size), and the 461 px rises by 36 px; the card's
  top, its timer and the chip stay visible. The APK is not affected with up to three decided
  rows (measured at 960 x 600 in Chrome: Approve and Deny y 443 to 474 with three, 32 px above the
  abort's row), so with a fleet of up to four: 461 + 3 x 36 = 569 px against 600 at density 320, and
  800 at density 240. The owner accepted this on 2026-11-02: the abort stays pinned. Todo 167.
- The pinned rows scroll once they and their padding exceed the box less 48 px while anything
  can scroll: in RC at 960 x 448 a chip and three decided rows, in RC at 320 x 568 a chip and one.
  Cancel task is then one scroll away, not clipped.
- With the pitot acknowledgement shown (`auto_preflight_cal` on, a launch UAV with
  airspeed), it and the start action sit 8 px lower than today for bungee and 2 px for
  container: the sidebar's bottom padding now lies between two margins that used to
  collapse. Nothing else moves.
- Two faults of origin/main stay at 960 x 448 (UniRC density 320 in Chrome), and worse on
  shorter panes. In planning, Load and Save plan lie under Reset view (todo 169). In RC, the
  last row of "Other" is clipped, and the mode column lies on the left stick's upper-left
  part (todo 170).
- Settings and the readiness panel are modals: full screen inside the viewport at 375 x 812 on the
  final build. No failure, no todo.
- Planning, collapsed, with a round at 375 x 812: the list covers 64 % of the toolbar's status label
  ("Fence: read from UAVs", x 240 to 363) until the round is decided. All 9 toolbar buttons stay
  clear and on top. It is a label and the cards win over it. New with this change, on a phone only.
- Collapsed at 812 x 375, Approve starts 152 px below the box's end; scrolling the list reaches it.
- Touch size and gloves stay with todo 94. The emulator is not the device.

## Test plan

Vitest: the rule at 694 and 695 with the flag; the two transitions (a rotation
never hides); the collapsed list box and the RC box for overlaps at 375 x 812,
812 x 375 and 960 x 448, with the window shown and not; the host style per state;
`TopBar` with the toggle, badge and E-STOP when narrow; a `vehicleWindowLayout`
case for a full 375 x 812 viewport; a toggle does not remount a pending card; the
footer holds the action of each phase, the start action pinned only with no round
and the abort always; `heightPx` caps the pinned part; `useLongPress` cancels on `touchcancel` and when its
element loses its box or the finger that pressed is more than 8 px off it.

Live: 375 x 812, 695 x 812, 1920 x 1080 and the UniRC sizes per the brief: no
horizontal overflow, controls not clipped, the vehicle window in picker, viewing
and maximized states, E-STOP one tap away, cards with Approve, Deny and Cancel
task while collapsed, RC at 375 with a held view, no new `DeveloperError` on
collapse and expand compared with origin/main at the same size. UniRC emulator at
density 240 and 320, landscape and portrait, Chrome 133 and the APK on WebView 101.

## Codex round 1, findings and answers

1. *No height fallback for Cancel task:* holds (three rows need 123 px, RC at
   812 x 375 leaves 100). Added `heightPx` above, and stated the capacity.
2. *Pending cards can hide the container abort:* holds; the sticky containing
   block was the sidebar. Moved the footer to a slot directly in the scroller,
   through a portal.
3. *Mounted state keeps unsafe transient state:* holds for a running hold; a
   hidden button kept counting. Fixed in `useLongPress`, and removed the rotation
   path: the panel now hides only on the operator's tap. A modal's backdrop
   covers the toggle, so an open launch confirmation cannot be hidden.
4. *Interactive overlaps:* holds. The collapsed list drops to z-index 9, under
   the controls' menus; the toast moves above the list. Remaining: a larger
   fleet's UAV menu can run under the list.
5. *Narrow RC warning placement:* holds. The list now starts below the banner's
   row and the banner takes the width.
6. *The phone mock scales the card:* holds. Recaptured at a real 307 px width.

## Codex round 2, findings and answers

1. *The half-height cap regresses Cancel task at 960 x 448 in RC:* holds (86 px cap
   against 121 px). The cap is now `heightPx - 48`, which is 125 px there.
2. *A backdrop does not stop the keyboard:* holds. Collapsing now closes the launch
   confirmation.
3. *Lowering the z-index leaves collisions (fence label, altitude tape):* holds.
   Collapsed list and phone RC list are at 11, above them, with the rule "a round
   outranks the map" and the cost stated under Limits.
4. *The portal footer does not keep the geometry as written:* holds. Wrapper padding
   and margin are now specified, and the live check measures y 657 and 288 px.

## Codex round 3 (verdict: approve with listed changes), findings and answers

1. *The raised phone RC list covers the flight-mode "Other" menu:* holds. The column
   goes to z-index 12, so a flight command opens over the list.
2. *The height rule omits the sidebar and the abort footer:* holds. Room for the
   scrolling part is now kept in every state.

## Review of the finished diff, Codex round 1 (verdict: changes required) and the live check

1. *The height reservation hides a Cancel task with nothing to scroll:* holds. RC at
   812 x 375 has 100 px; two decided rows need 87 and now fit. The reservation applies
   only while the scrolling part has content.
2. *The raised RC list covers a wrapped "View held" banner:* holds, and the live check
   measured it (banner 50 px tall, 10 px under the list). `FpvViewOverlay` now reports
   the banner's measured height and the list sits 8 px below it.
3. Live check, not from Codex: at 375 px the flight-mode column hid "SYNTH" of the
   SYNTHETIC tag, and the left of the held banner with no round open. Under a 542 px
   pane both now start right of the column; the tag ellipsizes at its end.

## Codex round 2 of the finished diff (verdict: changes required)

1. *The pinned footer covers a timed card's Approve and Deny at 960 x 448, and scrolling
   spends the time left:* holds; `TaskConfirmCard` decides at expiry. While a round exists
   the start action is no longer pinned. The abort stays pinned: that is the one case left,
   under Limits and todo 167.

## Codex round 3 of the finished diff (verdict: changes required)

1. *The remaining abort overlap needs the owner's decision:* holds as a decision, not as a
   code fault. The owner, given both options on 2026-11-02, kept the abort pinned; recorded
   under Limits and in todo 167.
2. *A hold survives after its button moves off screen:* holds. A round arriving moved HOLD
   TO LAUNCH from under the finger and the hold went on. It also holds on origin/main for
   content in the flow. Fixed in `useLongPress` for every cause of movement, not with a
   pinning key for this one case.

The test reviewer read the same diff with no view of Codex's points:

- *My first hold fix took the first finger on the screen:* holds; reproduced in jsdom. A
  hold with a thumb resting on a stick never completed. It now takes `changedTouches`, with
  a test.
- *Test gaps (what counts as a round, Approve and Deny while collapsed, App wiring,
  `BottomBar`, z-order, the collapsed list's minimum, the banner's measured height, the
  toast, a still's share, backgrounds, and two tests that could not fail):* hold; tests
  added for each. With the position check removed, exactly the three new hold tests fail.
- *Eight seconds after a Deny, START MISSION pins itself:* holds, under Limits.

## Lens reviews of the finished diff

Three read-only reviewers, each with a second one trying to refute its findings. Every
finding below survived that. All are from code and computed numbers unless a live result
is named.

Operator actions:

1. *The collapsed list moved its controls under a finger, so a tap could hit another round's
   Approve or Deny, or another UAV's Cancel task:* holds, major, new with this change. Fixed:
   the list grows down from the top of its box, like the docked list. Its toast moved with it.
   What this did not remove is in Codex round 4, below.
2. *Collapsed at 960 x 448, a timed card's Approve and Deny start far below the box:* holds,
   minor. It takes the operator's own collapse at this one size, and one drag reaches them (two
   with a decided row).
   Under Limits.
3. *E-STOP moved 48 px as RC started or ended, which can happen with no tap:* holds, minor, new
   with this change. Fixed: the toggle's place stays empty in RC.

Layout:

1. *The mode column at z-index 12 hid the airspeed readout in RC at 960 x 448:* holds, major, a
   regression of this change (also found by the wide-screen reviewer). Fixed: 12 only while a
   raised list shows.
2. *The "View held" banner has no usable width while the RC list is open, pane 542 to about
   730 px:* holds, minor, as on origin/main. The note said its text wraps, which is wrong and
   is corrected under Limits. Not fixed here: moving the list when a banner appears would move
   Approve and Deny under a finger. Todo 168.
3. *Load and Save plan lie under Reset view at 960 x 448:* holds, minor, as on origin/main. Under
   Limits, todo 169.
4. *In RC the last row of "Other" is clipped, and the mode column lies on the left stick:* holds,
   minor, as on origin/main. Under Limits, todo 170.
5. *View camera in the drawer opens the window under the drawer, and the card says "Viewing"
   with no picture while its time runs:* holds, minor, new with the drawer. Fixed: the tap
   closes the drawer.
6. *Upload progress overlaps at 375 x 812:* holds, minor, as on origin/main, but the note claimed
   the strip scrolls. Fixed: the items keep their width.

Wide screens:

1. Same as layout 1.
2. *With the pitot acknowledgement shown it and START sit lower:* holds, minor, nothing clipped or
   covered. Stated under Limits.

## Codex round 4 of the finished diff (verdict: changes required)

1. *Top anchoring still lets a tap hit another UAV's control:* the mechanism holds, and this
   note's "rows stay where they are" was wrong; corrected above and in the test's title. With
   approved UAVs 7 and 8, UAV 7 disarming moves 8's Cancel task into 7's place, and a round
   with an earlier deadline goes above the cards already shown. It is not new: the docked list
   on origin/main has the same order and the same one-tap Cancel task, Approve and Deny. This
   change removed what it added, the bottom anchor. A guard would change the confirm flow at
   every width. The owner, given both options on 2026-11-02, chose a separate task: under
   Limits and todo 171.
2. *A hold checks where the finger pressed, not where it is:* holds; Codex reproduced a valid
   hold cancelling and a hold completing with the button gone from the finger. Fixed: the hook
   follows the pressing finger by its identifier, and a mouse by its moves, with an 8 px slop
   so a rolling finger does not end a hold. A finger sliding off a button now ends the hold on
   touch too. Undoing the tracking fails 6 tests, and the identifier, the slop and the changed
   finger each fail their own.
3. *The overlap assertions pass on invalid coordinates:* holds. Fixed: `rect` in both layout
   test files throws on a non-number. With a helper reading a missing field again, both files
   fail.

Codex also found the changed toast expectation in `test_confirm_card_hosts_js.py` justified.

## Codex round 5 of the finished diff (verdict: changes required)

1. *A warning row in the vehicle window moves the collapsed list, and another UAV's Cancel task
   with it:* holds, and new with this change: the docked list does not follow the window. At
   375 x 812 one row moved the list's top from 289 to 314, and UAV 8's Cancel task into UAV 7's
   row. Fixed: the top no longer reads the measured rows, and the window grows under the list
   (Limits). The minimum under the window is judged down to the reset view, as the credit line's
   height also changes with no tap. The raised RC list had the same fault when a banner wrapped:
   it now keeps room for two lines. Undoing each of these fails its test.
2. *A decided row takes 36 px, not 32, so 461 px holds only with no decided row:* holds. Corrected
   under Limits and in todo 167: it rises by 36 px per decided row. The devices affected stay the
   same: the APK at 960 x 600 stays clear up to three decided rows.

## Codex round 6 of the finished diff (verdict: changes required)

1. *The collapsed list still moves with no tap, where origin/main's lists do not:* holds in part,
   and new with this change. The causes that come routinely in flight are gone: warning rows, the
   credit line and the banner reach no box. Left, under Limits and in todo 171: the window closing
   itself on an empty fleet, a plan upload ending with the window open, the top controls rewrapping
   on a plan label, and a rotation or resize. The window's maximize state cannot flip by itself
   where the list sits under it, as the pane is at least 505 px tall there, and that would take its
   chrome and credits over 329 px. Not taken: keeping the window's place always. At 375 x 667 the
   box would be 226 px with the window closed, putting a timed card's Approve and Deny out of the
   box. Nor a higher minimum: phones in a browser would lose the video above the card.
2. *Room for two banner lines still moves the raised RC list on a third:* holds. The list now keeps
   room for three lines, at top 174, and App no longer measures the banner. The cost is under Limits.
3. *The credit line's height changes the pinned cap and moves the card scroller, and 160 px is no
   minimum:* holds. At 375 x 601, credits going from 80 to 26 px moved the scroller 45 px. The box no
   longer reads the credit line and ends 56 px above the pane's bottom, so 160 px is now a true
   minimum under the window. The cost is about 16 px of list with a 28 px credit line, and a credit
   line over 53 px can lie partly under a full list. A 240 px tall viewport, where the box reaches
   zero, is not a target.
4. *The fallback covers the window's warning actions, and Limits named an impossible combination:*
   holds. Both corrected under Limits: Camera renders its loss rows without the held row, Synthetic
   the reverse, and lost or failed video removes the normal picture (`CameraFeed`). "32 px" in the
   host rules is now 36 px. Codex confirmed the abort numbers.
5. *The tests can fail, but miss the composed transitions:* holds. `appLayout.test.js` now feeds the
   box through the real `vehicleWindowLayout`, with warning rows and credit lines from none to tall,
   on panes with and without room, and pins App's call to pass neither. Undoing each fix fails its
   test. App cannot render in jsdom, so the visibility and maximize transitions are the live check's.

## Codex round 7 of the finished diff (verdict: approve with listed changes)

Every listed change is to this note or a comment; no behaviour changed.

1. *A scrolled pinned part clamps when a row leaves, which moves the rows above it:* holds,
   calculated, and new with this change. Under Limits and in todo 171. Codex asked for no guard here,
   given the owner's deferral.
2. *The residual causes need broader conditions:* holds. The top controls rewrapping moves the list
   with the window closed too, and a rotation or resize moves the raised RC list at 542 px. Limits
   corrected.
3. *Several numbers need qualifying:* holds. The 505 px is 492 at 320 px wide and more with wrapped
   top controls; the row overlaps rest on a measured row, which the live check later put at 29 px; the
   pinned capacity is per host, not "about six". Limits corrected.
4. *Three banner lines are an estimate:* holds. Stated as one, with what a fourth line would do. The
   live check then measured three as the most.
5. *Stale descriptions:* holds for two test titles, the banner's height comment and the toast test's
   comment; fixed. On the still: opening one moves no row even on a short box, as the cap shrinks and
   the pinned part's scroll position stays. Closing one can, by the clamp, on the operator's own tap.
   So the list's comment now says "opening it", and the test, which opens one, stays as it is.
6. *The credit line's cost belongs under Limits too:* holds; added. Codex's RC heights match the note's.
7. *The empty fleet is right, the Armenian range an estimate:* holds. A closed WebSocket leaves the
   store's vehicles, and the backend's `active_ids` come from the vehicles it still holds, with a lost
   link kept as `link_ok` false, so a lost link does not empty the fleet. The range is gone from Limits.
8. *The tests can fail and use the real path, browser coverage is outstanding:* agreed; that is the
   live check.

## Live results on the final build

Running app, 3 SITL UAVs, confirm rounds injected into the page, bundle `index-BSeLJDEm.js`; numbers
and screenshots in `.logs/handheld/live/final`. The Limits above hold the numbers that bound a
limit.

- No horizontal overflow, E-STOP inside, 6 collapse and expand toggles with 0 console errors and 0
  `DeveloperError`, the box ending 56 px above the pane's bottom: 375 x 812, 695 x 812, 1920 x 1080,
  960 x 600, 960 x 448, 1280 x 800 and 1280 x 648.
- 1920 x 1080: START MISSION at y 657 and 287 px wide, as in the baseline.
- No round, sticky footer: START MISSION y 349-384 at 960 x 448, 501-536 at 960 x 600, 549-584 at
  1280 x 648; container START y 335-380 at 960 x 448.
- With rounds: Approve and Deny take the tap at their centres. When the last round clears, START
  returns (y 744 to 349 at 960 x 448). Countdowns run on through a toggle, with no remount.
- Abort at 960 x 448: as under Limits, clear of the button from a 453 px viewport and of its row
  from 461 px.
- 375 x 812 collapsed: the list's top holds at 289 in the pane in picker, viewing and with a warning
  row; closed and maximized 56. Approve stays at y 623 as a row comes and goes.
- Raised RC list: top 174 with no banner and with one, at every language, width and reason measured.
- Holds, real touch in the APK and in Chrome 133 at 960 x 448: a plain hold opens the launch
  confirmation, and it stays open after release; a round arriving during a hold ends it. Scripted
  touch: a finger drifting 12 px off ends the hold, 6 px does not; a second finger's hold completes
  while another moves (on the map in monitor, and on the on-screen stick with ARM in RC; no physical
  thumb).
- Emulator, START and E-STOP visible with no scroll, sticky holds, a round unpins, one collapse and
  expand with no error from it. Density 320 landscape: WebView 101 shell 960 x 443, APK 960 x 600,
  Chrome 133 960 x 448. Density 240 landscape: APK 1280 x 800, Chrome 133 1280 x 648, WebView 101
  1280 x 619. Portrait, Chrome 133 and WebView 101: 600 x 808 and 600 x 803, 800 x 1128 and
  800 x 1099. The APK has no portrait (sensorLandscape).
- APK at density 320: 6 taps on the toggle with no error; a round collapsed gives a list at x 12, y 104,
  320 x 347, Approve y 395 to 426 on top. Container docked: START y 487 to 532, E-STOP visible.
- Collapsed, real taps with the POST held in the page: Approve, Deny and Cancel task send the right
  request at 375 x 812 and 960 x 448, and a card left to 0:00 posts deny after 30 s.
- Container launch: at 375 x 812 HOLD TO LAUNCH is sticky in the drawer, y 697 to 744; at 960 x 600 the
  abort's row is y 506 to 552, clear of Approve and Deny with no and with three decided rows.
- 6 toggles with the window viewing and 6 maximized, at 960 x 448 and 1280 x 800: the window stays
  inside, no error.
- Armenian at 375 x 812 and 960 x 448: no overflow, E-STOP inside, START not clipped.
- View camera at 375 x 812: the drawer closes, the window opens at y 104 to 325 with the card 12 px
  under it, and the countdown runs on. With `simulation.video_test_source`, the picture plays from
  the frontend port; from the backend-served page it is blocked (todo 173).
- With a mission on the vehicles the dock's D marker draws (template match 0.92 against the baseline's
  icon).
- A mission upload failing in these runs was not this change: todo 172.
- Not run: RC on a pane from 602 to 730 px; a round raised by NavPy; a panel scroll at 960 x 448 with
  a decided row; the device and a physical thumb (todo 94). Armenian only at the two sizes above.
