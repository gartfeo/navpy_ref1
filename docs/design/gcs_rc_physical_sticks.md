# GCS RC mode on a handheld with physical sticks

Status: agreed with the owner 2026-11-02. The drawn sticks stay; the mode buttons become
one mode picker; the HUD tapes move off the mode column and the sticks.

## Problem

On the SIYI UniRC 10 Pro (device check 2026-11-02, 853 x 533 CSS px, map pane 853 x 437)
RC mode draws two on-screen sticks, and the HUD's speed tape draws over the mode column
(22 m/s over DISARM, ladder labels over GUIDED and RTL) and over the left stick; the
altitude tape draws over the right stick. The owner asked to drop the drawn sticks when
the handheld's own sticks are connected to the UAV, and otherwise to treat them as not
connected.

## Why the drawn sticks stay

The GCS cannot tell that the physical sticks reach a given UAV:

- They never reach the GCS. Android lists no joystick or gamepad device (`getevent -p`:
  touch panel, keys, jacks only). Per SIYI's manual the sticks travel over SIYI's own radio
  to the one paired air unit, which outputs S.BUS/PWM to that aircraft's flight controller.
- `SYS_STATUS` RC-receiver health does not say which transmitter feeds the receiver. SITL
  reports it healthy with fixed channels (1500/1500/1000/1500), and so would a real
  receiver bound to another transmitter or holding failsafe values.
- A "healthy and a stick moved" rule on `RC_CHANNELS` was rejected in review (Codex,
  2026-11-02): ArduPilot reports RC overrides in `RC_CHANNELS`, so the GCS's own
  `MANUAL_CONTROL` makes the sticks look moved; `RCMAP_*`, `RC_OPTIONS` and failsafe
  values make channels 1 to 4 an unreliable stick signal.

So the physical sticks count as not connected, the drawn sticks stay for every UAV, and
nothing in RC mode changes on that account.

## Mode picker

Owner ruling 2026-11-02: the column's AUTO, FBWA and RTL buttons and the "Other" menu
become one picker, RTL included (two taps instead of one, accepted).

- Column, top to bottom: RC, the picker, ARM or DISARM, then the pre-arm warnings and the
  hold hint. Buttons stay 80 x 32 px. The warnings and the hint wrap at 80 px (today up to
  120), so the column never widens and nothing of it passes x 92.
- The picker shows the current mode with "▸" ("◂" while open), in the accent colour when
  the mode is one of the list's. An unknown mode shows its name; no mode shows "Mode".
- A tap opens the list to the right of the picker, top-aligned with it, as "Other" does
  today: the ten modes, AUTO, FBWA, RTL first, then MANUAL, LOITER, GUIDED, STABILIZE,
  CIRCLE, CRUISE, FBWB (`PICKER_MODES`), in a fixed order. Rows are 32 px tall, like the
  buttons ("Other" rows were 28); whether that suits a glove is the device check, todo 94.
  The list's border box is capped at the pane height less its pane-relative top and 12 px;
  rows do not shrink, and it scrolls beyond the cap: ten rows make a 348 px border box, and
  at 960 x 448 (pane 352) the list ends 12 px above the pane and scrolls to FBWB (todo
  223's clipped rows). While open it lies over the top of the left stick; a press there
  closes it and still moves the stick.
- Picking a mode sends it and closes the list; the current mode, or a second tap on the
  picker, closes it with nothing sent. Nothing else changes in how a mode is sent.
- The list never outlives what it was opened for: it closes when the selected UAV or its
  reported mode changes (so a pick can never go to another UAV, and the highlighted row is
  the mode the operator saw), on Escape, and on a pointer down outside the column. The
  outside press is not consumed, so a stick touch still moves the stick.
- No UAV selected: the picker is disabled, as the buttons are today.

## HUD tapes in RC mode

The tapes take no taps, but they must not hide controls.

**Speed tape**, left edge (x 12), below the column: from the column's bottom plus 8 px to
the top of the left stick less 8 px, at most 200 px. The column reports its height
(ResizeObserver inside the mounted column, removed on unmount), since the warnings and the
hint below ARM vary; App adds the column's 12 px inset. Until it is measured, and where
the room is under 80 px (many pre-arm warnings, or a short pane: at 960 x 448 it is
12 px), the speed tape goes right of the column (x 100) in the shared band below. On the
UniRC in flight (column bottom 153 with the wrapped hold hint): x 12, y 161, 94 px.

**Altitude tape**, and the speed tape's fallback, use the shared band:

- Vertically between the heading-tape row (`RC_FPV_TAG_TOP_PX`, 54) and the top of the
  sticks less a gap (`STICK_INSET_PX + STICK_SIZE_PX + 8` = 182 from the bottom). Height is
  the band, at most the 200 px today. Centred on the pane as today when that fits, else
  clamped into the band (pure helper `hudTapeBox`). At 853 x 437: top 55, height 200.
  A band under 80 px (a landscape phone, 640 x 360) keeps 80 px from the band's top and
  lies over the sticks: shorter, the speed readout and the GS line overlap.
- The altitude tape stays 12 px from the right edge.

Both tapes sit below the RC confirm list (9) and the first-person tag and held banner (8),
above the attitude (5); the open picker list (in the column, 10) lies over the speed
tape. The other HUD parts keep z-index 10. While "View held" shows, its banner, which says
the picture is frozen, may cover the speed tape (at 960 x 448 the fallback readout sits in
the banner's row); the banner wins, as it does over the confirm list.

Limit: on narrow panes the list covers a tape. From 542 to 601 px wide the narrowed list
starts at x 100 over a fallback speed tape; on a portrait phone the raised list can cover
both (todo 238). On the UniRC (853 px) the list starts at x 351 and covers neither.

## Tests

- Picker: shows the current mode (known, unknown, none), opens the ten modes in order,
  a pick sends the mode and closes, the current mode and a second picker tap send nothing,
  disabled with no UAV, list height capped with rows not shrinking; closes on a UAV
  change, a reported mode change (the list is tied in render to the UAV and mode it opened
  over), Escape and an outside press even when the press is stopped; column width stays 80
  with a long warning.
- Column height is reported on mount and when warnings change; the unmeasured state uses
  the fallback.
- `hudTapeBox` and the speed tape's box: tall pane, UniRC pane with no overlap with the
  column or a stick, the fallback right of the column, the 80 px minimum.
- The old placement is encoded in `tests/gcs/test_confirm_card_hosts_js.py` ("speed tape
  over the mode column") and `rcConfirmGeometry.test.jsx` (column at the tape's z-index);
  both change to the new order. `test_flight_modes_js.py`'s quick/other tests become
  picker tests.
- Device (UniRC, 2026-11-03, SITL, in flight): column ends at x 92, y 201 (page); speed
  tape x 12-82, y 209-303; altitude tape y 103-303; sticks from y 311. The list shows
  all ten rows (106-444), an outside tap and the current mode close it. Not checked on
  the device: ground state with pre-arm warnings, glove picking (owner).
- Browser at 960 x 448: list 101-388 in a pane ending at 400, scrolls to FBWB; speed
  tape beside the column (x 100, y 102-218). Escape closes the list.

## Later, not in scope

- Reading the sticks through SIYI's SDK (command 0x42, channel data) to forward them as
  `MANUAL_CONTROL` to the selected UAV. Reading them does not say which UAV the air unit
  drives. Not verified over UDP; it would put stick control on the telemetry link.
