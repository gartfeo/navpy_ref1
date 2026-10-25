# Radio budget: peer-telemetry filter and air load

Status: items 2-3 built 2026-10-20, then round 3 (image ids blocked, backlog
acceptance restated); items 1 and 4 are todos 97 and 98.
Codex rejected design v1: the hardware filter saved no air time, one autopilot
link leaked local pose to the radio, and an 8 kB/s gate passed a known
overload. Round 2 approved items 2-3 with changes, applied here. Found by
`deterministic_dev_sim.md` step 3 (line rate).

## Problem

The router sends each broadcast and each sniffer-visible packet a UAV puts on serial1 to
every other endpoint
(`router_model.py`; mavlink-router `d59ef94` with no filters, as `run_swarm.sh`
starts it). So each autopilot receives both peers' full telemetry and forwards
it on serial0 to its NavPy, which drops it (`inbound_router.py:111-118` keeps
its own system, NAVLINK and camera stream requests). Step-2 `hw` journal, seed
1, 175 s of sim time. The figures are message counts times each message's
full length, so upper bounds:

| Per UAV | kB/s | Line | Load |
|---|---|---|---|
| Sent on serial1: autopilot (comp 1) 2.9, companion (191) 0.6, camera (100) 0.37 | 3.9 | 5.76 (57600) | 68 % |
| Received on serial1 from the router | 7.8 | 5.76 | 135 % |
| NavPy's serial0 from its autopilot | 13.4 | 11.5 (115200) | 116 % |

With SITL's line rate on (`SIM_BAUDLIMIT_EN`) the serial1 input backlog grows
without bound (router to autopilot latency 528 to 1167 ticks, `hw` seed 2).
Wall runs carry the same load; nothing there limits it.

Two separate problems follow:

1. **Receive:** an autopilot and its NavPy get traffic they throw away. This
   overloads the autopilot's link and NavPy's link.
2. **Air:** by the upper-bound estimate, three UAVs send up to 11.7 kB/s against the
   RFD900x default air rate of 64 kbit/s (8 kB/s raw), before slot or framing
   overhead. Item 3 measures it exactly.

## Radio facts

- RFD900x defaults: UART 57600 baud, air rate 64 kbit/s. In multipoint
  firmware the nodes share the air in time slots, and destination 255 broadcasts
  to every node. A radio puts every packet it receives on its UART; it cannot
  filter by MAVLink component. (Vendor listings and ArduPilot forum threads.
  Codex also cites the RFDesign multipoint manual V1.1, pp. 7-13: even sharing,
  usable throughput below the air rate, limited serial buffers, UART set apart
  from the air rate. Not checked here: no PDF text tool on this PC.)
- Usable throughput is not known; a bench test measures it.
- SITL models UART lines only, never the shared air.

## Decision

### 1. Hardware direction (later; its own design and review; todo 97)

- Autopilot traffic for NavPy stays on its own autopilot port at the local rates
  and never reaches the radio. A second autopilot port carries the radio rates to
  a mavlink-router on the Orin.
- The radio connects to the Orin at a UART baud above the air rate.
- That router keeps peer autopilot and camera traffic (components 1 and 100) away
  from the autopilot and NavPy. This saves no air time and no radio UART
  capacity.
- Open questions:
  - which connection receives NAVLINK. A peer's component-191 broadcast could
    reach NavPy both from the router and through the autopilot's own forwarding;
  - autopilot forwarding settings and endpoint identities;
  - sniffer placement and discovery;
  - the image-transfer messages;
  - the board's real SR index for each port;
  - the GCS radio's UART baud and flow control.

### 2. Sim now: the same filter in the router

- SITL serial1 stands for TELEM1. The central router stands for the air plus
  every Orin's router. Each SITL endpoint gets `BlockSrcCompOut = 1,100` and
  `BlockMsgIdOut = 130,131`; the backend gets no filter. The packets each
  autopilot receives are the same as in item 1. Queues, latency and air time
  are not the same; the air is reported (item 3), not modelled.
- `BlockMsgIdOut = 130,131` (DATA_TRANSMISSION_HANDSHAKE, ENCAPSULATED_DATA):
  the companion's confirmation image is a broadcast meant for the GCS. One
  transfer is 35.9 kB of frames (handshake and 135 chunks) in 2.7 s, 13.3 kB/s
  against the 5.76 kB/s line, so it floods each peer's serial1 (`hw` seed 2).
  It still overruns the sender's own link: todo 100.
- In the sim, NavPy's NAVLINK still goes through its autopilot (serial0 to
  serial1), as today.
- What the filter passes, exactly:
  - companion traffic from component 191, so NAVLINK and also gimbal telemetry
    and STATUSTEXT, but not the image;
  - camera components other than 100;
  - GCS traffic (sysid 255, component 0).
  It blocks targeted messages from components 1 and 100 too, and ids outside
  the router's entry table: `read_msg` takes `msg_id` from the header for every
  packet, so the `UINT32_MAX` skip in `accept_msg` never applies
  (`endpoint.cpp:349,369,432`).
- Router model (`router_model.py`), following `accept_msg`:
  - the loop check first;
  - then `BlockMsgIdOut` and `BlockSrcCompOut`, the only filters this
    configuration uses (the model takes no other);
  - then target and sniffer rules.
  Each endpoint's result is accepted, filtered or rejected. A packet with at
  least one accepted endpoint is `routed`. A packet with none accepted and at
  least one filtered (the others rejected by the loop check or the target rules)
  ends as `router_filtered`, a new end state. A packet with all endpoints
  rejected stays `router_dropped`, mavlink-router's `msg_to_unknown`; a
  filtered endpoint keeps a packet out of that counter. Every consumer of end
  states (forwarding gates, journal checks) is updated; a NAVLINK packet
  `router_filtered` fails the forwarding gate like `router_dropped`.
- Parity: the WSL harness starts the binary with a config file holding the
  filters. Add cases for filtered broadcasts, filtered targeted messages, an
  unknown id from component 1 filtered, 191 traffic passing, a filtered endpoint
  before the sniffer, mixed loop, filter and target rejection, and the image
  ids from each companion (`filtered_image`). Compare bytes at every
  endpoint and the unknown-target counter. Keep the unfiltered cases.
- Wall: `run_swarm.sh` writes the SITL feed endpoints, each with the filter,
  to `$ARDUPILOT_DIR/router_<SWARM_OFFSET>.d/sitl_feeds.conf` per launch and
  starts the router with `-d` on that directory (`rfd900` profile unchanged).
  `-d` adds to the default `main.conf` instead of replacing it, as `-c` would.
  The sniffer and the Windows endpoints stay on the command line, since
  `wall_reap_wsl.py` finds the router by its monitor port. The file lives in
  the ArduPilot fork, whose deployed copy has long-standing uncommitted edits;
  its sha256 pin in `lockstep_topology.py` changes with it (now `38a51884…`,
  with the image block and the id allowlist). The new copy is
  staged as `run_swarm.sh.radio_budget` beside it and deployed at merge, since
  every chat's launches read the one deployed file.
- Expected effect: each SITL receives about 1.3 kB/s (23 % of 57600), and each
  NavPy's serial0 carries about 7 kB/s (61 % of 115200).

### 3. Air load report (no gate yet)

- `report.json` gets the air load in framed bytes: the byte ranges of the
  emitted packets (whole MAVLink frames), not counts times sizes. Reported apart
  from them: skipped bytes (failed CRC, no marker) and an incomplete frame at the
  end.
- For each node that transmits (each UAV's serial1 output, the backend's
  output): mean and max bytes per second over fixed 50-tick (1 s) windows of
  sim time, plus the network total. A tick's bytes count in the sim time that
  ends at its frame time; the parked frame at its own time (journaled since
  this item), drain ticks at theirs. A partial last window is scaled by its
  length. Setup (SITL boot to the first NavPy spawn: connect, parameter
  writes, mission uploads) is reported apart. Built: `eval_gcs_demo_air_load.py`,
  `report.json` `air_load`.
- Radio loss drops bytes before the router frames them, so offered load is
  measured only on runs with `uart_loss` off. A loss-on run reports what it
  observed after loss and says so.
- No pass or fail until the owner sets a usable budget from the bench test, so
  no threshold is invented.

### 4. Stream rates and air rate (owner decision, later; todo 98)

With the item-3 numbers: lower `SR1_*` (the GCS backend reads ATTITUDE,
GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS, GPS_RAW_INT, MISSION_CURRENT,
BATTERY_STATUS, EKF_STATUS_REPORT, RC_CHANNELS, HEARTBEAT and STATUSTEXT, and
none of the raw-sensor, AHRS, ESC, vibration, terrain or wind messages). Also
lower the companion's gimbal and camera status (published at 5 Hz,
`gimbal_telemetry_publisher.py:29`; the GCS calls them stale after 2 s, so
1 Hz risks gaps under loss). Or raise the air rate (shorter range). Wall launches
read the shared parameter file, not the eval overrides, so the rate change needs
its own way to reach them.

## Acceptance for items 2 and 3

- **Router model:** unit tests for the filter and its place after the
  loop check and before the target and sniffer rules, unknown ids, and the three end states, including mixed results. The
  parity fixture is recorded again against the binary, with the new cases.
- **Routing per message class:** model and parity cases for the GCS heartbeat
  to an autopilot, COMMAND_LONG and its COMMAND_ACK, PARAM_SET and
  PARAM_VALUE, a camera request with its reply and ack, and targeted and
  broadcast peer NAVLINK.
- **Run-level checks that already exist:**
  - mission upload with readback (`eval_gcs_demo_mission_upload.py`);
  - setup parameter writes and command-bound reads (they fail the run without
    their echo);
  - assignment chains and advert forwarding (fleet gates).
- **Backlog (round 3, replaces the latency-window rule):** on `hw` seed 2
  (line rate only), coordinator pending bytes stay 0 and every NAVLINK packet
  is delivered or explicitly lost. NAVLINK latency is measured in two halves
  (sender: emission to `router_copy`; receiver: `router_copy` to
  `navpy_delivered`) and end to end, from the journal by hand for now; a
  `report.json` field is todo 101. Firmware queues can grow while coordinator
  pending stays 0, so the run must also show, per UAV and half, image periods
  included: each burst drains and latency returns to its pre-burst range
  before the next one; comparable bursts do not add delay over the run; every
  packet is accounted for at the end. No numeric limit until the owner sets a
  traffic envelope (per UART, burst B plus rate r below the 5760 B/s line);
  the old rule against the line-off run also failed on normal bursts. Until
  shown, the backlog is "measured, acceptance unresolved", not passed.
- **Known failure:** seed 2 has wind, and every UAV misses its dock there
  (todo 77). A run that fails only on those reset markers is read as "radio
  acceptance passed; workflow failed with the known todo-77 signature". It
  stays failed, and any other diagnostic that differs from the line-off run
  must be explained.
- **Matrices:** `none`, `hw` and `stress` each give one digest and byte-equal
  artifacts, with new baselines.
- **Wall:** one wall eval run on the new `run_swarm.sh`, passing.

## Results (round 3, `7f613661`, `.logs/radio-budget-r3`)

- Parity 17 passed against the binary; `none` and `hw` matrices pass (one
  digest, 58 files equal each); wall eval passes on the `4a1c00af` launcher.
- **Backlog, `hw` seed 2:** pending 0 on every tick; NAVLINK packets not
  delivered are 5, with `router_copy` at ticks 9330-9333 and the run ending at 9336. Receiver half at most 6
  ticks per 5 s window on every UAV, image periods included. Sender half at
  most 174 ticks while a UAV's own image goes out (todo 100), back to 2-3 ticks
  within 10 s; in this run's windows, bursts do not add up. Line rate off: 2 ticks throughout. The
  run fails only on the todo-77 reset markers, as with the line rate off.
- **Not passing:** `stress` (one digest, 17 files equal, but every run fails
  UAV 3's parameter write: todo 103) and `hw` seed 10 (a broken frame cut UAVs 2
  and 3 off from the GCS at the router: todo 102). Both follow SITL byte loss
  (102 traced at the router, 103 inferred); neither is traced to items 2-3.

## Round 4 design: byte-loss fixes (todos 102, 103)

Owner direction (2026-10-20): the aircraft use a mesh radio that broadcasts,
not a central router, so the fix must not depend on the radio or the router
topology. Parameter writes get more attempts and fail fast on a dead link
(heartbeat), without treating heartbeats as lossless. Pipelined resend
rounds are todo 104.

How SITL loses bytes (`UARTDriver.cpp:283-293`; rounds 3-5 only, round 6
replaced it with the coordinator's radio loss): each UART write call drops
its last byte with probability `SIM_UART_LOSS` % × the call's size. ArduPilot
writes a frame as three calls, header, payload and CRC (`mavlink_helpers.h:
380-382`), so a frame breaks with about loss × its length. Reads lose
nothing. A receiver may join a broken frame to the next one, which gives a
frame with a random header. Measured on SITL outputs: breaks per frame match
loss × length (`stress` 0.11 against 0.105; `hw` seed 10 0.027 against
0.023), plus the garbage frames that follow.

### 102. Act only on frames whose CRC was checked

- Rule, for any radio: a node that learns routes or forwards may act only on
  a frame whose CRC it checked. A mesh radio forwards bytes and learns
  nothing, so it needs nothing. ArduPilot, NavPy and the GCS check the CRC of
  every frame they parse (crc_extra from their dialect). The one node that
  does not is mavlink-router for ids outside its table (`endpoint.cpp:413`),
  and it learns from them (`:246`).
- So every lossy router endpoint gets `AllowMsgIdIn` = the ids in the
  router's entry table (311, `router_entries.json`, extracted from the
  binary). The incoming filter runs before learning and routing (`:236-247`),
  and `read_msg` takes the id from the header, so the filter applies to
  every frame. A broken frame then either has a known id and fails the CRC,
  or has an unknown id and is dropped. A garbage frame passes only if a
  random id is known and its CRC matches by chance (about 2^-16 each).
- Lost legitimate traffic: only id 295, which every SITL sends but neither
  the router's table nor our pymavlink dialect knows, so nothing here reads
  it. All NAVLINK ids are in the table.
- Where: sim, each SITL feed endpoint (they carry the SITL byte loss); the
  GCS backend's link is lossless local UDP and stays as is. Wall:
  `run_swarm.sh` writes the key per feed (new pin). Hardware (item 1): the
  same key on the radio endpoint of each Orin router, after checking that
  every id the aircraft's components need is in that router build's table;
  a missing one goes into the router's dialect, never into the list
  unchecked. Not tied to sysids or topology.
- Not chosen: `AllowSrcSysIn = <own sysid>` per feed. It fixes the sim but
  depends on which sysids live behind an endpoint, which a mesh changes.
- Model: `EndpointFilter` gets `allow_msg_id_in`; `ReferenceRouter.route`
  applies it before learning; an incoming-filtered packet is not learned,
  has no destinations and ends `router_filtered`. Parity: the WSL harness
  writes the key; new cases: an unknown-id frame from 255/0 on a SITL feed,
  then a GCS packet to that UAV (must arrive), and a known-id frame on the
  same feed (passes). The source check stays: it checks emissions, before
  the router.

### 103. More attempts for GCS parameter exchanges, fast fail on a dead link

- QGroundControl resends a PARAM_SET twice after 1 s
  (`ParameterManager.h`, `kParamSetRetryCount = 2`): 3 attempts, as ours.
- In this SITL transport only the PARAM_VALUE echo is lossy (37 bytes); the
  request is a read for SITL. Estimated bound per attempt: the echo breaks,
  or a broken frame before it swallows it, about 2 × 37 × loss, 44 % at the
  `stress` top of 0.6 %. This is an estimate, not a measured exchange rate.
- Budget: `GCS_PARAM_ATTEMPTS = 12`, a provisional engineering budget from
  0.44^12 < 1e-4 (one GCS write or read failing less than 1 in 10 000 at the
  `stress` top, if attempts are independent). The 2 s wait per attempt
  stays. Each attempt logs its send and whether a matching echo came, and
  each exchange its outcome, so the runs give the consecutive-failure
  distribution and any false link-lost ends; matrix passes are regression
  evidence, not proof of the target.
- Scope: the GCS routes that write parameters (`PUT /params`) and read them
  by name (`/params/named`) pass the budget explicitly; the defaults stay 3.
  NavPy's own reads keep 3: one runs on the command path (todo 99).
- Dead link: the vehicle counts as alive while any valid frame from its
  autopilot (sysid N, component 1) arrived within `heartbeat_timeout_s`
  (5 s); the HEARTBEAT is one of them. A heartbeat alone would end a live
  exchange about 1 time in 1 000 at the `stress` top (21-byte frame, up to
  25 % lost, five in a row). Tens of frames per second make losing all of
  them for 5 s unlikely under independent loss, but a correlated outage can
  still do it: 5 s of silence means an inferred disconnection, not a dead
  vehicle, and the logs count these ends. The check runs
  after taking the write lock, before the first send and before each
  resend; a dead vehicle ends the exchange at most 5 s + one 2 s wait after
  its last frame.
- Deadline: both routes take `timeout_s` (default 55 s) and set one
  absolute deadline on entry, shared by every parameter. The lock wait and
  each echo wait are capped by the time left, and the deadline is checked
  again after taking the lock and before each send (each send still waits
  for the transport's send lock, which a send holds only while writing one
  frame). The deadline bounds server work, not the response: the caller keeps a margin above it for
  scheduling and delivery, and a write already sent can still take effect
  later. A client that disconnects early does not stop the work; the
  deadline bounds it. Per-parameter results: `true`, or
  `{"ok": false, "outcome": ..., "reason": ...}`. `unattempted` means
  nothing was sent (deadline or link lost before the first send);
  `unconfirmed` means sent without a matching echo, so it may be stored,
  with reason `no_echo`, `link_lost` or `deadline`. The cache is invalidated
  for every attempted write, as now. The frontend treats anything but
  `true` as failed (`useAasParams.js:276`). The eval passes `timeout_s` 5 s
  below its HTTP timeout. Lockstep binds the backend's clock to sim time, so
  there the budget counts sim seconds; the HTTP timeout counts wall seconds.
- Not chosen: lowering the `stress` loss (changes the test to pass); an eval
  re-PUT of failed writes (a retry in the gate, not in the product).

### Checks

- 102: model tests (unknown id from 255/0 not learned and later GCS packets
  delivered; known id passes; filter before learning), mutations (filter off,
  filter after learning); parity re-recorded with raw-byte cases: an
  unknown-id frame from 255/0 then a GCS packet to that UAV, a known id with a
  bad CRC, and legitimate traffic from two sysids on one feed.
- 103: tests for stale heartbeat with other autopilot frames (no abort), true
  silence (abort within the bound), live frames without echoes (all
  attempts, `unconfirmed`/`no_echo`), a late echo (counts), deadline expiry
  while waiting for the lock (`unattempted`) and after a send
  (`unconfirmed`/`deadline`), and the budget passed only by the GCS routes;
  mutations
  (attempts back to 3, no dead-link check, heartbeat-only liveness).
- Runs: `none`, `hw`, `stress` matrices, `hw` seeds 2 and 10, wall eval, full
  pytest, opt-in tests; the attempt-count distribution from the logs.

## Results (round 4, `.logs/radio-budget-r4`, `-r4c`)

- `none` and `hw` matrices pass (one digest each; `hw` rerun on a clean tree
  in `-r4c`). In `-r4` one `hw` cell failed at start ("slot is stopping",
  the registry said slot 42 was not active); cause not found, the rerun passed.
  Parity and opt-in tests pass (26); wall eval passes.
- **102:** `hw` seed 10 has no router drops and its confirmation goes through;
  every UAV reaches SNAP. It now fails only like seed 2 (todo-77 reset
  markers and the dock miss that follows), and seed 2 is unchanged from round 3.
- **103:** GCS parameter exchanges in all runs: 457 confirmed on the first send,
  11 on the second, none failed. `stress` now passes every parameter write.
- **Not passing:** `stress` fails later at one digest on mission transfers
  (todo 105).
- Full pytest: 13 failed, none from this change after `f04659eb`: the
  annotation guard (`delivery_demo.py`), the Codex agent sync (untracked
  `.codex/agents/*.toml`), the board detector, and the HITL bridge (Windows
  Application Control blocks the jsbsim DLL).

## Round 5 design: mission transfers (todo 105)

### Cause (`stress` plain-a, `.logs/radio-budget-r4`)

- Downloads lose item replies. A live `MISSION_ITEM_INT` answers its
  request within 2 ticks (40 ms); a lost one is a broken 84-byte frame on
  the SITL serial, or a broken frame before it (msgid 10, 261-byte random
  header) that swallows it. NavPy's three startup downloads sent 57 item
  requests and lost 11 replies (19 %; journal and `uav_*_navigation.log`).
- Each lost reply costs the downloader's full 10 s wait (log gaps of
  exactly 10 s), and 3 sends per item are too few. The GCS readbacks of
  6-item missions took 31, 71 and 41 s; the next `GET /mission` lost one
  item 3 times: 404 after 30.9 s. NavPy UAV 1 and 3 took 31 and 41 s; UAV 2
  had not finished when the stop came.
- NavPy downloads its mission inside the vehicle build, and nothing in the
  startup reads the stop. The operator stop came while NavPy 2 was still
  downloading: `HostFault` stop budget at tick 14702.
- Uploads passed: ArduPilot drives them, re-requesting a missing item every
  1 s (`MissionItemProtocol.cpp:409`) and abandoning after 8 s without one
  (`MissionItemProtocol.h:95`).

### Design

- **Timeouts as QGroundControl** (`PlanManager.h:59-62`, the mavlink.io
  mission protocol): wait `MISSION_ITEM_TIMEOUT_S = 0.25` for an item and
  `MISSION_ACK_TIMEOUT_S = 1.5` for a count, request or ACK; 1 + 5 retries.
  A resend after a late reply is harmless: requests are idempotent and the
  reply cursor is taken before the first send, so a late reply still counts.
- **Download steps are bounded exchanges.** `REQUEST_LIST` → `COUNT` and
  each `REQUEST_INT(seq)` → `ITEM(_INT)` run through the todo 103 loop
  (`run_exchange`, generalized to any reply type). One policy covers the
  transfer: its deadline and link check apply to every step.
- **Attempts per step:** default `MISSION_PROTOCOL_ATTEMPTS = 6` (19 % per
  send gives 5e-5 per item if sends fail independently). GCS routes pass
  `GCS_EXCHANGE_ATTEMPTS` (12) with the request deadline and `stop_on_silence`, as todo 103.
- **Upload:** `MISSION_COUNT` → first request is a bounded exchange. Then
  each wait takes the vehicle's next request, any seq, so a re-request of
  the last item is answered too, or its final ACK. A wait is cut by the
  deadline, checks the link and `cancelled` every 1.5 s, and ends after
  `UPLOAD_ABANDON_S = 8` without a request (ArduPilot has dropped the
  upload). A non-accepted ACK ends it at once.
- **Lost final ACK:** ArduPilot sends one ACK; an item after completion is
  answered `MISSION_ERROR` (`GCS_Common.cpp:985`), so it cannot be asked
  again. A failed GCS upload attempt therefore reads the mission back
  before clearing and uploading again (`waypoint_builder.py`). It counts as
  done only on an exact match: the count, and for every item but home
  frame, command, autocontinue, param1-4, x, y and z (floats compared as
  float32). Any difference, autopilot normalization included, falls back
  to clear and re-upload, as today, with one exception found in round 5:
  ArduPilot stores no frame for a command without a location and returns
  it as `MAV_FRAME_GLOBAL` (`AP_Mission.cpp`, `mission_cmd_to_mavlink_int`),
  so a `MAV_FRAME_MISSION` item (the skip `DO_JUMP`) reading back with
  frame 0 still matches; its other fields stay exact; the warn-only field check stays for
  confirmed uploads. Today a failed attempt re-uploads without a readback.
- **Stop:** `ExchangePolicy` gets `cancelled` (a predicate; reason
  `cancelled`), checked with the deadline and the link before each send
  and after each miss. Once the stop is requested, NavPy's vehicle cancels
  every parameter read and mission transfer it runs, for the rest of the
  process: reads return no value at once (not marked missing), transfers
  fail. Parameter writes are not cancelled, so shutdown restores
  (`WP_LOITER_RAD`, `SIM_SPEEDUP`) still run with their todo 103 bounds;
  reads after the stop, such as the `NavArgs.refresh` in controller
  cleanup, cost nothing. Composition checks the stop after the vehicle
  build and before `navigation.start()`, and the session checks it before
  starting vision and the controller; if set, nothing more starts, so no
  value read as cancelled is acted on. (A loop already running that reads
  an uncached parameter after the stop gets its default until cleanup
  joins it.) Either way the session closes what
  was built and acknowledges. A stop thus waits for at most one exchange
  in flight plus the restores. The stdin stop is created before the
  vehicle build for the same reason.
- **GCS request deadline:** `GET /mission` takes `timeout_s` (default 85
  s; the eval's HTTP timeout is 90 s); `POST /upload` sets one deadline for
  the request (225 s; the eval's is 240 s); the connect and launch probes
  use the 85 s default. The mission lock waits are capped by it too (GET,
  probe, upload), and a request that times out on the lock fails without
  touching the vehicle. Lockstep counts these in sim time.
- **Logs:** each step that needed more than one send or failed logs its
  outcome (as todo 103); a GCS transfer logs every send.
- Not chosen: longer waits or a lower `stress` loss (hide the cause);
  pipelined resends (todo 104); bounding the 30 s startup heartbeat wait
  (not seen failing).

### Checks

- Tests: a lost item reply resent after 0.25 s; attempts per step (6
  default, 12 GCS); deadline across steps; link silence ends a GCS
  transfer; `cancelled` ends a download and an upload within one wait;
  upload answers a re-request of the last item, ends on an error ACK and at
  the abandon bound; a lost final ACK is verified by readback, and a
  readback differing only in `DO_JUMP` param2 still re-uploads; stop during
  startup (`NavArgs` reads with replies missing) acknowledges through
  cleanup without starting the controller; reads after the stop return at
  once; writes after the stop still run; GCS routes pass the policy and
  cap the lock wait. Mutations: the old waits, 3 attempts, no cancel check, writes
  cancelled, warn-only readback.
- Runs: `none`, `hw`, `stress` matrices; full pytest.

### Review additions

- `POST /upload` starts no reconnect, `RTL_ALTITUDE` write, fence clear or
  fence parameter write past its deadline; the fence's lock wait is bounded,
  and a reconnect waits for the link only until the deadline. A lock won
  after the deadline is given back unused. An error ACK to `MISSION_COUNT`
  ends an upload at once.
- Residual overrun: a parameter write or a reconnect already started may
  end past the deadline (one write, about 6 s; building the reconnected
  vehicle is not bounded). The eval's HTTP timeout is 15 s above the budget.
- `VehicleMav` lost its `heartbeat_hz` argument (a constant now, 1 Hz as
  every caller passed) to stay within the constructor guard.

## Results (round 5, `.logs/radio-budget-r5`, `-r5b`, `-r5c`)

- `none` and `hw` matrices pass (one digest each; `none` `61727653…`,
  `hw` `84da3889…`). Opt-in tests pass (26). Wall eval passes when run
  alone (`-r5b`); right after the matrices it failed on approach quality
  (a 55.5° roll step on UAV 1, UAV 3 SNAP at 0.79 m without `DOCKED`),
  with no mission-transfer link.
- **105:** in `stress` NavPy's startup downloads take about 2 s (were
  30-70 s), no stop overruns, and the GCS readbacks pass. One GCS upload
  lost its final ACK; its readback differed only in the skip `DO_JUMP`'s
  frame (above); with that fixed it counts as stored (`-r5c`).
- **Not passing:** `stress` fails at one digest (`812fb485…`), later, at
  the swarm task assignment: todo 114.
- Full pytest: 8 failed, none from this change: the Codex agent sync
  (untracked `.codex/agents/*.toml`) and the board detector.

## Round 6: radio-only loss and repeat until answered (todo 114)

Design: `swarm-task-assignment-ack.md` "Lossy link". The coordinator now
loses radio bytes (each SITL's serial1 and the backend uplink), not SITL;
serial0 is clean, and each broken packet is journaled. The swarm handover
repeats every 0.5 s until answered.

## Results (round 6, `.logs/radio-budget-r6`)

- `none` and `hw` matrices pass (one digest each; `none` `ca654c38…`,
  `hw` `474a62e5…`). Opt-in tests pass (26). Full pytest: 8 failed, the
  same as round 5 (Codex agent sync, board detector).
- **114:** `stress` seed 1 (0.30 % per byte) gets every UAV through the
  swarm handover and the operator approvals, at one digest (`87fc996a…`).
- **Not passing:** `stress` still fails:
  - seed 1: 2 adverts from UAV 1 never leave its SITL (4 failed
    forwarding paths). While NavPy 1 sends its confirmation image, SITL
    1's serial1 (5.76 kB/s) is offered 6.1-6.3 kB/s; NAVLINK waits 2-3.4 s
    and much of it is never re-emitted (9 of 22 in that window; seed 4: 38
    of 62), likely ArduPilot forwarding only with TX space (todo 116).
  - seed 4 (0.56 %): the same flood during UAV 1's image; UAV 3 gets no
    APPLIED for 5 s, drops task 3 and flies home while the owner holds it
    CONFIRMED (todos 116, 115).
  - seed 8 (0.42 %): UAV 2's `SET_MODE` (AUTO) was broken on the uplink;
    the launch never checks the mode, so UAV 2 never took off (todo 117).
- Wall eval (no lockstep radio): failed after the matrices on approach
  quality, a 53.6° roll-command step, as in round 5; not rerun alone.
- Gate fix found by these runs: a confirmation response the uplink broke
  still counts as sent (`eval_gcs_demo_confirmation_timing.py`).

## Round 7 design: image share and confirmed launch mode (todos 116, 117, 110)

### Cause (round 6 journals, `stress` seeds 1, 4 and 8)

While NavPy 1 sends its image (640x480, quality 80, about 34 kB in 136
`ENCAPSULATED_DATA` chunks, 5 ms apart), its SITL's serial1 is offered
6.1-6.3 kB/s against 5.76 kB/s: the image takes 2.7-3.5 kB/s, everything
else 2.8-3.4 kB/s. NAVLINK then waits 100-170 ticks and much of it is never
re-emitted. So the image has room for about 1.5 kB/s at most.

Seed 8: UAV 2's one `SET_MODE` (AUTO) was broken on the uplink. The launch
never reads the mode, so it armed UAV 2 in its old mode and timed out.

### 116. One paced image sender per link

- Thumbnail 320x240, quality 80 (owner's choice, 2026-10-25): about 12 kB,
  47 chunks.
- `NetworkMavlink` owns one image sender thread (`PacedImageSender`). It
  sends one image at a time, so all images of that link share one budget.
  After each frame it waits that frame's wire time at the image rate:
  (payload size + 12 bytes MAVLink 2 framing) / rate, the payload size from
  the pymavlink message class (255 for a chunk: 267 bytes, 185 ms).
- Rate: `IMAGE_SHARE_OF_LINK` = 25 % of `RADIO_LINK_BYTES_PER_S` = 5760
  (57600 baud 8N1, the radio port's rate): 1440 B/s. Basis: other traffic
  peaks at 3.4 kB/s, leaving 2.36 kB/s; 1440 keeps 0.9 kB/s for bursts. A
  12 kB image takes about 8.5 s. Seeds 1, 4 and 8 check that it is enough.
- `send_image(poi_id, image_b64, admit)` queues the image and returns.
  `admit(send) -> bool` runs one frame's send under the round's exact fence
  (`send_if_current` / `send_pending_if_current`) and refuses once the round
  is resolved or retired; the sender then drops the rest of that image. The
  wait between frames is outside the fence. A send error is logged and ends
  that image (today it fails the round; a failed request send still does).
- At most one image per POI waits: a new image for a POI replaces any
  queued one for it (a replaced round's frames would be refused anyway);
  for the POI being sent, it waits behind it.
- The base `NetworkAbc.send_image` (WiFi, serial) keeps its synchronous
  loop and gains the same `admit` per frame.

### 110. The approval no longer waits for the image

- The round worker sends the request, queues the image and goes straight to
  its wait loop: request resends every 2 s and the deadline are unchanged.
- The thumbnail request handler only builds the thumbnail and queues it, so
  the dispatch thread of NetworkMavlink is free at once.
- A response resolves at once (the fence is held for one frame only); the
  next frame is refused. The thumbnail build and artifact save run outside
  the fence.

### GCS: ask for a thumbnail only when the image stalls

The GCS asks once per round, 3 s after the first request of the round, if
the image is incomplete (`task_confirm_rounds.py` `thumbnail_request_due`).
An 8.5 s image is still arriving then, so it would ask for a second copy.
The 3 s grace now counts from the later of the first request and the last
handshake or chunk of that round's image.

### 117. Confirm AUTO, resend until it holds, stop for the operator

- The telemetry feed passes each vehicle's heartbeat mode to the launch
  controller with `armed`, plus a heartbeat count (`HeartbeatState.observe`
  counts each autopilot heartbeat; `VehicleMav.heartbeat_count`; snapshot
  `heartbeat_count`). AUTO counts as confirmed only from a heartbeat counted
  after the first AUTO attempt, so a heartbeat from before the command
  cannot satisfy it, even when the vehicle already read AUTO.
- One helper, `_command_until(attempt, done, timeout)`: run `attempt`, poll
  `done` every 0.1 s as today, run `attempt` again every
  `LAUNCH_COMMAND_RESEND_S` = 1 s (the heartbeat rate: sooner, the effect of
  the last command cannot be seen yet), stop on done, timeout or abort.
- Each attempt first checks that the launch still owns the vehicle's
  commands. Any operator command for that vehicle (`/command`: mode, arm,
  disarm, E-STOP, reboot, ...) revokes that ownership for the session: no
  more resends; the wait goes on, so E-STOP or MANUAL ends in a timeout.
- Each arm attempt runs the EKF re-check right before it, as the one arm
  does today.
- Step 1 (and the already-airborne path) waits for AUTO within
  `arm_timeout`; on timeout the session fails with "AUTO mode timeout". The
  settle pause follows. The arm wait uses the same helper.
- Right before release, after its last await, the launch checks that it
  still owns the vehicle's commands and that the vehicle still reads AUTO
  and armed; if not, the session fails without the trigger, like the EKF
  re-check.
- Bungee launch has the same one-shot flaw but "proceed anyway" semantics;
  logged as a new todo.

### Code review additions (Codex, 2026-10-25)

- The fence is held for one step at a time. A queueing network
  (`NetworkMavlink`) fences its hand-over to the queue itself; the
  synchronous base `send_image` fences each frame. `ConfirmationMedia`
  no longer wraps the whole call, so a WiFi image sleeps outside the fence.
- AUTO counts from the heartbeat count read right before a send to a
  connected vehicle; a vehicle that is not connected sets no baseline, so
  its cached AUTO cannot confirm.
- A launch vehicle with no snapshot reads no mode and not armed
  (`forget_missing`), never its last cached state.
- The release conditions (no operator command, AUTO and armed) are
  checked on the executor, right before the trigger call, so a release
  queued behind busy workers stops for a change while it waited. The
  launch command helpers moved to `launch_commands.py`.

### Not covered

- Image delivery under byte loss: every chunk is needed; at 0.30 % per byte
  a 267-byte chunk survives about 45 % of the time, so a `stress` image
  rarely arrives whole. The matrices do not check image receipt. New todo.

### Checks

- Pacing from wire size and share (fake clock); one image at a time across
  POIs; a newer image replaces a queued one for its POI, also when a
  replacement round arrives before its first image and before a fallback.
- `admit` refusing ends the image; waits are outside the fence; a response
  delivered synchronously inside a frame's send resolves and the next frame
  is refused; a response mid-image resolves at once.
- Request resends keep their 2 s cadence during an image; the thumbnail
  handler returns without waiting.
- GCS: no thumbnail request while chunks keep arriving; one after a 3 s stall.
- Launch: a lost `SET_MODE` is resent until AUTO; a heartbeat from before
  the first attempt does not confirm it; timeout fails; arm the same; an
  operator command stops resends and, after arm, blocks release; the EKF
  check runs before every arm attempt; release refuses a vehicle no longer
  AUTO and armed; abort stops.
- Each mechanism test proved by mutation (23 mutations, all killed).
- `none`, `hw` and `stress` seed-1 matrices pass from a fresh worktree off
  origin/main; `stress` seeds 4 and 8 run as diagnostics, with serial1
  offered load and NAVLINK lag in the image window read from the journal;
  full pytest shows only main's failures.

## Not covered

- Air timing and per-link loss: todo 96.
