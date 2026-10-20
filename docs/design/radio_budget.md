# Radio budget: peer-telemetry filter and air load

Status: items 2-3 built 2026-10-20; items 1 and 4 are todos 91 and 92.
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

### 1. Hardware direction (later; its own design and review; todo 91)

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
  every Orin's router. Each SITL endpoint gets `BlockSrcCompOut = 1,100`; the
  backend gets no filter. The packets each autopilot receives are the same as
  in item 1. Queues, latency and air time are not the same; the air is
  reported (item 3), not modelled.
- In the sim, NavPy's NAVLINK still goes through its autopilot (serial0 to
  serial1), as today.
- What the filter passes, exactly:
  - companion traffic from component 191, so NAVLINK and also gimbal telemetry
    and STATUSTEXT;
  - camera components other than 100;
  - GCS traffic (sysid 255, component 0).
  It blocks targeted messages from components 1 and 100 too, and ids outside
  the router's entry table: `read_msg` takes `msg_id` from the header for every
  packet, so the `UINT32_MAX` skip in `accept_msg` never applies
  (`endpoint.cpp:349,369,432`).
- Router model (`router_model.py`), following `accept_msg`:
  - the loop check first;
  - then `BlockSrcCompOut`, the only filter this configuration uses (the model
    takes no other);
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
  before the sniffer, and mixed loop, filter and target rejection. Compare bytes at every
  endpoint and the unknown-target counter. Keep the unfiltered cases.
- Wall: `run_swarm.sh` writes the SITL feed endpoints, each with the filter,
  to `$ARDUPILOT_DIR/router_<SWARM_OFFSET>.d/sitl_feeds.conf` per launch and
  starts the router with `-d` on that directory (`rfd900` profile unchanged).
  `-d` adds to the default `main.conf` instead of replacing it, as `-c` would.
  The sniffer and the Windows endpoints stay on the command line, since
  `wall_reap_wsl.py` finds the router by its monitor port. The file lives in
  the ArduPilot fork, whose deployed copy has long-standing uncommitted edits;
  its sha256 pin in `lockstep_topology.py` changes with it. The new copy is
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
  sim time, plus the network total. A partial last window is scaled by its
  length. Setup (before the first NavPy spawn: connect, parameter writes,
  mission uploads) is reported apart. Built: `eval_gcs_demo_air_load.py`,
  `report.json` `air_load`.
- SITL drops lost bytes before the coordinator sees them, so offered load is
  measured only on runs with `uart_loss` off. A loss-on run reports what it
  observed and says so.
- No pass or fail until the owner sets a usable budget from the bench test, so
  no threshold is invented.

### 4. Stream rates and air rate (owner decision, later; todo 92)

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
- **Backlog:** `hw` seed 2 (line rate only) passes. Its NAVLINK latency from
  `router_copy` to `navpy_delivered` passes through the firmware's input FIFO
  and the autopilot's forwarding, so it shows FIFO growth. No 1000-tick window
  may exceed the largest latency of the same seed with the line rate off.
  Coordinator pending bytes stay 0.
- **Matrices:** `none`, `hw` and `stress` each give one digest and byte-equal
  artifacts, with new baselines.
- **Wall:** one wall eval run on the new `run_swarm.sh`, passing.

## Not covered

- Air timing and per-link loss: todo 90.
