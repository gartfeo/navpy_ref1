# MAVLink reader: yield per batch (design v3, todo 31/34/38)

## Problem
During the vision final approach (NAV) at 10x the MAVLink reader handles
~400-540 packets/s against ~3000/s offered (all types; ~820/s ATTITUDE +
position), falls up to 2.7 s behind, and approaches fly on old attitude:
pitch runs away, `missed_dock`/`no_detection` (`.logs/quiet-1`, `-2`, quiet
host). After NAV it drains at 900+/s.

## Measured (`scratchpad/diag_quiet/ab.md`, 12 runs, quiet host)
Max NAV reader lag per UAV:
- base: up to 354 ms (126, 262, 275, 84, 18, 18, 12, 222, 354)
- 5 ms switch interval in NAV (si5): up to 501 ms (no better)
- **yield once per batch: 6-16 ms**
- both: 4-26 ms
(9 NAV legs per variant; the batch run was unbounded and NavPy-only.)
Detector frames/s (68-81) and command-worker median run time (0.41-0.78 ms)
are the same in all variants. The reader made ~300 yields/s for ~3000
packets/s in the batch variant. So the per-packet `rt.sleep(0)`
(`plugins/vehicle/ardupilot/mav_bus_reader.py:19-21,77`) is the throttle; the
NAV 1 ms switch interval is not, and stays.

## Fix
`MavBusReader._read_until_stopped`: after a blocking `recv_match` returns a
packet, keep dispatching packets that are already decodable without blocking
(`recv_match(blocking=False)`), then yield once.
- **Fairness budget:** the batch also ends once it has run for one scheduler
  wall slot, so continuous ingress cannot starve the detector or command
  worker (why the per-packet yield exists). Ownership:
  - `MavBus` owns a locked budget registry (`MavBus.reader_budgets`, a plain
    attribute: the bus is at its method limit): `request(owner)` returns a
    handle with `set(wall_slot_s)` and `release()`. The reader uses the
    minimum of the live requests, or the default (one 1x slot of a 50 Hz
    loop, 20 ms) when none. So the GCS backend, with no cadence, keeps 20 ms,
    and the fastest vehicle on a shared bus wins.
  - In NavPy the composition root opens the vehicle's bus
    (`open_vehicle_bus`) and passes a request on it to `SchedulerCadence` as
    a `ReaderBudgetFeed` (sink + scheduler period; `wall_scheduler_cadence.py`),
    so `VehicleMav` gains no method (its facade allowance is frozen). The
    period is resolved before the first sync; the cadence sets
    `period / speedup` (2 ms at 10x) on every speedup sync (both sync paths)
    and releases on close. Before the cadence exists the default holds; the
    lag only appears in NAV, long after.
  - At 1x hardware the budget is 20 ms; hardware links rarely keep the
    reader busy that long. A test pins it.
- **Soft bound:** checked between packets, so a batch overshoots by at most
  one packet's receive + dispatch (synchronous callbacks; todo 38). The yield
  runs in `finally`, also on dispatch errors. A blocking TCP reconnect inside
  pymavlink (`autoreconnect=True`) ends the batch like today's read does.
  Stop is checked per packet.
- **Reply retention, fail closed:** neither today's per-packet yield nor a
  batch guarantees a waiter runs before its reply is evicted; that depends on
  thread scheduling. What makes it safe is failing closed: `MessageStore`
  already raises `MessageHistoryOverrun` when a waiter's cursor was evicted.
  `MissionInbox` evicts silently past 256 (`mission_inbox.py:28,42`); it now
  tracks the evicted cursor the same way and raises, so a lost answer is an
  error, never a silent miss. Normal batches hold a few replies (rate x
  budget), far under 256/1024.
- **Parser semantics:** a `None` from the non-blocking call ends the batch,
  whether the socket is empty or holds a partial frame; the next blocking
  call completes it. Several packets in one datagram are returned one per
  call from pymavlink's buffer, so none is skipped (test).
- The yield stays the rt port (`rt.sleep(0)` via `cooperative_yield`), so
  lockstep keeps its baton scheduling point; `LockstepBusReader` is unchanged.
- Applies to every `MavBusReader` (NavPy and GCS backend). Lockstep is
  unaffected: `LockstepBusReader` keeps its per-packet `yield_now(baton)`
  (`lockstep_bus.py:97,110,132`); wall and lockstep reader scheduling now
  differ, and the parity wording says so.

Not in this change: a reader-lag fail-closed check (separate design, see
`reader_lag.md` review), and the lag-independent `no_detection` seen at
~5 ms lag.

## Tests
- Batch: all ready packets dispatched in order before one yield; a burst
  longer than the wall slot yields at the bound; stop mid-batch exits; an
  idle socket blocks with the existing timeout; a datagram with several
  packets and a split frame across datagrams are all dispatched once.
- Budget: default 20 ms; set to `period / speedup` on speedup sync; batch
  ends at the budget; yield in `finally` after a dispatch error (replaces the
  two per-packet-yield tests in `test_mav_bus.py`, on purpose).
- Reply history: a burst within one batch reaches `wait_after` and
  `MissionInbox` waiters; a burst beyond each depth with a delayed waiter
  raises (`MessageStore` and now `MissionInbox`); `test_lockstep_bus.py:327`
  unchanged.
- Shared bus: two owners, minimum wins; release restores the next minimum or
  the default; speedup sync updates the budget.
- SITL (the bounded, NavPy + GCS variant): two quiet-host 3-UAV wall runs,
  NAV lag < 50 ms per UAV; lockstep
  acceptance (`NAVPY_LOCKSTEP_SITL=1`) docks and two same-seed runs share a
  digest.
