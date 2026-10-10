# Roadmap to the pickup demo (draft, 2026-10-09)

This roadmap merges the step orders from `mission_modules.md`, `architecture_split.md`,
`flow_runner.md` and `component_ports.md`. It was planned against origin/main `0c7f2ad`
(after PR #14), and code paths are under `src/navpy/modules/`.

**Owner re-order (2026-10-09):** first a clean `main` (done), then the refactor, then the rest. Refactor step 1 only separates modules: `step1_separation.md`.

**How to read it:**
- **Single main:** the work starts and finishes with only `main`. Steps run one at a time, and each is merged into `main` before the next starts. No branch outlives its step.
- **keep** means no behavior change: the oracle and the evals prove it.
- **change** means a behavior change the owner has ruled on.
- All evidence is SITL only and never shows a physical capture.
- No step adds a MAVLink message id, so the demo needs no firmware rebuild.

## A. Prepare
| Step | What | Kind | Size |
|---|---|---|---|
| R1 | Resync the docs to `0c7f2ad` (details below). Get owner approval. Record the baseline, including the 2 SOLID guard failures and the red orbit gate. | doc | S |
| R2 | Guards: a pure-vision scan derived from the law's imports (adds `pitch_law.py`), a poison vehicle, an `is_active` freeze and eval-marker tests | keep | M |
| R3 | Monte Carlo A: today's dive to an elevated POI, with wind and turbulence. Adds `--sitl-param` and a pull-out height-loss metric to `scripts/eval_navigation_cli.py` | keep | S |
| R4 | An operator's LOITER is a hold | change | S |
| R5 | Unknown task kinds decode as UNKNOWN on vehicles and in the GCS backend | change | S |
| R6 | Transition oracle (tests only), including the PR #14 rows and DEL_CTRL 2→0 at RESET | keep | M |
| R7 | Sim error knobs (pixel noise, latency, gimbal timestamp, sway) and a capture judge. Monte Carlo go/no-go, with pull-out measured on pickup's climb-out | keep | M |

R1 resyncs these points:
- 21 edges, not 20;
- the work order is now peer task > own POI > DDH;
- a decline replaces the sentinel bid;
- the heartbeat byte is FREE/BUSY;
- `line_dive` replaces the level pass;
- the pass budget lives in `PKUP_` params.

## B. Encapsulate (the refactor)
| Step | What | Kind | Size |
|---|---|---|---|
| R8 | One law registry replaces 3 tables. Every law (vision PN, PN, PID) is a plugin: inputs in, commands out, nothing else. It declares its required inputs and their quality (GPS CEP, frame rate, latency, real airspeed), and gets only those. The law is chosen at setup and never changes in flight. `NavLaw` stays where it is | keep | M |
| R9 | Flow types (`modules/flow/`) and delivery's label graph in `missions/delivery/`. The PID/PN orbit prep, recovery climb and floor move into a geo-approach skill that drives those laws. They are still gated each tick. Adds the layer guard | keep | M |
| R10 | `take()` replaces `phase.request` at the 21 sites, so only the runner picks edges | keep | M |
| R11 | Task kinds belong to the module, so comm stops knowing about docks | keep | M |
| R12 | An approach ends passed, lost or aborted:<reason>, carried through both failure latches | keep | L |
| R14 | Profiles `dive` and `line_dive`; `FinalApproachRequest(profile, track id)`. The profile is stamped in the log row only, not in the parsed status text | keep | M |
| R15 | Capture-line class 1 and its sim render. One shared line truth, at its height, is seeded into all 3 sims. The dock's ranges are pinned by a test | keep | M |
| R16 | DDH moves into `missions/delivery` | keep | M |
| R17 | The one-shot finish moves into delivery, and delivery is paused | keep | S |

## C. Pickup platform pieces
| Step | What | Kind | Size |
|---|---|---|---|
| R18 | Node-graph runner: loops, guards, staged context, one log row per commit | keep | M |
| R19 | Write fence plus the RTL and sortie-done latch for node graphs. Delivery stays unfenced | keep | M |
| R20 | Mailboxes keyed by task id; keyed, idempotent service actions | keep | S |
| R21 | RESET split into end_pass and end_task | keep | M |
| R22 | Module and role as Lua params, read at the arm edge: `AAS_MOD` plus a `PKUP_` table (role, passes, verify timeout, hold point, crossing heading, detection altitude, altitude bands) | change | M |
| R23 | CATCH kind and role bids: the scout and backup decline and the catcher bids. Assign requests use the same policy. An all-decline goes to the operator | change | M |

## D. Pickup module
| Step | What | Kind | Size |
|---|---|---|---|
| R24 | Skills hold, acquire and circle (in `behaviors/`) | change | M |
| R25 | Skills line_up, clear and wait_handover. line_up is ready only after its own vision re-detects the line | change | M |
| R26 | Catcher graph and pass loop. passed and aborted lead to clear until R28. 1-UAV SITL with an injected assignment | change | M |
| R27 | Scout graph: search, review, an explicit offer (no self-assign), observe | change | M |
| R28 | Cancel and verdict by task id, via acked `SWARM_REQUEST` types 3 and 4 (a vehicle listener plus GCS ack handling). verify starts with a climb-out | change | L |

## E. GCS and demo
| Step | What | Kind | Size |
|---|---|---|---|
| R29 | GCS plugin mechanism | keep | M |
| R30 | GCS pickup plugin: roles, `PKUP_` params, separation bands, cancel and verdict, pass n/N | change | L |
| R31 | Handover to the backup: cancel, operator enable, and a new CATCH task id | change | M |
| R32 | 3-UAV pickup eval; the demo is ready | keep | M |

- **Critical path:** R1 → R2 → R6 → R8 → R9 → R10 → R18 → R19 → R24 → R26 → R28 → R30 → R31 → R32.
- **In parallel:** the Monte Carlo (R3 → R7), the line class (R15), the GCS shell (R29) and delivery's extraction (R16 → R17).

## Added 2026-10-09 (owner)
- **Laws are chosen by their declared inputs.**
  - Each law declares its required inputs and their quality, which can depend on the profile.
  - The vehicle profile declares its data sources with the same fields: GPS CEP, camera rate and latency, pitot.
  - All of this happens at setup, before the sortie, never in flight:
    - R30: the mission builder lists the available laws, and what is missing for the rest.
    - R22: a preflight self-test measures frame rate and latency on the ground, and the law is latched at arming together with the module and role.
  - A switch during the demo happens between sorties.

## After the demo, before hardware
- **R13: airspeed only with a real sensor.** Gate it at the vision sources. In SITL the simulated pitot counts as real, so this step has no effect on the demo.
- **TUNNEL:** our messages travel inside MAVLink TUNNEL, so stock firmware works (`component_ports.md`).
- **Hardware gates** (`mission_modules.md`): fail-closed actuation, lost companion, fence as flown.
- **Scenario intelligence (later):** an AI model that reads the scene (forest or open ground, masts or balloon kit) and picks the strategy, the law and the profile. It is a model, not a heuristic, and it never feeds the vision law's commands.
- **Repos:** each top folder (platform, delivery, pickup, law plugins, GCS plugins) moves to its own repo, linked as versioned packages.
- **Later:**
  - the approach engine per role (P3);
  - each law into its own plugin package (S9, S10);
  - ports C0-C4;
  - dead peers (TODO 4);
  - delivery's vision circle (D1);
  - the rest of the TODO items.

## Review findings each step must keep
- **R9, R12:** today the law can switch at a reset, so the geo-approach skill's pieces stay gated each tick until R22 latches the law at arming.
- **R13:** the law reads airspeed from the vision frame (`navigation/nav/vision_nav/ingress.py:101`), and a frame with airspeed ≤ 0 is dropped.
- **R15:** a new class shifts the min and max class sizes, and with them the dock's maximum detection distance and the GCS confirm range.
- **R22:**
  - The GCS writes the params after NavPy has booted, so they are read at the arm edge.
  - Onhold happens only for invalid values, a forced arm or a restart in flight.
  - Run `npm test` and `npm run build`.
- **R23:** `can_fly` applies the role policy too (`swarm/task_capability.py:155`).
- **R25:** NAV exit stops tracking and centres the gimbal, so pass 2 must acquire again.
- **R27:** the advert path skips confirmed POIs, and today the scout's review self-assigns the task.
- **R28:**
  - Without a climb-out, the catcher stays low near the line while it waits for a verdict.
  - The verify card shows the SNAP miss and pass n/N; the sim judge's miss is labelled sim-only.
  - Run live-tester.
- **R30:** the scout's orbit, the backup's hold and RTL stay out of the dive corridor. R32 checks separation.
- **R31:**
  - Task ids are detection ids, so a re-offer needs a fresh id.
  - The scout learns catch-closed and enable from broadcast requests.
- **R32:** wait for a full roster of 3 before CATCH, because one node down stalls the auction.

## Owner questions (default in brackets)
1. Approve this order? [yes; commit the design docs through a temp worktree]
2. Re-pin the size guard to `0c7f2ad`? [yes]
3. What is the Monte Carlo go threshold? [p95 lateral miss under half the fork width, and pull-out loss under the segment's clearance] If it's a no-go, does the refactor still land? [yes]
4. Is delivery's orbit gate expected red under the vision law while delivery is paused? [yes]
5. Assign CATCH by auction, with only the catcher bidding and the operator handing over? [yes]
6. On a verify timeout: wait for handover, or fly one more pass? [wait for handover]
7. Does the backup get a fresh pass budget, or what the catcher had left? [fresh]
8. With no pitot, what speed does the law use? [decide after the demo]
