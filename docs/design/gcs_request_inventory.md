# Every request the GCS app sends (dev sim step 10)

Step 10 of `deterministic_dev_sim.md`. Each request the app sends, startup reads included, and the ones it sends outside the run, with whether the demo needs it and what it needs. `tests/gcs/test_request_inventory.py` checks the table against `src/gcs/frontend/src`: a new method and route, WebSocket message, built URL or external host without an entry fails, and so does an entry no longer sent or a request the scanner cannot resolve (a URL built by concatenation, another request API). The test reads source text, so a call that is never reached still counts: those rows say `dead`. Checked on main `fa77f32d`, 2026-11-03.

## Answers for step 11
These are about the requests the relay forwards into the run (the backend's routes). The WHEP video signalling has its own headers and SDP bodies, but it goes to the media server, not the run ("Outside the run").
- **Headers: no backend route needs one.** Some calls send `X-GCS-Client-ID` and `X-GCS-Request-ID` (`utils/diagnostics.js:28-36`). The backend uses them only to label diagnostic records (`main.py:175-176`, `routes/full_params.py:77,147`, `routes/missions.py:50`). Without them it makes its own request id with `rt.uuid4()` (`main.py:175`), so nothing the run does changes; only the timeline loses the browser's ids. `Content-Type: application/json` is implied by the wire's `json`.
- **Bodies: every backend request body is JSON** (`JSON.stringify`, checked at every `body:`), or there is none. No upload is a file or form; the mission "upload" is JSON that the backend sends over MAVLink. So the HTTP payload does not grow.
- **But one input is not HTTP:** the WebSocket messages `manual_control` (checked every 50 ms, sent when the sticks moved) and `manual_control_stop` (`hooks/useManualControl.js:44,49,87`). The backend turns them into MAVLink `MANUAL_CONTROL` (`routes/telemetry.py:62-84,127-130`). The wire has no kind for them. They are not demo-required; on lockstep the app relay drops them and logs it (step 11, todo 240).
- **Two responses are files, not text:** the diagnostics zip and the log zip. Neither is demo-required. A lockstep run refuses both (501, todo 241).

## Columns
- **Demo:** `yes` if the hand demo (`delivery_demo_prep.md`, "Normal-run demo") needs it: auto-connect, plan load, upload, Launch, confirm, abort. `sent` if not needed but sent anyway during the demo, so it is still an input. `dead` if no live code path calls it (todo 242).
- **When:** `startup` (on page load or when settings arrive), `connect` (when a vehicle appears), `user` (a click or edit), `timer` (a wall timer sends it, sends it again or decides it; see the clock audit).
- **Needs:** `MAVLink` (talks to a vehicle), `NavPy` (starts, stops or messages a companion), `ESP32` (the launcher), `files` (reads or writes host files), `local` (backend state only).
- **Sim today:** `eval` (the eval's operator workflow calls it on lockstep), `untested` (no refusal known in its path, but nothing calls it on lockstep; step 13 proves it or refuses it), `step 13` (wall-only today), `probed` (step 13's REST probe sent it through the app's relay on a lockstep run and it answered as the route intends; `.logs/dds-step13/live/app6_probe.jsonl`), `refused` (the lockstep coordinator answers 501, hardware-only, and journals it; step 11, `app_refusals.py`), `none` (no backend route).

## Requests
| Method | Route | Sent from | When | Demo | Needs | Sim today |
|---|---|---|---|---|---|---|
| WS | /ws/telemetry | `stores/telemetryStore.js:98` | startup, timer (reconnect 2 s) | yes | local | eval (`subscribe`) |
| WS-SEND | manual_control | `hooks/useManualControl.js:44` | user, timer (50 ms) | no | MAVLink | none (todo 240) |
| WS-SEND | manual_control_stop | `hooks/useManualControl.js:87` | user | no | MAVLink | none (todo 240) |
| GET | /api/settings | `hooks/useSettings.js:39,92` | startup | yes | files | probed |
| PUT | /api/settings | `hooks/useSettings.js:70` | user | no | files, NavPy (lifecycle change restarts) | probed (the run's own `gcs_settings.json`) |
| GET | /api/map-config | `utils/cesiumConfig.js:26` | startup | yes | files (the Ion token, `map_config.py`) | probed (no Ion token on lockstep, todo 280) |
| GET | /api/vision-profiles | `hooks/useSettings.js:40,81`, `settings/ProfileSelector.jsx:45` | startup | yes | files | probed |
| PUT | /api/vision-profiles/default/{profile_name} | `settings/ProfileSelector.jsx:572` | user | no | files, NavPy (restarts a running NavPy when the active profile changes, `routes/vision_profiles.py:228-241`) | refused (todo 269) |
| PUT | /api/vision-profiles/{profile_name} | `settings/ProfileSelector.jsx:105` | user | no | files, NavPy (as above) | refused (todo 269) |
| PUT | /api/vision-profiles/{profile_name}/devices/{device_name} | `settings/ProfileSelector.jsx:87` | user | no | files, NavPy (as above) | refused (todo 269) |
| POST | /api/vehicles/scan | `hooks/usePlanningApi.js:232` | never: `scanVehicles` has no caller | dead | MAVLink | probed |
| POST | /api/vehicles/discover | `hooks/usePlanningApi.js:246` via `handleScan` (`hooks/useVehicleConnection.js:632-634`) | startup, timer (auto-connect retry 3 s, 20 tries, `App.jsx:487-492`), user (Scan) | yes | MAVLink, NavPy, ESP32 (starts companions and the ESP32 sim, `routes/vehicles.py:87-93`) | probed |
| POST | /api/vehicles/connect | `hooks/usePlanningApi.js:210` | user (20 s abort) | no | MAVLink, NavPy (auto lifecycle), ESP32 (starts the ESP32 sim, `routes/vehicles.py:37-39`) | eval |
| POST | /api/vehicles/disconnect | `hooks/usePlanningApi.js:259` | user | no | MAVLink, NavPy | probed |
| GET | /api/vehicles/{sys_id}/params | `hooks/useVehicleConnection.js:188,653`, `hooks/useAasParams.js:135` | connect, user (10 s abort) | yes | MAVLink | eval |
| PUT | /api/vehicles/{sys_id}/params | `hooks/useAasParams.js:362`, `hooks/usePlanningApi.js:130` | user (upload writes `targ_wps`, `nav_last_wp`) | yes | MAVLink | eval |
| GET | /api/vehicles/{sys_id}/parameters | `hooks/useFullParams.js:158,165` | connect (prewarm, `App.jsx:396-398`), user | sent | MAVLink (MAVFTP) | eval (step 13; a torn file is fetched again, todo 272) |
| PUT | /api/vehicles/{sys_id}/parameters | `hooks/useFullParams.js:465` | user | no | MAVLink | eval |
| POST | /api/vehicles/{sys_id}/parameters/arm-token | `hooks/useFullParams.js:386` | user (long press) | no | local | probed |
| GET | /api/vehicles/{sys_id}/mission | `hooks/usePlanningApi.js:272`, `settings/AasTab.jsx:86,213` | connect, user (45 s abort, retries) | yes | MAVLink | eval |
| GET | /api/vehicles/{sys_id}/fence | `hooks/usePlanningApi.js:299` | user | no | MAVLink | probed |
| POST | /api/vehicles/upload | `hooks/usePlanningApi.js:100` | user (retries) | yes | MAVLink | eval |
| POST | /api/plan-camera-check | `hooks/usePlanningApi.js:119` | user, timer (400 ms debounce) | yes | files | probed |
| POST | /api/plan/analyze | `hooks/usePlanningApi.js:41` | never: drawing uses the local `localAnalyze` (`usePlanningOrchestrator.js:120-123`) | dead | none | none (404, todo 242) |
| POST | /api/plan/generate | `hooks/usePlanningApi.js:80` | never: plans are generated in the browser (`usePlanGeneration.js`) | dead | none | none (404, todo 242) |
| POST | /api/control/navpy-sim/start | `sidebar/MonitoringSidebar.jsx:151` | user | no | NavPy | probed (launcher port) |
| POST | /api/control/navpy-sim/stop/{sys_id} | `sidebar/MonitoringSidebar.jsx:148` | user | no | NavPy | probed (launcher port) |
| POST | /api/control/navpy-sim/restart-ready | `hooks/usePlanningApi.js:143` | user (after upload) | yes | NavPy | eval |
| POST | /api/control/command | `hooks/usePlanningApi.js:157`, `hooks/useCompassCal.js:17`, `PreflightReadiness.jsx:140` | user | yes (mode, E-stop) | MAVLink | probed; calibrations and reboots refused (todo 271) |
| POST | /api/control/launch | `hooks/usePlanningApi.js:171` | user | yes | MAVLink, ESP32 | eval |
| POST | /api/control/launch/trigger | `hooks/usePlanningApi.js:191` | user | yes | MAVLink, ESP32 | eval |
| POST | /api/control/launch/abort | `hooks/useVehicleCommands.js:81` | user | yes | MAVLink, ESP32 | probed |
| POST | /api/control/restart | `hooks/usePlanningApi.js:309` | user | no | MAVLink | probed |
| GET | /api/control/launch/esp32-status | `hooks/useContainerLauncher.js:21`, `settings/LaunchTab.jsx:22` | connect (container mode), user | yes | ESP32 | eval |
| GET | /api/control/launch/esp32-sim/status | `settings/LaunchTab.jsx:40` | user | no | ESP32 | eval |
| POST | /api/control/launch/esp32-sim/start | `settings/LaunchTab.jsx:51` | user, then a 500 ms re-check | no | ESP32 | eval |
| POST | /api/control/launch/esp32-sim/stop | `settings/LaunchTab.jsx:50` | user, then a 500 ms re-check | no | ESP32 | probed |
| POST | /api/control/task_confirm | `hooks/useTaskConfirmation.js:165` | user, timer (countdown auto-fire) | yes | MAVLink, NavPy | eval |
| POST | /api/control/task_confirm_override | `hooks/useTaskConfirmation.js:124` | user | no | MAVLink, NavPy | probed |
| POST | /api/diagnostics/events | `utils/diagnostics.js:45` | startup, connect, user, timer (`client_timeout` after a fetch abort; `plan_rendered` after a plan regeneration) | sent | files | probed |
| GET | /api/diagnostics/export | `settings/ConnectionTab.jsx:35` | user | no | files (zip response) | refused (todo 241) |
| GET | /api/logs/download/{sys_id} | `sidebar/NavpySimButton.jsx:14` | user | no | files (zip response) | refused (todo 241) |

**Built URLs** (a call whose URL is a variable; the test checks this list too; `wrapper` means the callers' rows hold the routes):
| File | Variable | Routes |
|---|---|---|
| `hooks/useFullParams.js` | `url` | /api/vehicles/{sys_id}/parameters |
| `settings/LaunchTab.jsx` | `endpoint` | /api/control/launch/esp32-sim/start, /api/control/launch/esp32-sim/stop |
| `utils/fetchRetry.js` | `url` | wrapper |

## Outside the run
These go to other servers, so the relay never forwards them and they never enter the run. The test checks that each external host in the source is named here; the vendored WHEP reader is read by hand.

| Request | Sent from | When | Demo | Notes |
|---|---|---|---|---|
| WHEP `OPTIONS`, `POST` (SDP body, `application/sdp`), `PATCH` (`application/trickle-ice-sdpfrag`, `If-Match`), `DELETE`, to each stream's `whep_url` | `vendor/mediamtx/reader.js:94,637,684,766`, via `hooks/useWhepStream.js:35` | a video window opens or closes | no (video is optional) | the media server; a simulated run has only the optional test pattern |
| Map tiles from `tile.openstreetmap.org` (street) and `{s}.tile.opentopomap.org` (terrain, the default) | `map/utils/baseLayers.js:68,73` | startup, map moves, a layer pick | yes (display) | display only |
| Cesium ion imagery and terrain (`IonImageryProvider`, `sampleTerrainMostDetailed`) | `map/utils/baseLayers.js:65`, `map/hooks/useCesiumViewer.js:285`, `map/hooks/usePlanSync.js:84`, `map/hooks/useCorridorLayer.js:204`, `map/hooks/useClosestApproachMarks.js:217` | startup, plan edits | yes (display) | display only: terrain heights feed the drawn map, not a backend request |
| Static files (`/aas-logo.svg`, the bundle) | `LoadingScreen.jsx:136`, the page | startup | yes | the frontend's own server |
| Links the operator opens: `www.google.com` maps, `ardupilot.org` docs | `TaskConfirmCard.jsx:376`, `utils/preflightRemediation.js:21-23` | user | no | opened in a new tab, not sent by the app |
| `www.w3.org` | SVG icons | never | no | the SVG namespace, not a request |

**Backend routes the app does not call** (the eval or tools do): `GET /api/vehicles`, `GET /api/vehicles/{sys_id}/params/named`, `DELETE /api/vehicles/{sys_id}/parameters/cache`, `POST /api/control/navpy-sim/start-all`, `POST /api/control/navpy-sim/stop-all`, `GET /api/control/navpy-sim/status`, `GET /api/diagnostics/session`, `POST /api/settings/reset`, `POST /api/settings/default-delivery-hubs/import`, `GET /health`.

## Clock audit
**Backend, behind every request simulation supports** (all rows but `refused` and `none`):
- The RuntimeTime lint (`tests/guards/runtime_time_lint.py`, run by `tests/guards`: 155 passed) covers all of `src/gcs/backend` and `src/navpy`. Covered code reads clocks, sleeps, makes threads, timers and ids only through `runtime_time`. `asyncio.sleep`, `wait_for` and timeouts run on the loop's clock, which under lockstep is the baton loop's sim time (`lockstep/backend_host.py:16`); AnyIO worker threads run on its managed executor (`backend_host.py:8,64-72`). So the resend gaps (`task_confirm.py:94`, `task_confirm_override.py:78`), the launch stagger and settle waits (`control_launch_actions.py:53,67,96`, `control_launch_workflow.py:158,172`, `control_preflight.py:134`), the manual-control loop (`telemetry.py:51`), the broadcast loop (`telemetry_loop.py:252`) and the startup auto-connect retry (`startup_autoconnect.py:90`) are on sim time.
- The lint's exemptions: `instance_registry*` and `cpu_share*` serve the launcher and the backend's start and stop, never a route. Since step 13 `_mavftp` is not exempt: `VehicleFtp` and its inbox run on `runtime_time`, and the full-parameter read it serves works on lockstep (todo 245, done). Before step 11 the app's prewarm reached it with wall clocks (`time.time()` stamps, a wall-timed queue), while serial-step never answered MAVFTP.
- What the lint cannot see:
  - `navpy_process_*` waits on processes with wall timeouts (`navpy_process_instance.py:254-267`, `navpy_process_spawn.py:51`, `navpy_process_stop.py:61`). Only the subprocess manager does this, and lockstep refuses it (`backend_host.py:111`); companions go through the launcher port. Not reached.
  - The two zip downloads take wall time from the standard library: `zipfile` stamps entries with file times or `time.localtime()`, and `FileResponse` sends `Last-Modified` and `ETag` from the file's stat (`routes/diagnostics.py:101-113`, `diagnostics.py:210-214`, `routes/logs.py:59-72`). Both are refused on lockstep since step 11 (todo 241).
  - The diagnostic session prunes old sessions by directory mtime (wall) against a sim-time cutoff (`diagnostics.py:136-149`). It runs at backend start, not on a request; a lockstep run starts in a fresh directory, so nothing is pruned. Logged (todo 243).
- Found nothing else: no `time.time`, `datetime.now`, wall sleep or wall timer outside `RuntimeTime` behind a supported request, and, since step 11 refuses the MAVFTP one above, none on a reached path.

**Frontend timers that can send a request** (display-only timers, such as map animation, blink and toasts, are left out):
| Timer | Sends | Status |
|---|---|---|
| Confirm countdown, `confirmClock.js:7-9` via `TaskConfirmCard.jsx:146-150` | `task_confirm` approve or deny at zero | fixed, step 11: counts the telemetry `ts` (`deterministic_dev_sim.md` section 5) |
| Armed token checked on `Date.now()`, `useFullParams.js:454,463` | decides whether `armed_token` is in the `PUT .../parameters` body | fixed, step 11: the telemetry `ts` |
| Fetch aborts: 20 s connect, 45 s mission (`usePlanningApi.js:10-20`), 10 s params (`useVehicleConnection.js:187`) | no cancel; but the connect and mission aborts post a `client_timeout` diagnostic (`usePlanningApi.js:218-219,282-283`) | fixed, step 11: each carries its `deadline_s`; on lockstep the page sets no wall timer, and the run's deadline answer (504) is its timeout, so sim time decides the diagnostic and any follow-up (todo 244) |
| Retry backoff, `fetchRetry.js:14,23` (upload 1 s then 2 s; mission 0.5 s then 1 s) | the same request again after a 5xx, 408 or network error | fixed, step 11: no retry on lockstep (todo 244) |
| Auto-connect retry, `useSimAutoConnect.js:50` (3 s, 20 tries) | `POST /api/vehicles/discover` again until a vehicle is found (it calls `handleScan`) | new, todo 244 |
| Manual control, `useManualControl.js:49` (50 ms) | `manual_control` | todo 240 |
| Debounce: camera check 400 ms (`usePlanCameraCheck.js:48`); the area-analyze debounce (`usePlanningApi.js:39`) is dead code | the request for the operator's latest edit | accepted: a debounce only picks when an edit is sent, like a click; the action file records it |
| Plan regeneration throttle 100 ms, `usePlanGeneration.js:317-325` | a new plan, then the `plan_rendered` diagnostic (`App.jsx:115-122`) and the camera check (after its debounce) | accepted, as for the debounces: it follows an edit |
| Status re-check 500 ms, `LaunchTab.jsx:56` | `GET /api/control/launch/esp32-status` | accepted, a read after a click |
| WebSocket reconnect 2 s, `telemetryStore.js:150` | a new telemetry connection | accepted: the relay serves telemetry from the journaled broadcasts, so watching never enters the run |
| Press timers: long press (`ParametersTab.jsx:710-714`), press guard (`usePressGuard.js`, `useLongPress.js`) | the request once the hold counts | accepted: the operator's own gesture; the request it gives is recorded |

**The relay.** Since step 11 the app relay (`scripts/lockstep_app_relay.py`) forwards only: no retry, timeout, batching window or keepalive of its own, and a closed page sends no cancel. Before it, what reached the run kept no wall timer: the scripted operator runs on its own baton in sim time (`operator_ports.py:142-151`), and admission is by tick (`deliver_operator_requests`, `coordinator_operator.py`). The SITL serial relay (`scripts/lockstep_relay.py`) sleeps only for seeded host delays, which the matrix's `delay-7` and `delay-13` cells prove do not move the digest. (`deterministic_dev_sim.md` section 5.)
