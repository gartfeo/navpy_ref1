# HITL whole-dock detector (design v2.1, 2026-11-05)

Status: design for owner review. No code until the owner approves. It adds one step to the HITL lean plan
(`hitl_final_approach.md` section 9, revision 25edd29c), so the plan needs a revision too (section 7).
v2 applies Codex review round 1 (section 10), v2.1 round 2 (section 11).

## 1. Why

HITL step 23 (PR #89, merged to main 2026-11-06) flies the owner's 300 m orbit
(`final_approach_orbit.radius_m` in profile `siyi_zr10_hitl_encoded`). Its pre-series check refuses every series:

- ChArUco needs about 11.5 px per 40 cm square along the line of sight (dock4, `hitl_unreal_camera.md`). At 10x
  (fy 16812 on the 1920 x 1080 stream) and 80 m above the board that limits the slant to about 216 m. At 300 m
  radius (slant 310 m, depression 14.9 deg) a square gets about 5.6 px (`tools/hitl/scenario_checks.py`
  `check_reader`).
- Step 22's pose gate (`board_pose.py` `MAX_CENTRE_SIGMA_M` 0.3 m) refuses fits beyond about 115 m, so the orbit
  centre falls back to the height-zero ground ray.

Owner, 2026-11-05: "as we are detecting big object we can be far"; "Detect the net as one object".
The NAV approach accuracy goal (< 0.5 m) is a separate task and is not changed here.

## 2. What changes, in short

1. **Detector:** the HITL profile runs the existing YOLO backend (`plugins/detection/yolo/yolo_detector.py`) with a
   model trained for one class, `dock`: the net as one box. ChArUco is not used in this profile. Other profiles keep
   ChArUco as the product default.
2. **Orbit centre:** the existing height-zero ground ray (`SelfDetectedApproach._vision_centre`), with an error
   bound the pre-series check must absorb. A multi-ray re-centre is designed (section 4.3) but built only if the
   measured ground-ray error fails the check.
3. **Pre-series check:** `check_reader` becomes `check_detector` (box size against the detector's measured
   minimum, at the first-detection distance and on the orbit); the centre error is the ground-ray bound.
4. **Measurements** decide the numbers (section 6). Every threshold below is a named placeholder until measured.

## 3. Detector

- **Backend:** `--detector-backend yolo` already exists (`core/kernel/config/detector_backend.py`). It is opt-in and
  needs `--detector-model-path`; that stays. The HITL profile names it; no implicit model.
- **Model:** fine-tune `yolov8s` (already kept in `.models/`) on one class, `dock`
  (`vision_class_profile.py` `DOCK_PRESET_NAME`). Weights stay out of git, in the private HITL folder; the run
  record (step 1, `approach_records.py`) records the model, dataset and preprocessing hashes.
- **What is boxed and aimed at:** the net only, not its platform. A 360-degree net looks the same from every
  azimuth, so its box centre stays near one physical point, the net's axis at mid-height. That point is the POI.
  Section 6 item 3 measures how far the box centre moves from it.
- **Training data:** Unreal renders of the dock through the same encoder path as the series (H.265, the HITL
  stream settings), labelled automatically from the Unreal projection of the net mesh. Truth is used offline for
  labels only, never at run time. Three disjoint sets by scene, seed and dock placement: training, commissioning
  (thresholds, input size) and a final held-out set. Weights, confidence threshold and preprocessing are frozen
  before the held-out set and before any series; the series' private seeds are in none of the sets. Include
  terrain, light and haze variation and dock-free frames.
- **Input size and crop:** at 300 m radius and 80 m above the dock, a 2 m side along the line of sight is about
  28 px on the full frame, but 9 px at `imgsz` 640, 14 px at 960 and 19 px at 1280 (ultralytics letterboxes to
  `imgsz`). Search and reacquisition run on the full frame. Once locked, an optional crop around the track box is
  resized to `imgsz` with a recorded transform, and box pixels are mapped back to the full frame before any use.
  The effective scale is (`imgsz` / crop side); input size and crop are chosen by measurement (section 6).
- **Output:** the existing `Detection` (box, class, confidence). Navigation uses only the box centre pixel, turned
  into a frame-local ray, as with ChArUco's board centre today.
- **False and ambiguous detections:** a box detector has no fiducial check, and a static look-alike gives a
  persistent track. So:
  - acquisition gate: a track of `MIN_LOCK_S` with confidence ≥ the frozen threshold, whose box's smaller side is at
    least `ACQ_MIN_BOX_PX`, starts the orbit; more than one `dock` track in view means no lock (hold the search, log
    it). The box-size floor is enforced in navpy and bounds the first-detection distance (section 4.1); box size is
    detector output, used in acquisition only, never in NAV;
  - NAV gate: step 25's confirm before NAV, with the track held on the orbit for `MIN_ORBIT_TRACK_S`. Step 25 checks
    freshness, lock and order, not identity, so a persistent look-alike passes it. Who confirms identity is owner
    question 6: the operator, looking at the video (recommended), or automatic confirm with the measured risk;
  - an orbit started on a false box is a wasted orbit at lane height, not a NAV; NAV on a false box is the failure
    the negative tests measure (section 6 item 2).
- **Identity:** the detector cannot tell two docks apart, and assignment rules do not prove which physical dock is
  seen. The HITL series has one dock; several docks are out of scope and the ambiguity rule above refuses them.

## 4. Orbit centre without a ChArUco pose

### 4.1 Ground ray (exists)

`SelfDetectedApproach._vision_centre` falls back to `ground_location(..., reference_height_m=0.0)` when there is no
board pose; in this profile there is never one. The centre comes from the **first eligible detection**, at
horizontal distance D. A whole-object detector can fire far beyond 300 m, so D is bounded by the enforced
acquisition box floor `ACQ_MIN_BOX_PX` (section 3): D_max is the distance where the net's smallest projected box
side, less the measured box-size error, equals that floor; never less than r. Error terms at D, slant
S = sqrt(D^2 + h^2), h the lane height above the POI (the net's mid-height):

| Source | Horizontal error (examples at D 300 m, h 80 m, 10x) |
|---|---|
| POI height above home, hd (unknown to navpy) | D x hd / h |
| Lane height error δh (aircraft relative altitude and home reference) | D x δh / h (3.75 m per metre) |
| Camera depression bias δ (gimbal readback, mount, aircraft pitch) | S^2 / h x δ (2.1 m per 0.1 deg) |
| Camera azimuth bias ψ (gimbal yaw readback, mount, aircraft compass yaw) | S x ψ, sideways (5.2 m per deg) |
| Box-centre bias and noise, vertical / horizontal pixels | S^2 / (h fy) per px radial (0.07 m); S / fx per px sideways (0.02 m) |
| Capture-time skew t (aircraft at v) | v x t (2.4 m per 100 ms at 24 m/s), plus rotation-rate x t terms |
| Aircraft position error common to aircraft and centre | none: the orbit is flown in the same frame |

Aircraft compass yaw is allowed here because this is acquisition, not NAV; its bias is the largest declared term
and must be bounded honestly (a few degrees is tens of metres). The bound e used by the check is the plain sum of
the declared bounds at D_max: the scene's design POI height, declared lane-height, depression and azimuth bias
bounds (stage 1's emulated gimbal and SITL attitude give small values; stage 3 and later use measured readback and
compass bounds), the measured box-centre bias (section 6 item 3) and the measured capture-time skew.

The error does not lose the dock: the gimbal tracks the box, and the picture at 10x covers about 35 x 20 m at 310 m.
It does change the horizontal distance to the dock round the circle to between r - e and r + e (slant
sqrt((r ± e)^2 + h^2)), which every check must cover.

### 4.2 Pure Vision Approach Constraint

- The orbit centre is acquisition geolocation (GPS position and aircraft attitude, allowed outside the approach,
  owner 2026-10-07). As in step 22, nothing of it reaches the NAV command or completion paths.
- The NAV law reads only the box centre as a frame-local ray, with gimbal readback as camera state. Box size and
  box height never reach the command path.
- Test: the existing isolation guards plus mutations that feed box size, the orbit centre, aircraft heading,
  position or altitude into each NAV command and completion input; every mutation must be caught.

### 4.3 Multi-ray re-centre (designed, built only if 4.1 fails)

If the measured ground-ray error makes the check refuse, add one re-centre during the orbit: the least-squares point
closest to N camera rays to the box centre (sum of (I - d d^T); the standard line intersection). It needs no dock
height. Limits, from review: bias cancellation holds only for full, even coverage round a circle centred near the
dock; a constant depression bias moves the fix vertically with zero residual; correlated errors are invisible to the
fitted sigma. So the fit's sigma is never used as the error bound. The check would use a validated systematic-error
bound from seeded simulations with injected yaw, depression and timing bias and uneven, partial coverage, plus
coverage and conditioning gates. Module: `core/skills/geo_approach/ray_orbit_centre.py`; it would be its own plan
step with its own approval.

## 5. Pre-series check changes (`tools/hitl/scenario_checks.py`)

- **`check_reader` → `check_detector`:** the dock's smaller box side, in full-frame pixels times the input scale,
  is at least `detector.min_box_px`, over dock yaws, at the farthest horizontal distance on the orbit (r + e). That
  value is the measured box size where recall meets the target (section 6 item 1), stored in the profile with its
  evidence pointer. No ChArUco square size in this check.
- **First-detection distance D_max:** from the enforced acquisition floor `ACQ_MIN_BOX_PX` at the search zoom and
  the net's size, as in section 4.1. It sets the ground-ray bound e; no truth is read.
- **Dock size** comes from the scenario's public half (`scenario_public.py`, its owner is the step 23 tooling): the
  net's diameter and height. It is a scene design fact, validated like the other public fields, read only by the
  check, never by navpy. Per-seed dock height or pose stay private.
- **Containment:** the same both-axis margins with the net's box instead of the ChArUco grid, at r - e (zoom-in) and
  r + e (no zoom past).
- `pose_accepted_at` and the ChArUco pose prediction leave this profile's check. Turn radius, descent, model speed
  and recovery floor are unchanged, with r ± e as now.

## 6. What has to be measured

All in Unreal through the encoded HITL stream, on the held-out set unless stated. Simulation numbers only. Rates are
reported with 95 % intervals.

1. **Detection versus box size:** per-frame recall and false-box rate against the smaller box side, at the series'
   zoom steps and depressions 10-20 deg, per candidate input size and crop (commissioning set). Sets
   `detector.min_box_px`: the smallest size with recall ≥ 95 % per frame and a lock within `MIN_LOCK_S` + 1 s in
   ≥ 99 % of passes (my targets, for approval).
2. **False boxes and false locks:** search sweeps (step 21) and orbits over the scene's terrain, dock-free and with
   look-alikes beside the real dock, plus track-switch and reacquisition cases, for a declared exposure. Report
   locks and confirms with their one-sided 95 % upper bound: 0 events in 2 h bounds the rate only to about 1.5 per
   hour, so zero events alone does not justify automatic confirm (owner question 6).
3. **Box-centre bias:** box centre against the projected net POI, in pixels and as metres at range, by azimuth,
   range and partial clipping. Acceptance: within `MAX_CENTRE_BIAS_PX` (set from the orbit and NAV needs; it also
   feeds the later < 0.5 m task).
4. **Approach tail through the real detector and tracker:** encoded approach replays from orbit to contact, with
   zoom transitions and settling, the net clipped by the frame edge and partly hidden, loss and reacquisition.
   Report the longest valid-measurement gap, box-centre jumps at clipping, and frame capture-to-consumption age and
   queue depth (not only inference time), against the existing feasibility contract
   (`hitl_final_approach.md` section 4, "Feasibility contract").
5. **Inference time** per frame on the PC GPU for each input size and crop (the Orin later, stage 2), against the
   stream rate.
6. **Ground-ray error:** first-detection distance D and centre error against truth (scorer only), on SITL replays,
   to confirm the section 4.1 bound. If the bound fails the check, section 4.3 becomes a plan step.

## 7. Proposed plan change (needs owner approval)

New step **23b, whole-dock detector**, next after step 23 (merged 2026-11-06 with the ChArUco reader check, which refuses every 300 m series). 23b replaces that check with `check_detector`.

23b. **Whole-dock detector — `plugins/detection/yolo`, the HITL profile, `tools/hitl/scenario_checks.py`,
`tools/hitl/scenario_public.py`, an Unreal dataset tool in `tools/unreal`.** Add a net mesh to the Unreal scene;
train a one-class `dock` model on auto-labelled encoded renders in three disjoint sets; freeze it; run it in
`siyi_zr10_hitl_encoded` with the acquisition and ambiguity gates; orbit on the ground ray; replace the reader check
with the detector check. Test `test_approach_geometry.py` (detector check feasible/impossible, D beyond r),
detector-gate tests (two tracks, short track) and the isolation guards with the section 4.2 mutations. Done: the
section 6 measurements recorded; the check passes at 300 m with measured values; 0 false confirms in the declared
exposure; 0 box-size, centre, heading, position or altitude reads on NAV command and completion paths.

## 8. Owner questions

1. **The net in the HITL scene:** diameter, height, mounting and colour? Today the scene has only the flat
   2.0 x 2.8 m ChArUco board. My recommendation: add a net mesh at the real net's size; no board on it.
2. **Ground ray first, multi-ray only if needed** (recommended, Codex agrees), or build the multi-ray re-centre now?
3. **Plan placement:** step 23b as the next step, before step 24 (recommended), or later in the plan.
4. **Targets:** recall ≥ 95 % per frame, 0 false confirms in 2 h of dock-free video. Approve or set others.
5. **Training data:** auto-labelled Unreal renders now; real ZR10 video of the net later (stage 5; the sim-to-real
   gap is not covered here). Agree?
6. **Who confirms the dock's identity before NAV:** the operator from the video (recommended; a look-alike cannot
   pass), or automatic confirm, accepting the measured false-confirm bound of section 6 item 2? Operator confirm
   means a person on every series run.

## 9. What this cannot prove

Simulation only: no physical docking, net capture or cargo receipt. Detection on Unreal renders does not prove
detection on real ZR10 video. The NAV < 0.5 m accuracy is a separate task.

## 10. Codex review round 1 (2026-11-05): "rework", applied in v2

1. Fit sigma is no error bound; common GPS offset and depression bias are invisible. Holds. The multi-ray fix is now
   conditional (4.3) and its bound must be a validated systematic budget. A common GPS offset is harmless to the
   orbit itself (same frame), noted in 4.1.
2. The first detection, not the orbit radius, sets the ground-ray error. Holds (the branch's a811d5e5 already uses
   D). 4.1 and 5 use D and slant ranges.
3. A static look-alike passes persistence and the fix. Holds. Separate acquisition, ambiguity and NAV gates and a
   negative exposure test (3, 6.2).
4. The approach tail was not covered. Holds. Measurement 6.4.
5. A whole-object box centre is not a fixed point. Holds. Net only, defined POI, bias acceptance (3, 6.3).
6. ChArUco at 300 m gets 5.6 px, not 8; a crop has no scale 1. Holds; both fixed (1, 3, 5).
7. Three disjoint sets, frozen weights, intervals, isolation on all command and completion paths. Holds; applied.
   Simpler option: ground ray first. Agreed; now the recommendation.

## 11. Codex review round 2 (2026-11-05): "rework", applied in v2.1, not re-reviewed

Round 1's points 1, 4, 6 and 7 resolved; 2, 3 and 5 partly. Codex checked the corrected numbers.
1. The recall threshold does not bound D; a detector fires farther. Holds. An enforced acquisition box floor
   `ACQ_MIN_BOX_PX` now bounds D_max (3, 4.1, 5).
2. The bound omitted box-centre error, azimuth error and lane-height error; "dock height" was ambiguous. Holds. All
   added to the 4.1 table and to e; height is now to the net's mid-height POI.
3. Step 25 has no identity check; 0 events in 2 h bounds only about 1.5 per hour. Holds. Look-alike and
   track-switch cases, the upper bound reported, and owner question 6 (operator or automatic confirm).
A third round was not run (lean process, 2026-11-04: one review round); the owner may ask for one.
