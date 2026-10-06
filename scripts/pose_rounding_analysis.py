"""Three-arm, same-trajectory attribution after the complete deterministic matrix."""

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts"), str(ROOT)]
import numpy as np
import pymap3d

from analyze_noise_profiles import local_path, command_metrics
from eval_direct_pixel_command_causality import _samples
from pose_rounding_evidence import read_pairs
from replay_navigation_trace import digest, run as replay_recorded, verify_live_rows
from replay_pose_rounding import replay_arm
from run_pose_matrix import BASELINE, cells
from simtime_navigation_protocol import RenderTruth, Snapshot

ORIGINAL_REVERSAL_US = 83106312
FIRMWARE = ROOT / "docs/validation/navigation-pose-firmware-20260921.json"


def require_matrix(report: dict) -> None:
    if report.get("passed") is not True or report.get("version") != 1 or report.get("steps") != 15000:
        raise ValueError("pose analysis needs the complete accepted matrix")
    if (report["method_sha256"] != digest(ROOT / "scripts/run_pose_matrix.py") or
            report["pairs_method_sha256"] != digest(ROOT / "scripts/pose_rounding_evidence.py") or
            report["baseline_sha256"] != digest(BASELINE)):
        raise ValueError("pose validator or baseline changed")
    ledger_path = local_path(report["ledger"])
    if digest(ledger_path) != report["ledger_sha256"]:
        raise ValueError("pose ledger changed")
    attempts = json.loads(ledger_path.read_text())["attempts"]
    expected = cells()
    if len(attempts) != len(expected) or len(report["runs"]) != len(expected):
        raise ValueError("pose matrix has missing cells")
    baseline = json.loads(BASELINE.read_text())["normalized_history_sha256"]
    firmware = json.loads(FIRMWARE.read_text())
    for attempt, row, cell in zip(attempts, report["runs"], expected):
        if (attempt["status"] != "captured" or
                any(attempt[key] != value for key, value in cell.items()) or
                any(row[key] != cell[key] for key in ("pose_capture", "speedup", "delayed")) or
                attempt["run_id"] != row["run_id"] or
                local_path(attempt["directory"]).resolve() != local_path(row["directory"]).resolve() or
                row["steps"] != 3000 or row["normalized_history_sha256"] != baseline):
            raise ValueError("pose matrix differs from declared cells/baseline")
        if (row["identity"]["binary_sha256"] != firmware["binary_sha256"] or
                row["identity"]["firmware_head"] != firmware["base_commit"]):
            raise ValueError("pose matrix used an unreviewed firmware build")
        if digest(local_path(row["directory"]) / "fleet.json") != row["manifest_sha256"]:
            raise ValueError("pose manifest changed")
    if len({row["run_id"] for row in report["runs"]}) != 5 or len({tuple(row["boot"]) for row in report["runs"]}) != 5:
        raise ValueError("pose matrix reused a boot")
    if len({row["pose_hash"] for row in report["runs"][1:]}) != 1 or not report["runs"][1]["pose_hash"]:
        raise ValueError("pose metadata differs across boots")


def reversal(samples: list[dict], source_us: int) -> dict:
    index = next((i for i, row in enumerate(samples) if round(row["obs_ts"]*1e6) == source_us), None)
    if index is None or index == 0:
        return dict(source_us=source_us, available=False)
    before, after = samples[index-1], samples[index]
    return dict(source_us=source_us, available=True, before_roll=before["cmd_roll_deg"],
        after_roll=after["cmd_roll_deg"], roll_change=after["cmd_roll_deg"]-before["cmd_roll_deg"],
        visual_rate_change=after["lateral_rate_deg_s"]-before["lateral_rate_deg_s"],
        own_turn_change=after["aircraft_turn_rate_deg_s"]-before["aircraft_turn_rate_deg_s"],
        before_branch=before["plan_reason"], after_branch=after["plan_reason"],
        dt_s=after["obs_ts"]-before["obs_ts"])


def signed_summary(values: list[float]) -> dict:
    array = np.asarray(values, dtype=float)
    if not len(array) or not np.isfinite(array).all():
        raise ValueError("missing/nonfinite paired signal")
    return dict(mean_signed=float(array.mean()), std=float(array.std()),
        min_signed=float(array.min()), max_signed=float(array.max()),
        p95_abs=float(np.percentile(np.abs(array), 95)), max_abs=float(np.abs(array).max()))


def signal_links(legacy: dict, precast: dict, output: Path, start: int, end: int) -> dict:
    frames = [{r["source_us"]: r["evidence"]["pixels"] for r in arm["command_records"]
               if r["evidence"].get("captured")} for arm in (legacy, precast)]
    samples = [{round(r["obs_ts"]*1e6): r for r in _samples(output / name / "navigation_debug.csv")}
               for name in ("legacy", "precast")]
    times = sorted(t for t in samples[0] if start <= t < end)
    if set(times) != {t for t in samples[1] if start <= t < end}:
        raise ValueError("paired law samples differ inside the common window")
    differences = [dict(source_us=t, pixel_u=frames[1][t][0]-frames[0][t][0],
        pixel_v=frames[1][t][1]-frames[0][t][1],
        lateral_rate_deg_s=samples[1][t]["lateral_rate_deg_s"]-samples[0][t]["lateral_rate_deg_s"],
        command_roll_deg=samples[1][t]["cmd_roll_deg"]-samples[0][t]["cmd_roll_deg"])
        for t in times]
    path = output / "paired-differences.json"
    path.write_text(json.dumps(differences, indent=2))
    return dict(direction="precast minus legacy at the same source time", samples=len(times),
        summary={key:signed_summary([r[key] for r in differences]) for key in
                 ("pixel_u", "pixel_v", "lateral_rate_deg_s", "command_roll_deg")},
        at_original_reversal=next((r for r in differences if r["source_us"] == ORIGINAL_REVERSAL_US), None),
        paired_differences_sha256=digest(path))


def image_motion_split(peer: dict, snapshots: list[Snapshot]) -> list[dict]:
    # Symmetric position-first/attitude-first differences, as in the reviewed
    # pre-implementation diagnostic. This is image motion, not the law's rate.
    from simtime_navigation_runtime import BoundaryAttitude, BoundaryTruth
    from navpy.modules.common.models.attitude import Attitude
    from navpy.modules.common.models.location import Location
    from navpy.modules.vision.sim.direct_pixel_render import DirectPixelRenderer
    from navpy.modules.vision.sim.pose_associator import AssociatedPose
    from navpy.modules.vision.sim.sim_camera_ports import FrameSize
    import math
    renderer = DirectPixelRenderer(Location(*peer["dock"], is_absolute=True), source_name="simtime-ideal360",
        aircraft_sequence="ZYX", aircraft_degrees=True, frame_size=FrameSize(1920,1080),
        source_now_s=lambda:0., wall_now_s=lambda:0.)
    def project(snapshot: Snapshot, position: RenderTruth, orientation: RenderTruth) -> np.ndarray:
        obs = snapshot.observation
        time = snapshot.identity.source_us/1e6
        attitude = BoundaryAttitude(Attitude(math.degrees(obs.pitch_rad),0.,math.degrees(obs.roll_rad)),
            time,0.,obs.rates_rad_s)
        truth = BoundaryTruth(Location(position.latitude,position.longitude,position.altitude,is_absolute=True),
            Attitude(orientation.pitch_deg,orientation.yaw_deg,orientation.roll_deg),0.)
        associated = AssociatedPose(attitude,truth,time,0.,0.,obs.airspeed_mps)
        detection = renderer.render(associated,(truth.location,truth.attitude))
        if detection is None: raise ValueError("missing image projection")
        return np.array([detection.pixel.u_px,detection.pixel.v_px])
    captured = [s for s,r in zip(snapshots,peer["records"]) if r["evidence"].get("captured")]
    index = next(i for i,s in enumerate(captured) if s.identity.source_us == ORIGINAL_REVERSAL_US)
    if index < 2: raise ValueError("original reversal lacks preceding frames")
    result = []
    for old,new in zip(captured[index-2:index],captured[index-1:index+1]):
        aa,ba,ab,bb = (project(old,old.truth,old.truth),project(new,new.truth,old.truth),
                      project(new,old.truth,new.truth),project(new,new.truth,new.truth))
        for snapshot,pixel in ((old,aa),(new,bb)):
            if pixel.tolist() != peer["records"][snapshot.identity.step-1]["evidence"]["pixels"]:
                raise ValueError("image split cannot reproduce captured pixels")
        dt = (new.identity.source_us-old.identity.source_us)/1e6
        position,attitude = ((ba-aa)+(bb-ab))/2,((ab-aa)+(bb-ba))/2
        result.append(dict(from_us=old.identity.source_us,to_us=new.identity.source_us,dt_s=dt,
            position_pixel_delta=position.tolist(),attitude_pixel_delta=attitude.tolist(),
            image_azimuth_rate_deg_s=dict(position=position[0]*180/1920/dt,
                attitude=attitude[0]*180/1920/dt,total=(bb-aa)[0]*180/1920/dt)))
    return result


def run(matrix: Path, output: Path) -> dict:
    report = json.loads(matrix.read_text())
    require_matrix(report)
    output.mkdir(parents=True, exist_ok=False)
    run_row = report["runs"][1]  # predeclared first enabled 10x/immediate capture
    directory = local_path(run_row["directory"])
    case = directory / str(run_row["vehicle"])
    peer = json.loads((case / "peer.json").read_text())
    rows, pairs = read_pairs(case, peer)
    recorded = replay_recorded(case, directory, 1, output / "recorded")
    legacy = replay_arm(peer, pairs, output / "legacy", arm="legacy")
    verify_live_rows(case, output / "legacy")
    precast = replay_arm(peer, pairs, output / "precast", arm="precast")
    if legacy["seed_step"] != peer["seed_step"] or precast["seed_step"] != peer["seed_step"]:
        raise ValueError("paired replay changed confirmation seed; compare separately")
    if legacy["first_pass_step"] is None or precast["first_pass_step"] is None:
        raise ValueError("paired replay did not pass; requires separate phase analysis")
    snapshots = [Snapshot.decode(bytes.fromhex(record["snapshot"])) for record in peer["records"]]
    start = snapshots[peer["seed_step"]-1].identity.source_us
    end = snapshots[min(legacy["first_pass_step"], precast["first_pass_step"])-1].identity.source_us
    arms = {}
    for name, result in (("legacy", legacy), ("precast", precast)):
        samples = _samples(output / name / "navigation_debug.csv")
        common = [row for row in samples if start <= round(row["obs_ts"]*1e6) < end]
        own_end = snapshots[result["first_pass_step"]-1].identity.source_us
        own = [row for row in samples if start <= round(row["obs_ts"]*1e6) < own_end]
        arms[name] = dict(common_window=command_metrics(common), own_window=command_metrics(own),
            seed_step=result["seed_step"], first_pass_step=result["first_pass_step"],
            at_original_reversal=reversal(samples, ORIGINAL_REVERSAL_US), causality=result["causality"])
    if (not arms["legacy"]["at_original_reversal"]["available"] or
            round(arms["legacy"]["own_window"]["largest_roll_step"]["source_s"]*1e6) != ORIGINAL_REVERSAL_US):
        raise ValueError("legacy largest reversal differs from the predeclared baseline")
    indices = [i for i, record in enumerate(peer["records"]) if record["evidence"].get("captured")
               and start <= snapshots[i].identity.source_us < end]
    displacements = np.array([pymap3d.geodetic2ned(*pairs[i].precast, *pairs[i].rounded) for i in indices])
    mean = displacements.mean(axis=0)
    bias = dict(mean_ned_m=mean.tolist(), std_about_mean_ned_m=displacements.std(axis=0).tolist(),
        max_abs_ned_m=np.abs(displacements).max(axis=0).tolist(), samples=len(indices))
    origin_changes = []
    for i in range(1, len(rows)):
        if (rows[i]["origin_lat"], rows[i]["origin_lng"]) != (rows[i-1]["origin_lat"], rows[i-1]["origin_lng"]):
            step = pymap3d.geodetic2ned(*pairs[i].precast, *pairs[i-1].precast)
            origin_changes.append(dict(source_us=snapshots[i].identity.source_us, precast_step_ned_m=list(step)))
    result = dict(matrix_sha256=digest(matrix), method_sha256={name:digest(ROOT / "scripts" / name)
        for name in ("pose_rounding_analysis.py", "replay_pose_rounding.py", "pose_rounding_evidence.py")},
        case=str(case), exact_recorded_replay=recorded["exact_replay"], exact_legacy_control=legacy["exact_control"],
        firmware_contract_sha256=digest(FIRMWARE),
        signal_links=signal_links(legacy,precast,output,start,end),
        original_image_motion_split=image_motion_split(peer,snapshots),
        common_window_us=[start, end], arms=arms, displacement=bias,
        branch_counts=dict(Counter(row["branch"] for row in rows)), origin_changes=origin_changes,
        evidence_sha256={name:digest(case / name) for name in ("peer.json", "navpy-pose.csv", "identity.json")},
        limits="Direct rounding-to-renderer-to-law path on one recorded trajectory. Mean displacement and varying error both change. Recorded attitude/trajectory retain closed-loop feedback. No claim of closed-loop stability, approach improvement or docking.")
    (output / "analysis.json").write_text(json.dumps(result, indent=2))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = run(args.matrix, args.output)
    print(json.dumps(dict(arms=result["arms"], displacement=result["displacement"]), indent=2))


if __name__ == "__main__":
    main()
