"""Semantic guards for scheduler cadence and pure-vision final approach."""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/navpy"
TERMINAL = ROOT / "src/navpy/modules/navigation/nav/vision_nav"
SCHEDULER = "src/navpy/modules/common/scheduler_cadence.py"
EXECUTOR = "src/navpy/modules/navigation/nav/vision_nav/command_executor.py"
SIM_ALIASES = {
    "src/navpy/modules/common/sim_time.py",
    "src/navpy/modules/vision/sim/sim_timebase.py",
}
LEGACY_APIS = {"wall_period_for_sim_period", "loop_sim_speedup"}
CORE = {
    "frame.py": "TerminalVisionFrame", "frame_projection.py": "TerminalFrameProjector",
    "source_epoch.py": "SourceEpochLedger", "confirmation.py": "TerminalConfirmation",
    "ingress.py": "TerminalIngress", "rate_filter.py": "VerticalRateFilter",
    "law.py": "VisionNavLaw", "visual_pass.py": "VisualPassDetector",
    "command_transaction.py": "TerminalCommandTransaction",
    "lateral_rate.py": "LateralRateFilter",
    "law_config.py": "TerminalLawConfig",
    "command_anchor.py": "TerminalCommandAnchor",
    "law_plan.py": "TerminalLawPlan",
}
COMMAND_INPUT_FILES = tuple(CORE) + (
    "command_freshness.py",
    "command_hold.py",
    "runtime.py",
)
FRAME_FIELDS = (
    "source_name", "source_generation", "task_id", "obj_id", "source_timestamp_s",
    "body_x", "body_y", "body_z", "control_x", "control_y", "control_z",
    "aircraft_roll_deg", "air_speed_mps", "aircraft_yaw_rate_rad_s",
    "aircraft_pitch_deg",
)
# The ONLY yaw-token identifiers the command core may touch, per reviewed file.
# These name an instantaneous gyro-derived body-rate conversion (owner ruling:
# fixed-bias instantaneous rates are legal; integrated/compass heading is not).
# Provenance of the value is pinned by test_yaw_rate_is_gyro_conversion_only.
# Yaw ANGLES, headings, and the canonical name in any other file stay forbidden.
GYRO_RATE_ALLOWANCES = {
    "frame.py": frozenset({"aircraft_yaw_rate_rad_s"}),
    "frame_projection.py": frozenset({"aircraft_yaw_rate_rad_s", "yaw_rate"}),
    "lateral_rate.py": frozenset({"aircraft_yaw_rate_rad_s", "yaw_rate_rad_s"}),
}
# Truth-position vocabulary. AGENTS.md allows frame-local vision LOS/pixels,
# gimbal/camera readback as camera state, pitch/roll attitude, airspeed, wind,
# and previous command state -- never a world-frame position of the target or
# the aircraft. Abbreviations fold first, so `tgt_height` is judged as
# `reference_height_m` and `tgt_pos` as `target_position`.
TRUTH_ALIASES = {
    "tgt": "target", "pos": "position", "posn": "position",
    "positions": "position", "loc": "location", "coord": "coordinate",
    "coords": "coordinate", "coordinates": "coordinate",
}
POSITION_WORDS = {"position", "coordinate"}
# A pixel/image position is frame-local vision output, so it stays legal --
# but only while it names no truth subject. `pixel_target_position` is a
# world-frame target position wearing a frame-local prefix, not vision output.
FRAME_LOCAL_POSITION_WORDS = {"pixel", "pixels", "image"}
TRUTH_SUBJECT_WORDS = {"target", "object", "detection", "vehicle", "aircraft",
                       "own"}
# World/earth frames, world axes, and truth sources. No such token exists in
# the command core today; each names a quantity the command path must never
# read. Frame-local vision spells its axes `body_x`/`control_y`, never
# north/east/down.
WORLD_FRAME_WORDS = {
    "ned", "enu", "ecef", "lla", "wgs84", "utm", "waypoint", "lon", "lng",
    "latlon", "latlng", "north", "northing", "east", "easting", "down",
    "gps", "home", "truth", "groundtruth", "sim", "simstate",
    # World-frame qualifiers. These catch a truth position whose position word
    # is spelled as a raw axis tuple -- `target_world_xyz` names no banned
    # position token, so only the frame qualifier gives it away. `inertial` is
    # deliberately NOT here: `inertial_los_rate_deg_s` and `raw_inertial_rate`
    # are frame-local LOS rates and legal command inputs.
    "world", "global", "earth", "absolute", "geodetic",
}
# Direct range to the target in the spellings the `range` rule alone misses.
# Unconditional, deliberately: the pre-existing `range` rule already rejects
# `pixel_range`/`range_px` with no frame-local exemption, so exempting these
# synonyms would rebuild the very spelling asymmetry this guard exists to
# close -- reject `pixel_range`, allow `pixel_distance`. The command core
# already spells a frame-local pixel magnitude `norm`, never distance:
# `horizontal_norm = math.hypot(...)` in law.py and `norm = math.sqrt(...)`
# in frame.py. If this rejects a genuinely frame-local quantity, rename it to
# `..._norm`; loosening the whole range family is an owner decision.
RANGE_WORDS = {"distance", "dist", "slant"}
# `lat` is this repo's abbreviation for LATERAL (`resolve_lat_gyro_term`), so
# latitude is caught only beside a coordinate unit or a truth subject. A bare
# `lat` stays legal -- that ambiguity is the accepted residual.
COORDINATE_UNIT_WORDS = {"deg", "degrees", "rad", "radians", "e7"}
YAW_RATE_CONVERSION = "src/navpy/modules/vision/models/pixel_observation.py"
YAW_RATE_PRODUCERS = (
    "src/navpy/modules/vision/real_detected_object_builder.py",
    "src/navpy/modules/vision/sim/finite_target_projector.py",
    "src/navpy/modules/vision/sim/ideal_target_projector.py",
)
CLOCK_MODULES = {"time", "timeit", "datetime"}
CLOCK_CALLS = {
    "time", "time_ns", "monotonic", "monotonic_ns", "perf_counter",
    "perf_counter_ns", "process_time", "default_timer", "now", "utcnow",
}
NON_PERIOD_WORDS = {
    "source", "timestamp", "age", "dt", "clock", "elapsed", "now",
    "frame", "observation", "sensor", "pose",
}

def _production() -> list[tuple[str, ast.Module]]:
    return [
        (path.relative_to(ROOT).as_posix(), ast.parse(path.read_text("utf-8")))
        for path in sorted(SRC.rglob("*.py"))
    ]

def _path(node: ast.AST) -> tuple[str, ...]:
    if isinstance(node, ast.Name):
        return (node.id,)
    return (*_path(node.value), node.attr) if isinstance(node, ast.Attribute) else ()

def _parents(tree: ast.AST) -> dict[ast.AST, ast.AST]:
    return {child: parent for parent in ast.walk(tree)
            for child in ast.iter_child_nodes(parent)}

def _ancestor(node: ast.AST, parents: dict, kind: type | tuple[type, ...]):
    node = parents.get(node)
    while node is not None and not isinstance(node, kind):
        node = parents.get(node)
    return node

def _definition(tree: ast.AST, name: str, kind: type | tuple[type, ...]):
    return next((node for node in ast.walk(tree)
                 if isinstance(node, kind) and node.name == name), None)

def _node_names(node: ast.AST) -> tuple[str, ...]:
    if isinstance(node, ast.Name):
        return (node.id,)
    if isinstance(node, ast.Attribute):
        return (node.attr,)
    if isinstance(node, ast.alias):
        return tuple(name for name in (node.name, node.asname) if name)
    if isinstance(node, (ast.ClassDef, ast.FunctionDef)):
        return (node.name,)
    return ()

def _identifiers(node: ast.AST) -> set[str]:
    return {name for child in ast.walk(node) for name in _node_names(child)}

def _tokens(name: str) -> set[str]:
    snake = ""
    for index, char in enumerate(name):
        if index and char.isupper() and name[index - 1].islower():
            snake += "_"
        snake += char.lower() if char.isalnum() else "_"
    return set(filter(None, snake.split("_")))

def _references(tree: ast.Module, forbidden: set[str]) -> list[str]:
    return [f"{name}@{getattr(node, 'lineno', 0)}"
            for node in ast.walk(tree) for name in _node_names(node)
            if name in forbidden]

def _calls(tree: ast.AST, *suffix: str) -> list[ast.Call]:
    return [node for node in ast.walk(tree) if isinstance(node, ast.Call)
            and (not suffix or _path(node.func)[-len(suffix):] == suffix)]

def _clock_hits(tree: ast.Module) -> list[str]:
    hits = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            hits += [
                f"import {item.name}@{node.lineno}" for item in node.names
                if item.name.split(".")[0] in CLOCK_MODULES
            ]
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            if module.split(".")[0] in CLOCK_MODULES:
                hits.append(f"from {module}@{node.lineno}")
        elif isinstance(node, ast.Call):
            call = _path(node.func)
            if call and call[-1] in CLOCK_CALLS:
                hits.append(f"{'.'.join(call)}@{node.lineno}")
            if call == ("getattr",) and len(node.args) > 1:
                member = node.args[1]
                if isinstance(member, ast.Constant) and member.value in CLOCK_CALLS:
                    hits.append(f"getattr({member.value})@{node.lineno}")
    return hits

def _fold_aliases(words: set[str]) -> set[str]:
    return {TRUTH_ALIASES.get(word, word) for word in words}


def _is_forbidden_input(name: str, context: tuple[str, ...]) -> bool:
    words = _fold_aliases(_tokens(name))
    surrounding = set().union(*map(_tokens, context))
    return any((
        "heading" in words,
        "yaw" in words and not surrounding & {"camera", "gimbal"},
        "velocity" in words or {"ground", "speed"} <= words,
        bool(words & {"altitude", "agl", "amsl"}) or {"alt", "rate"} <= words,
        bool(words & {"location", "latitude", "longitude", "geo", "range"}),
        "bbox" in name.lower() or {"bounding", "box"} <= words,
        "height" in words and bool(words & {"target", "object", "detection", "bbox"}),
        bool(words & POSITION_WORDS) and not (
            words & FRAME_LOCAL_POSITION_WORDS and not words & TRUTH_SUBJECT_WORDS
        ),
        bool(words & WORLD_FRAME_WORDS) or bool(words & RANGE_WORDS),
        "lat" in words and bool(
            words & (COORDINATE_UNIT_WORDS | TRUTH_SUBJECT_WORDS)
        ),
    ))

def _forbidden_inputs(
    tree: ast.Module, allowed: frozenset[str] = frozenset()
) -> list[str]:
    parents, hits = _parents(tree), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            name, context = node.attr, _path(node)
        elif isinstance(node, (ast.Name, ast.arg)):
            name = node.id if isinstance(node, ast.Name) else node.arg
            context = (name,)
            if name == "range" and isinstance(parents.get(node), ast.Call):
                continue
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            parent = parents.get(node)
            if not isinstance(parent, ast.Call) or _path(parent.func) != ("getattr",):
                continue
            name, context = node.value, (node.value,)
        else:
            continue
        if name not in allowed and _is_forbidden_input(name, context):
            hits.add(f"{name}@{node.lineno}")
    return sorted(hits)

def _allowed_speedup_division(relative: str, node: ast.BinOp, owner) -> bool:
    return (relative == SCHEDULER and owner is not None
            and owner.name == "wall_period_for_scheduler_period"
            and isinstance(node.op, ast.Div) and _path(node.left) == ("period",)
            and _path(node.right) == ("self", "_speedup"))

def _cadence_violations(trees: list[tuple[str, ast.Module]]) -> list[str]:
    hits = []
    for relative, tree in trees:
        parents = _parents(tree)
        for node in ast.walk(tree):
            if isinstance(node, ast.BinOp) and isinstance(
                node.op, (ast.Mult, ast.Div, ast.FloorDiv, ast.Pow)
            ) and any("speedup" in name.lower() for name in _identifiers(node)):
                owner = _ancestor(node, parents, (ast.FunctionDef, ast.AsyncFunctionDef))
                if not _allowed_speedup_division(relative, node, owner):
                    hits.append(f"{relative}:{node.lineno} speedup scaling")
            if (isinstance(node, ast.Attribute) and node.attr == "current_speedup"
                    and relative != SCHEDULER):
                hits.append(f"{relative}:{node.lineno} raw speedup")
            if not (isinstance(node, ast.Call)
                    and _path(node.func)[-1:] == ("wall_period_for_scheduler_period",)):
                continue
            words = (
                set().union(*map(_tokens, _identifiers(node.args[0])))
                if len(node.args) == 1 else set()
            )
            if (len(node.args) != 1 or node.keywords or "period" not in words
                    or words & NON_PERIOD_WORDS):
                hits.append(f"{relative}:{node.lineno} non-period cadence input")
    return hits

def test_sim_timebase_is_only_an_exact_compatibility_alias() -> None:
    trees = dict(_production())
    expected = ast.parse(
        "from navpy.modules.common.scheduler_cadence import SchedulerCadence\n"
        "SimTimebase = SchedulerCadence\n__all__ = ['SimTimebase']\n")
    assert SIM_ALIASES <= trees.keys()
    for relative, tree in trees.items():
        if relative not in SIM_ALIASES:
            assert not _references(tree, {"SimTimebase"}), relative
            continue
        body = [
            node for node in tree.body
            if not (isinstance(node, ast.Expr)
                    and isinstance(node.value, ast.Constant)
                    and isinstance(node.value.value, str))
        ]
        actual = ast.Module(body=body, type_ignores=[])
        assert ast.dump(actual, include_attributes=False) == ast.dump(
            expected, include_attributes=False), relative

def test_legacy_apis_are_absent_and_speedup_changes_only_cadence() -> None:
    trees = _production()
    legacy = [
        f"{relative}:{hit}" for relative, tree in trees
        for hit in _references(tree, LEGACY_APIS)
    ]
    assert not legacy
    scheduler = dict(trees)[SCHEDULER]
    owner = _definition(scheduler, "SchedulerCadence", ast.ClassDef)
    period = _definition(owner, "wall_period_for_scheduler_period", ast.FunctionDef)
    assert owner is not None and period is not None
    assert [arg.arg for arg in period.args.args] == ["self", "scheduler_period_s"]
    assert not _cadence_violations(trees)

def test_cadence_guard_rejects_scaled_dt_and_source_timestamp() -> None:
    bad = ast.parse("""
def bad(cadence, frame, dt_s, speedup):
    scaled_dt = dt_s * speedup
    return cadence.wall_period_for_scheduler_period(frame.source_timestamp_s)
""")
    assert len(_cadence_violations([("synthetic.py", bad)])) == 2

def test_terminal_scope_and_primitive_frame_cannot_disappear() -> None:
    trees = {path.name: ast.parse(path.read_text("utf-8"))
             for path in sorted(TERMINAL.glob("*.py"))}
    assert trees
    for filename, class_name in CORE.items():
        assert filename in trees
        assert _definition(trees[filename], class_name, ast.ClassDef) is not None
    frame = _definition(trees["frame.py"], "TerminalVisionFrame", ast.ClassDef)
    fields = [
        (node.target.id, ast.unparse(node.annotation)) for node in frame.body
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name)
    ]
    assert tuple(name for name, _ in fields) == FRAME_FIELDS
    assert {annotation for _, annotation in fields} <= {"str", "int", "float"}

def test_terminal_command_core_has_no_clock_or_forbidden_truth_inputs() -> None:
    clocks, inputs = [], []
    for filename in CORE:
        tree = ast.parse((TERMINAL / filename).read_text("utf-8"))
        clocks += [f"{filename}:{hit}" for hit in _clock_hits(tree)]
    for filename in COMMAND_INPUT_FILES:
        tree = ast.parse((TERMINAL / filename).read_text("utf-8"))
        allowed = GYRO_RATE_ALLOWANCES.get(filename, frozenset())
        inputs += [f"{filename}:{hit}" for hit in _forbidden_inputs(tree, allowed)]
    assert not clocks, "terminal clock reads:\n  " + "\n  ".join(clocks)
    assert not inputs, "forbidden terminal inputs:\n  " + "\n  ".join(inputs)


def test_hold_wall_clock_is_execution_diagnostics_only() -> None:
    tree = ast.parse((TERMINAL / "command_hold.py").read_text("utf-8"))
    freshness_tree = ast.parse(
        (TERMINAL / "command_freshness.py").read_text("utf-8")
    )
    owner = _definition(tree, "TerminalCommandHold", ast.ClassDef)
    issue = _definition(owner, "issue", ast.FunctionDef)
    freshness = _definition(freshness_tree, "_age_is_fresh", ast.FunctionDef)
    assert owner is not None and issue is not None and freshness is not None
    assert "_monotonic_s" not in _identifiers(freshness)
    clocks = _calls(issue, "_monotonic_s")
    assert len(clocks) == 2

def test_terminal_guard_rejects_alias_clock_and_hidden_truth() -> None:
    bad = ast.parse("""
import time as hidden
def decide(vehicle, frame, target_range_m):
    tick = hidden.monotonic()
    heading = getattr(vehicle, "heading")
    return frame.ground_speed_ned, target_range_m, tick, heading
""")
    assert _clock_hits(bad)
    found = {hit.split("@", 1)[0] for hit in _forbidden_inputs(bad)}
    assert {"heading", "ground_speed_ned", "target_range_m"} <= found

def test_yaw_rate_is_gyro_conversion_only() -> None:
    """Pin the provenance of `aircraft_yaw_rate_rad_s` to the body-rate gyro.

    The frame's yaw RATE is legal only because it is an instantaneous
    conversion of body q/r gyro rates through roll/pitch — never compass
    heading, never an integral. This pins: (1) the one canonical conversion,
    its signature, and that its body touches no heading/yaw identifiers;
    (2) that it is defined nowhere else in production; (3) that every producer
    fills the frame keyword with a direct call to that conversion.
    """
    conversion = ast.parse((ROOT / YAW_RATE_CONVERSION).read_text("utf-8"))
    definition = _definition(conversion, "aircraft_yaw_rate_rad_s", ast.FunctionDef)
    assert definition is not None
    assert [arg.arg for arg in definition.args.args] == [
        "pitch_deg", "roll_deg", "body_rates_rad_s",
    ]
    names = _identifiers(definition)
    assert not any("heading" in _tokens(name) for name in names)
    assert {n for n in names if "yaw" in _tokens(n)} == {"aircraft_yaw_rate_rad_s"}
    definitions = [
        f"{relative}" for relative, tree in _production()
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "aircraft_yaw_rate_rad_s"
    ]
    assert definitions == [YAW_RATE_CONVERSION]
    for producer in YAW_RATE_PRODUCERS:
        tree = ast.parse((ROOT / producer).read_text("utf-8"))
        imported = any(
            isinstance(node, ast.ImportFrom)
            and node.module == "navpy.modules.vision.models.pixel_observation"
            and any(alias.name == "aircraft_yaw_rate_rad_s" and alias.asname is None
                    for alias in node.names)
            for node in ast.walk(tree)
        )
        assert imported, producer
        keywords = [
            keyword for call in _calls(tree) for keyword in call.keywords
            if keyword.arg == "aircraft_yaw_rate_rad_s"
        ]
        assert len(keywords) == 1, producer
        value = keywords[0].value
        assert isinstance(value, ast.Call), producer
        assert _path(value.func) == ("aircraft_yaw_rate_rad_s",), producer


def test_gyro_rate_allowance_is_exact_and_file_scoped() -> None:
    angle = ast.parse(
        "def f(frame):\n"
        "    return frame.aircraft_yaw_deg, frame.yaw_rate_dps, frame.heading_rate_rad_s\n"
    )
    hits = {hit.split("@", 1)[0]
            for hit in _forbidden_inputs(angle, GYRO_RATE_ALLOWANCES["frame.py"])}
    assert {"aircraft_yaw_deg", "yaw_rate_dps", "heading_rate_rad_s"} <= hits
    elsewhere = ast.parse("def f(frame):\n    return frame.aircraft_yaw_rate_rad_s\n")
    assert _forbidden_inputs(elsewhere)
    assert set(GYRO_RATE_ALLOWANCES) <= set(COMMAND_INPUT_FILES)


def test_executor_perf_counter_is_deferred_diagnostics_only() -> None:
    tree = ast.parse((ROOT / EXECUTOR).read_text("utf-8"))
    owner = _definition(tree, "TerminalCommandExecutor", ast.ClassDef)
    execute = _definition(owner, "execute", ast.FunctionDef)
    execute_current = _definition(owner, "_execute_current", ast.FunctionDef)
    recorder = _definition(owner, "_record_source_time", ast.FunctionDef)
    assert (
        owner is not None
        and execute is not None
        and execute_current is not None
        and recorder is not None
    )
    parents = _parents(tree)
    clocks = [call for call in _calls(owner)
              if _path(call.func)[-1:] in {(name,) for name in CLOCK_CALLS}]
    assert len(clocks) == 2
    assert {_path(call.func) for call in clocks} == {("time", "perf_counter")}
    transaction = _calls(execute_current, "transaction", "execute")
    assert len(transaction) == 1
    assert [_path(arg) for arg in transaction[0].args] == [("work", "frame")]
    assert not transaction[0].keywords
    assert not set(ast.walk(transaction[0])) & set(clocks)
    diagnostic = _calls(execute, "_record_source_time")
    assert len(diagnostic) == 1
    callback = _ancestor(diagnostic[0], parents, ast.Lambda)
    push = parents.get(callback) if callback is not None else None
    assert isinstance(push, ast.Call)
    assert _path(push.func)[-2:] == ("cleanup", "push")
    entry = [call for call in clocks
             if _ancestor(call, parents, ast.FunctionDef) is execute]
    assert len(entry) == 1
    assignment = _ancestor(entry[0], parents, ast.Assign)
    assert isinstance(assignment, ast.Assign) and [_path(target) for target
                                                   in assignment.targets] == [("entry_s",)]
    entry_uses = [node for node in ast.walk(execute) if isinstance(node, ast.Name)
                  and node.id == "entry_s" and isinstance(node.ctx, ast.Load)]
    assert entry_uses and all(_ancestor(node, parents, ast.Call) is diagnostic[0]
                              for node in entry_uses)
    completion = [call for call in clocks
                  if _ancestor(call, parents, ast.FunctionDef) is recorder]
    assert len(completion) == 1
    keyword = _ancestor(completion[0], parents, ast.keyword)
    record_call = parents.get(keyword) if keyword is not None else None
    assert isinstance(keyword, ast.keyword) and keyword.arg == "execution_ms"
    assert isinstance(record_call, ast.Call)
    assert _path(record_call.func)[-2:] == ("source_time", "record_command")


def test_command_input_scan_catches_injected_target_position(tmp_path) -> None:
    """Truth position added to a command-input file must not slip the scan.

    Only `TerminalVisionFrame`'s field list is frozen name-by-name. Every other
    command-input file is protected by token scanning alone, so a truth
    position spelled `target_position` is exactly the shape a regression takes.
    Copy a real command-input file, splice the reference in, and re-run the
    production scan over the copy.
    """
    original = (TERMINAL / "law.py").read_text("utf-8")
    allowed = GYRO_RATE_ALLOWANCES.get("law.py", frozenset())
    assert not _forbidden_inputs(ast.parse(original), allowed)
    injected = original.replace(
        "class VisionNavLaw:",
        "class VisionNavLaw:\n"
        "    def _leak(self, frame):\n"
        "        return frame.target_position\n",
        1,
    )
    assert injected != original
    fixture = tmp_path / "law.py"
    fixture.write_text(injected, "utf-8")
    hits = _forbidden_inputs(ast.parse(fixture.read_text("utf-8")), allowed)
    assert [hit for hit in hits if hit.startswith("target_position@")], hits


def test_guard_rejects_truth_position_spellings() -> None:
    """Every world-frame position spelling the command path could reach for."""
    bad = ast.parse("""
def leak(frame, lat_deg, tgt_pos, sim_state, dist_to_tgt):
    return (
        frame.target_position, frame.position_ned, frame.truth_position,
        frame.home_position, frame.gps_coords, frame.latlon_deg,
        frame.lon_deg, frame.ecef_x, frame.target_coordinate,
        frame.tgt_height_px, lat_deg, tgt_pos, sim_state,
        frame.target_north_m, frame.target_east_m, frame.target_down_m,
        frame.distance_to_target, frame.slant_dist, frame.target_lat,
        frame.pixel_target_position, dist_to_tgt,
        frame.target_world_xyz, frame.world_x, frame.earth_frame_x,
        frame.absolute_target_x,
        getattr(frame, "target_pos"),
    )
""")
    hits = {hit.split("@", 1)[0] for hit in _forbidden_inputs(bad)}
    assert {
        "target_position", "position_ned", "truth_position", "home_position",
        "gps_coords", "latlon_deg", "lon_deg", "ecef_x", "target_coordinate",
        "tgt_height_px", "lat_deg", "tgt_pos", "sim_state", "target_pos",
        "target_north_m", "target_east_m", "target_down_m", "target_lat",
        "distance_to_target", "slant_dist", "dist_to_tgt",
        "pixel_target_position", "target_world_xyz", "world_x",
        "earth_frame_x", "absolute_target_x",
    } <= hits


def test_truth_position_guard_spares_frame_local_and_lateral_names() -> None:
    """Pin the names the widened rules must never flag.

    `lat` is the codebase's abbreviation for LATERAL, not latitude, so it is
    forbidden only beside a coordinate unit or a truth subject. Pixel/image
    positions are frame-local vision output and stay legal per AGENTS.md --
    the companion `pixel_target_position` case in the reject test pins that
    the exemption does not launder a truth subject.
    """
    good = ast.parse("""
def keep(frame, state, next_state, origin, lat_gyro_term):
    return (
        LAT_GYRO_TERM_ENV, LAT_GYRO_DELAY_ENV, resolve_lat_gyro_delay_s,
        LateralRateState, VisualPassState, TerminalPlanOrigin, DetectedObject,
        frame.target_passed_override, frame.pixel_position, frame.image_pos,
        frame.body_x, frame.control_y, frame.air_speed_mps,
        frame.aircraft_pitch_deg, frame.aircraft_roll_deg,
        frame.inertial_los_rate_deg_s, frame.raw_inertial_rate_rad_s,
        abs(state), next_state, origin, lat_gyro_term,
    )
""")
    assert not _forbidden_inputs(good)
