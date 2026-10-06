"""Source-derived SOLID limits used by the architecture guard tests."""

from __future__ import annotations

import ast
import hashlib
import io
import subprocess
import tarfile
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping


PINNED_BASE = "a2ab2fcb7cbd34b4589e7f0754cfe6eca3bd7ba6"
MATERIAL_ROOTS = ("src/navpy", "src/gcs", "scripts", "tools")
MAX_CLASS_LINES = 200
MAX_DIRECT_METHODS = 15
MAX_OWNED_FIELDS = 12
MAX_CONSTRUCTOR_DEPENDENCIES = 12
MAX_MATERIAL_MODULE_LINES = 300
MAX_TOP_LEVEL_FUNCTION_LINES = 150

# Frozen size-debt ceilings for the NAMED experiment harnesses only, at their
# measured 2026-08-17 sizes. Deliberately stricter than exclusion: any growth
# past the recorded size fails, shrinking is always legal, and a file without
# an entry gets the normal limits. Operational launchers (gcs_launch.py,
# upload_north_line_mission.py), src/navpy, src/gcs and tools receive NO entry
# and stay on the strict limits. Ratchet an entry DOWN when its harness
# shrinks; never raise one without an owner decision.
EXPERIMENT_MODULE_LINE_DEBT = {
    # +1 each for the one-line pre-arm origin gate; both files sat exactly
    # at their ceiling, and the gate's own logic lives in eval_nav_origin.
    "scripts/eval_direct_pixel_pn.py": 707,
    "scripts/eval_direct_pixel_pn_three_uav.py": 340,
    "scripts/plant_workbook.py": 687,
    "scripts/scratch_direct_geo_child.py": 317,
    # +6 for the gate's bound in `worst_case_wall_s`: `_arm` now holds two
    # waits, and the parent sizes its kill timer from that sum.
    "scripts/scratch_navigation_uav.py": 1137,
    "scripts/scratch_plant_id_child.py": 595,
    "scripts/scratch_plant_id_eval.py": 313,
    "scripts/scratch_plant_matrix_child.py": 573,
    "scripts/scratch_plant_matrix_eval.py": 479,
    "scripts/scratch_sitl_scale.py": 1569,
    # +23 for the same gate on the scratch arm path. The eval call sites take
    # the reasoning by import; `_arm` is reached from two harnesses that both
    # turn ARMING_CHECK off, so it carries the WHY and the target_system
    # wiring the gate needs. This file was also exactly at its ceiling.
    "scripts/scratch_sitl_uav.py": 806,
    "scripts/sitl_truth_pose.py": 530,
    "scripts/siyi_geo_track_child.py": 352,
    "scripts/siyi_pixel_pn_child.py": 327,
}
EXPERIMENT_FUNCTION_LINE_DEBT = {
    ("scripts/eval_direct_pixel_pn.py", "run_case"): 215,
    ("scripts/scratch_direct_geo_eval.py", "run_case"): 181,
    ("scripts/scratch_navigation_uav.py", "run_navigation_episode"): 319,
    ("scripts/scratch_navigation_uav.py", "run"): 151,
    ("scripts/scratch_sitl_scale.py", "_run_level"): 600,
    ("scripts/scratch_sitl_uav.py", "run"): 246,
}

# Lifecycle/orchestration owners refactored in this change have a tighter
# boundary than the legacy global ceiling. This prevents responsibility from
# accumulating back into the controllers while broader legacy cleanup proceeds.
FOCUSED_CLASS_LINE_LIMITS = {
    (
        "src/navpy/modules/nav/nav_network.py",
        "NavNetworkRuntime",
    ): 150,
    (
        "src/navpy/modules/nav/peer_poi_dispatch.py",
        "PeerPoiDispatchWorker",
    ): 150,
    (
        "src/navpy/modules/nav/final_approach_detection_event_pump.py",
        "FinalApproachDetectionEventPump",
    ): 150,
    (
        "src/navpy/modules/swarm/swarm_presence.py",
        "SwarmPresence",
    ): 150,
    (
        "src/navpy/modules/vision/frame_capture_runtime.py",
        "FrameCaptureRuntime",
    ): 150,
    (
        "src/navpy/modules/vision/frame_capture_ffmpeg.py",
        "FfmpegFrameCapture",
    ): 150,
    (
        "src/navpy/modules/vision/gimbal_telemetry_publisher.py",
        "GimbalTelemetryPublisher",
    ): 150,
    (
        "src/navpy/modules/vision/real_detector_lifecycle.py",
        "DetectorLifecycle",
    ): 150,
    (
        "src/navpy/modules/vision/vision_lifecycle.py",
        "VisionLifecycle",
    ): 150,
    (
        "src/navpy/modules/vision/peripheral/siyi/hardware/lifecycle.py",
        "SiyiHardwareLifecycle",
    ): 150,
    (
        "src/navpy/modules/vision/peripheral/camera_intrinsics.py",
        "CameraIntrinsics",
    ): 150,
    (
        "src/navpy/modules/vision/sim/sim_detector_loop.py",
        "SimDetectorWorker",
    ): 150,
}

VENDORED_SHA256 = {
    "src/navpy/modules/vehicle/_mavftp/_upstream_mavftp.py":
        "6977ACE6D7FF6DE95AF5FF63FEB1883F3863C76266A2FF6E1607C86312BCBE9F",
    "src/navpy/modules/vision/peripheral/siyi/siyi_sdk.py":
        "DCEBF8A3BB4EFAE396D213236918CB66D1C95D67816B8681A377255E4A8C8843",
    "src/navpy/modules/vision/peripheral/siyi/siyi_message.py":
        "BB8E3BAAD0827FC41459FB488DE8C356E2050432AD25A8CDC3DFA97DCEAF904D",
}

# These are immutable schema records, not behavior owners. The exact path,
# qualified name, frozen shape, and current maximum prevent name-based escape.
# A record stays passive with `__post_init__` validation, `@property` derived
# views, and `@staticmethod` helpers -- none of those mutate or own behavior on
# a frozen dataclass. Any other method disqualifies the allowance.
PASSIVE_RECORD_FIELD_LIMITS = {
    ("src/navpy/logger/navigation_sample.py", "NavigationSample"): 16,
    ("src/navpy/modules/vision/mot_state.py", "TrackedObject"): 14,
    # 17 = the original 16 plus ONE nested frozen CaptureTiming record added
    # by the capture-time association change; capture-domain times are one
    # value-object field, not four loose floats.
    (
        "src/navpy/modules/vision/real_frame_association.py",
        "RealFrameAssociation",
    ): 17,
    ("src/navpy/modules/vision/track_export_memory.py", "_MemoryState"): 13,
    (
        "src/navpy/modules/navigation/nav/vision_nav/frame.py",
        "FinalApproachVisionFrame",
    ): 15,
    (
        "src/navpy/modules/navigation/nav/vision_nav/command_transaction.py",
        "FinalApproachLawEvidence",
    ): 16,
}


@dataclass(frozen=True)
class FieldAllowance:
    maximum: int
    required_base: str


# Launcher is the Tk composition root and owns the exact widget/state bundle.
# It gets one extra field, not an open-ended UI or public-adapter exemption.
FIELD_ALLOWANCES = {
    ("src/navpy/main_gui.py", "Launcher"): FieldAllowance(13, "tk.Tk"),
}


@dataclass(frozen=True)
class FacadeMethodAllowance:
    maximum_direct: int
    maximum_effective: int
    required_fields: frozenset[str]


# These exact public compatibility boundaries deliberately expose broad APIs,
# while all behavior and state live in focused collaborators behind one parts
# field. Counting their imported facets prevents mixins from becoming a general
# escape hatch; only these path/name/shape-bounded adapters receive an allowance.
FACADE_METHOD_ALLOWANCES = {
    (
        "src/navpy/modules/navigation/gimbal_navigation.py",
        "GimbalNavigation",
    ): FacadeMethodAllowance(1, 20, frozenset({"_parts"})),
    # 82 = 80 + the two capture-pose accessors (attitude_history,
    # link_identity) added by the approved capture-time association design
    # (§4 plumbing) -- a documented ratchet, not headroom.
    # 83 = 82 + on_admission, the admission tap's one method (D1 of the
    # LANDING2 step-1 plan), approved by the
    # owner on 2026-09-10 -- the same kind of ratchet, not headroom.
    (
        "src/navpy/modules/vehicle/vehicle_mav.py",
        "VehicleMav",
    ): FacadeMethodAllowance(1, 83, frozenset({"_parts"})),
    (
        "src/navpy/modules/vision/camera_mount.py",
        "CameraMount",
    ): FacadeMethodAllowance(
        1,
        44,
        frozenset({
            "_optics_lock",
            "_parts",
            "camera",
            "gimbal",
            "name",
            "zoom_calibration",
        }),
    ),
    (
        "src/navpy/modules/vision/detection_coordinator.py",
        "DetectionCoordinator",
    ): FacadeMethodAllowance(1, 31, frozenset({"_parts"})),
    (
        "src/navpy/modules/vision/detector.py",
        "Detector",
    ): FacadeMethodAllowance(1, 40, frozenset({"_parts"})),
    (
        "src/navpy/modules/vision/poi_zoom_orchestrator.py",
        "PoiZoomTracker",
    ): FacadeMethodAllowance(1, 17, frozenset({"_parts"})),
    (
        "src/navpy/modules/vision/vision_controller.py",
        "VisionController",
    ): FacadeMethodAllowance(16, 16, frozenset({"_ports"})),
    (
        "src/navpy/modules/vision/peripheral/gimbal_siyi.py",
        "GimbalSiyi",
    ): FacadeMethodAllowance(1, 22, frozenset({"_parts"})),
    (
        "src/navpy/modules/vision/peripheral/siyi/sim/gimbal_physics.py",
        "GimbalPhysics",
    ): FacadeMethodAllowance(1, 20, frozenset({"_parts"})),
    (
        "src/navpy/modules/vision/peripheral/siyi/sim/gimbal_siyi_sim.py",
        "GimbalSiyiSim",
    ): FacadeMethodAllowance(1, 22, frozenset({"_parts"})),
    (
        "src/navpy/modules/vision/sim/detector_sim.py",
        "DetectorSim",
    ): FacadeMethodAllowance(1, 36, frozenset({"_parts"})),
}


@dataclass(frozen=True)
class Violation:
    code: str
    relative: str
    line: int
    owner: str
    detail: str

    def render(self) -> str:
        location = f"{self.relative}:{self.line}" if self.line else self.relative
        owner = f" {self.owner}" if self.owner else ""
        return f"{location}{owner}: {self.detail} [{self.code}]"


@dataclass(frozen=True)
class Definition:
    qualname: str
    node: ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef


def read_current_sources(repo_root: Path) -> dict[str, str]:
    sources: dict[str, str] = {}
    for root in MATERIAL_ROOTS:
        directory = repo_root / root
        if not directory.is_dir():
            continue
        for path in directory.rglob("*.py"):
            relative = path.relative_to(repo_root).as_posix()
            sources[relative] = path.read_bytes().decode("utf-8")
    return sources


def read_pinned_sources(repo_root: Path) -> dict[str, str]:
    pathspecs = [f":(glob){root}/**/*.py" for root in MATERIAL_ROOTS]
    result = subprocess.run(
        ["git", "archive", "--format=tar", PINNED_BASE, "--", *pathspecs],
        cwd=repo_root,
        capture_output=True,
        check=False,
    )
    if result.returncode:
        error = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"cannot read pinned SOLID base {PINNED_BASE}: {error}")
    sources: dict[str, str] = {}
    with tarfile.open(fileobj=io.BytesIO(result.stdout), mode="r:") as archive:
        for member in archive.getmembers():
            if not member.isfile() or not member.name.endswith(".py"):
                continue
            stream = archive.extractfile(member)
            if stream is not None:
                sources[member.name] = stream.read().decode("utf-8")
    return sources


def apply_exact_exclusions(
    sources: Mapping[str, str],
    exclusions: Mapping[str, str] = VENDORED_SHA256,
) -> tuple[dict[str, str], list[Violation]]:
    kept: dict[str, str] = {}
    violations: list[Violation] = []
    for relative, source in sources.items():
        expected = exclusions.get(relative)
        if expected is None:
            kept[relative] = source
            continue
        actual = hashlib.sha256(source.encode("utf-8")).hexdigest().upper()
        if actual == expected.upper():
            continue
        kept[relative] = source
        violations.append(
            Violation(
                "excluded-source-sha256",
                relative,
                0,
                "",
                f"expected exact excluded-source SHA256 {expected}, got {actual}",
            )
        )
    return kept, violations


def material_sources(
    current: Mapping[str, str],
    pinned: Mapping[str, str],
) -> dict[str, str]:
    return {
        relative: source
        for relative, source in current.items()
        if relative not in pinned
        or _ast_shape(source, relative) != _ast_shape(pinned[relative], relative)
    }


def scan_global_class_limits(
    sources: Mapping[str, str],
    exclusions: Mapping[str, str] = VENDORED_SHA256,
) -> list[Violation]:
    kept, violations = apply_exact_exclusions(sources, exclusions)
    navpy = {
        relative: source
        for relative, source in kept.items()
        if relative.startswith("src/navpy/")
    }
    definitions = {
        relative: _definitions(_parse(source, relative))
        for relative, source in navpy.items()
    }
    occurrences = Counter(
        (relative, definition.qualname)
        for relative, items in definitions.items()
        for definition in items
        if isinstance(definition.node, ast.ClassDef)
    )
    effective_members = _effective_facet_members(navpy)
    for relative, items in definitions.items():
        for definition in items:
            node = definition.node
            if not isinstance(node, ast.ClassDef):
                continue
            methods = _direct_methods(node)
            direct_fields, dynamic_writes = owned_fields(node)
            dependencies = _constructor_dependencies(methods)
            line_count = _physical_extent(node)
            key = (relative, definition.qualname)
            effective_count, fields = effective_members.get(
                key,
                (len(methods), frozenset(direct_fields)),
            )
            line_limit = (
                FOCUSED_CLASS_LINE_LIMITS.get(key, MAX_CLASS_LINES)
                if occurrences[key] == 1
                else MAX_CLASS_LINES
            )
            field_limit = _field_limit(
                key, node, methods, occurrences[key], len(fields)
            )
            facade = _facade_method_allowance(
                key,
                occurrences[key],
                fields,
            )
            direct_limit = (
                MAX_DIRECT_METHODS
                if facade is None
                else facade.maximum_direct
            )
            effective_limit = (
                MAX_DIRECT_METHODS
                if facade is None
                else facade.maximum_effective
            )
            metrics = (
                ("class-lines", line_count, line_limit, "physical lines"),
                (
                    "direct-methods",
                    len(methods),
                    direct_limit,
                    "direct methods",
                ),
                (
                    "effective-methods",
                    effective_count,
                    effective_limit,
                    "direct and facet methods",
                ),
                ("owned-fields", len(fields), field_limit, "owned fields"),
                (
                    "constructor-dependencies",
                    dependencies,
                    MAX_CONSTRUCTOR_DEPENDENCIES,
                    "constructor dependencies",
                ),
            )
            for code, actual, limit, label in metrics:
                if actual > limit:
                    violations.append(
                        Violation(
                            code,
                            relative,
                            node.lineno,
                            definition.qualname,
                            f"{actual} {label}; limit is {limit}",
                        )
                    )
            for line, description in dynamic_writes:
                violations.append(
                    Violation(
                        "dynamic-field-write",
                        relative,
                        line,
                        definition.qualname,
                        f"cannot statically identify owned field from {description}",
                    )
                )
    return _sorted(violations)


def scan_material_limits(
    current: Mapping[str, str],
    pinned: Mapping[str, str],
    exclusions: Mapping[str, str] = VENDORED_SHA256,
) -> list[Violation]:
    kept, violations = apply_exact_exclusions(current, exclusions)
    for relative, source in material_sources(kept, pinned).items():
        module_lines = len(source.splitlines())
        module_limit = EXPERIMENT_MODULE_LINE_DEBT.get(
            relative, MAX_MATERIAL_MODULE_LINES
        )
        if module_lines > module_limit:
            violations.append(
                Violation(
                    "material-module-lines",
                    relative,
                    1,
                    "",
                    f"{module_lines} physical lines; limit is "
                    f"{module_limit}",
                )
            )
        tree = _parse(source, relative)
        for node in tree.body:
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            function_lines = _physical_extent(node)
            function_limit = EXPERIMENT_FUNCTION_LINE_DEBT.get(
                (relative, node.name), MAX_TOP_LEVEL_FUNCTION_LINES
            )
            if function_lines > function_limit:
                violations.append(
                    Violation(
                        "top-level-function-lines",
                        relative,
                        node.lineno,
                        node.name,
                        f"{function_lines} physical lines; limit is "
                        f"{function_limit}",
                    )
                )
    return _sorted(violations)


def scan_new_definition_annotations(
    current: Mapping[str, str],
    pinned: Mapping[str, str],
    exclusions: Mapping[str, str] = VENDORED_SHA256,
) -> list[Violation]:
    kept, violations = apply_exact_exclusions(current, exclusions)
    for relative, source in kept.items():
        current_functions = [
            definition
            for definition in _definitions(_parse(source, relative))
            if isinstance(
                definition.node, (ast.FunctionDef, ast.AsyncFunctionDef)
            )
        ]
        old_source = pinned.get(relative)
        old_names = (
            {
                definition.qualname
                for definition in _definitions(_parse(old_source, relative))
                if isinstance(
                    definition.node, (ast.FunctionDef, ast.AsyncFunctionDef)
                )
            }
            if old_source is not None
            else set()
        )
        for definition in current_functions:
            if definition.qualname in old_names:
                continue
            node = definition.node
            missing = [
                argument.arg
                for argument in _arguments(node)
                if argument.arg not in {"self", "cls"}
                and argument.annotation is None
            ]
            if missing:
                violations.append(
                    Violation(
                        "missing-parameter-annotations",
                        relative,
                        node.lineno,
                        definition.qualname,
                        f"new definition has unannotated parameters {missing}",
                    )
                )
            if node.returns is None:
                violations.append(
                    Violation(
                        "missing-return-annotation",
                        relative,
                        node.lineno,
                        definition.qualname,
                        "new definition has no return annotation",
                    )
                )
    return _sorted(violations)


def owned_fields(node: ast.ClassDef) -> tuple[set[str], list[tuple[int, str]]]:
    scanner = _OwnedFieldScanner()
    for method in _direct_methods(node):
        for statement in method.body:
            scanner.visit(statement)
    if _is_dataclass(node):
        for statement in node.body:
            if (
                isinstance(statement, ast.AnnAssign)
                and isinstance(statement.target, ast.Name)
                and _final_approach_name(statement.annotation) != "ClassVar"
            ):
                scanner.fields.add(statement.target.id)
    return scanner.fields, scanner.dynamic_writes


def format_violations(violations: Iterable[Violation]) -> str:
    items = list(violations)
    counts = Counter(item.code for item in items)
    summary = ", ".join(f"{code}={count}" for code, count in sorted(counts.items()))
    details = "\n  ".join(item.render() for item in items)
    return f"{len(items)} violation(s) ({summary}):\n  {details}"


def _parse(source: str, relative: str) -> ast.Module:
    if source.startswith("\ufeff"):
        source = source[1:]
    return ast.parse(source, filename=relative)


def _ast_shape(source: str, relative: str) -> str:
    tree = _parse(source, relative)
    tree.body = [
        statement
        for statement in tree.body
        if not (
            isinstance(statement, ast.ImportFrom)
            and statement.module == "__future__"
            and {alias.name for alias in statement.names} == {"annotations"}
        )
    ]
    return ast.dump(tree, include_attributes=False)


def _definitions(tree: ast.AST) -> list[Definition]:
    visitor = _DefinitionVisitor()
    visitor.visit(tree)
    return visitor.definitions


class _DefinitionVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.stack: list[str] = []
        self.definitions: list[Definition] = []

    def _visit_definition(
        self, node: ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef
    ) -> None:
        self.stack.append(node.name)
        self.definitions.append(Definition(".".join(self.stack), node))
        for statement in node.body:
            self.visit(statement)
        self.stack.pop()

    visit_ClassDef = _visit_definition
    visit_FunctionDef = _visit_definition
    visit_AsyncFunctionDef = _visit_definition


class _OwnedFieldScanner(ast.NodeVisitor):
    def __init__(self) -> None:
        self.fields: set[str] = set()
        self.dynamic_writes: list[tuple[int, str]] = []

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        return

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        return

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        return

    def visit_Lambda(self, node: ast.Lambda) -> None:
        return

    def visit_Attribute(self, node: ast.Attribute) -> None:
        if (
            isinstance(node.ctx, ast.Store)
            and isinstance(node.value, ast.Name)
            and node.value.id in {"self", "cls"}
        ):
            if node.attr == "__dict__":
                self.dynamic_writes.append((node.lineno, "whole __dict__ assignment"))
            else:
                self.fields.add(node.attr)
        self.generic_visit(node)

    def visit_Subscript(self, node: ast.Subscript) -> None:
        if isinstance(node.ctx, ast.Store) and _is_instance_dict(node.value):
            name = _literal_string(node.slice)
            if name is None:
                self.dynamic_writes.append((node.lineno, "dynamic __dict__ key"))
            else:
                self.fields.add(name)
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call) -> None:
        call_name = _dotted_name(node.func)
        if call_name in {"setattr", "object.__setattr__"}:
            if len(node.args) > 1 and _is_self(node.args[0]):
                self._record_named_write(node.args[1], node.lineno, call_name)
        elif call_name == "self.__setattr__" and node.args:
            self._record_named_write(node.args[0], node.lineno, call_name)
        if (
            isinstance(node.func, ast.Attribute)
            and node.func.attr == "update"
            and _is_instance_dict(node.func.value)
        ):
            self._visit_dict_update(node)
        self.generic_visit(node)

    def _record_named_write(
        self, key: ast.AST, line: int, description: str
    ) -> None:
        name = _literal_string(key)
        if name is None:
            self.dynamic_writes.append((line, f"dynamic {description}"))
        else:
            self.fields.add(name)

    def _visit_dict_update(self, node: ast.Call) -> None:
        for keyword in node.keywords:
            if keyword.arg is None:
                self.dynamic_writes.append((node.lineno, "dynamic __dict__.update"))
            else:
                self.fields.add(keyword.arg)
        for argument in node.args:
            if not isinstance(argument, ast.Dict):
                self.dynamic_writes.append((node.lineno, "dynamic __dict__.update"))
                continue
            for key in argument.keys:
                name = _literal_string(key) if key is not None else None
                if name is None:
                    self.dynamic_writes.append(
                        (node.lineno, "dynamic __dict__.update key")
                    )
                else:
                    self.fields.add(name)


def _direct_methods(
    node: ast.ClassDef,
) -> list[ast.FunctionDef | ast.AsyncFunctionDef]:
    return [
        statement
        for statement in node.body
        if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef))
    ]


def _is_passive_record_method(
    method: ast.FunctionDef | ast.AsyncFunctionDef,
) -> bool:
    """Validation and read-only derived views keep a frozen record passive.

    The decorator alone is not the contract: a `@property` body can still
    mutate a frozen dataclass through `object.__setattr__`. Any reachable
    mutation escape in the body disqualifies the method, `__post_init__`
    included -- a passive record validates there, it does not assign.
    """
    if _reaches_attribute_mutation(method):
        return False
    if method.name == "__post_init__":
        return True
    decorators = {_dotted_name(decorator) for decorator in method.decorator_list}
    return bool(decorators & {"property", "staticmethod"})


def _reaches_attribute_mutation(
    method: ast.FunctionDef | ast.AsyncFunctionDef,
) -> bool:
    for node in ast.walk(method):
        if isinstance(node, ast.Attribute) and node.attr in {
            "__setattr__",
            "__delattr__",
            "__dict__",
        }:
            return True
        if isinstance(node, ast.Name) and node.id in {
            "setattr",
            "delattr",
            "vars",
        }:
            return True
    return False


def _constructor_dependencies(
    methods: Iterable[ast.FunctionDef | ast.AsyncFunctionDef],
) -> int:
    constructor = next((method for method in methods if method.name == "__init__"), None)
    if constructor is None:
        return 0
    return sum(
        argument.arg not in {"self", "cls"} for argument in _arguments(constructor)
    )


def _arguments(
    node: ast.FunctionDef | ast.AsyncFunctionDef,
) -> list[ast.arg]:
    arguments = [
        *node.args.posonlyargs,
        *node.args.args,
        *node.args.kwonlyargs,
    ]
    if node.args.vararg is not None:
        arguments.append(node.args.vararg)
    if node.args.kwarg is not None:
        arguments.append(node.args.kwarg)
    return arguments


def _field_limit(
    key: tuple[str, str],
    node: ast.ClassDef,
    methods: list[ast.FunctionDef | ast.AsyncFunctionDef],
    occurrences: int,
    field_count: int,
) -> int:
    passive_limit = PASSIVE_RECORD_FIELD_LIMITS.get(key)
    if (
        passive_limit is not None
        and occurrences == 1
        and all(_is_passive_record_method(method) for method in methods)
        and _is_frozen_dataclass(node)
        and field_count <= passive_limit
    ):
        return passive_limit
    allowance = FIELD_ALLOWANCES.get(key)
    if (
        allowance is not None
        and occurrences == 1
        and allowance.required_base in {_dotted_name(base) for base in node.bases}
    ):
        return allowance.maximum
    return MAX_OWNED_FIELDS


def _facade_method_allowance(
    key: tuple[str, str],
    occurrences: int,
    fields: set[str],
) -> FacadeMethodAllowance | None:
    allowance = FACADE_METHOD_ALLOWANCES.get(key)
    if (
        allowance is None
        or occurrences != 1
        or fields != allowance.required_fields
    ):
        return None
    return allowance


def _effective_facet_members(
    sources: Mapping[str, str],
) -> dict[tuple[str, str], tuple[int, frozenset[str]]]:
    """Aggregate methods and owned fields from local Facet/Mixin bases."""
    modules: dict[str, tuple[str, ast.Module]] = {}
    for relative, source in sources.items():
        module = _module_name(relative)
        if module is not None:
            modules[module] = (relative, _parse(source, relative))

    classes: dict[tuple[str, str], ast.ClassDef] = {}
    for module, (_, tree) in modules.items():
        for node in tree.body:
            if isinstance(node, ast.ClassDef):
                classes[(module, node.name)] = node

    aliases: dict[str, dict[str, tuple[str, str]]] = {}
    for module, (_, tree) in modules.items():
        resolved = {
            name: (owner, name)
            for owner, name in classes
            if owner == module
        }
        for statement in tree.body:
            if not isinstance(statement, ast.ImportFrom):
                continue
            imported_module = _absolute_import_module(module, statement)
            if imported_module is None:
                continue
            for item in statement.names:
                resolved[item.asname or item.name] = (
                    imported_module,
                    item.name,
                )
        aliases[module] = resolved

    memo: dict[
        tuple[str, str],
        tuple[frozenset[str], frozenset[str]],
    ] = {}

    def collect(
        reference: tuple[str, str],
        active: frozenset[tuple[str, str]] = frozenset(),
    ) -> tuple[frozenset[str], frozenset[str]]:
        cached = memo.get(reference)
        if cached is not None:
            return cached
        if reference in active:
            return frozenset(), frozenset()
        node = classes[reference]
        methods = {
            item.name
            for item in node.body
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        fields, _dynamic_writes = owned_fields(node)
        next_active = active | {reference}
        for base in node.bases:
            name = _final_approach_name(base)
            if name is None:
                continue
            target = aliases[reference[0]].get(name)
            if (
                target not in classes
                or not target[1].endswith(("Facet", "Mixin"))
            ):
                continue
            inherited_methods, inherited_fields = collect(target, next_active)
            methods.update(inherited_methods)
            fields.update(inherited_fields)
        result = frozenset(methods), frozenset(fields)
        memo[reference] = result
        return result

    metrics: dict[tuple[str, str], tuple[int, frozenset[str]]] = {}
    for reference in classes:
        relative = modules[reference[0]][0]
        methods, fields = collect(reference)
        metrics[(relative, reference[1])] = len(methods), fields
    return metrics


def _module_name(relative: str) -> str | None:
    if not relative.startswith("src/") or not relative.endswith(".py"):
        return None
    module = relative[4:-3].replace("/", ".")
    return module.removesuffix(".__init__")


def _absolute_import_module(
    current_module: str,
    statement: ast.ImportFrom,
) -> str | None:
    if statement.level == 0:
        return statement.module
    package = current_module.rsplit(".", 1)[0]
    parts = package.split(".")
    keep = len(parts) - statement.level + 1
    if keep < 0:
        return None
    prefix = ".".join(parts[:keep])
    if statement.module:
        return f"{prefix}.{statement.module}" if prefix else statement.module
    return prefix


def _is_dataclass(node: ast.ClassDef) -> bool:
    return any(
        _final_approach_name(
            decorator.func if isinstance(decorator, ast.Call) else decorator
        )
        == "dataclass"
        for decorator in node.decorator_list
    )


def _is_frozen_dataclass(node: ast.ClassDef) -> bool:
    for decorator in node.decorator_list:
        if not isinstance(decorator, ast.Call):
            continue
        if _final_approach_name(decorator.func) != "dataclass":
            continue
        return any(
            keyword.arg == "frozen"
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value is True
            for keyword in decorator.keywords
        )
    return False


def _physical_extent(
    node: ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef,
) -> int:
    start = min([node.lineno, *(item.lineno for item in node.decorator_list)])
    return (node.end_lineno or node.lineno) - start + 1


def _final_approach_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Subscript):
        return _final_approach_name(node.value)
    dotted = _dotted_name(node)
    return dotted.rsplit(".", 1)[-1] if dotted else None


def _dotted_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        prefix = _dotted_name(node.value)
        return f"{prefix}.{node.attr}" if prefix else node.attr
    if isinstance(node, ast.Subscript):
        return _dotted_name(node.value)
    return None


def _literal_string(node: ast.AST) -> str | None:
    return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else None


def _is_self(node: ast.AST) -> bool:
    return isinstance(node, ast.Name) and node.id in {"self", "cls"}


def _is_instance_dict(node: ast.AST) -> bool:
    if (
        isinstance(node, ast.Attribute)
        and node.attr == "__dict__"
        and _is_self(node.value)
    ):
        return True
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "vars"
        and len(node.args) == 1
        and _is_self(node.args[0])
    )


def _sorted(violations: Iterable[Violation]) -> list[Violation]:
    return sorted(
        violations,
        key=lambda item: (
            item.relative,
            item.line,
            item.owner,
            item.code,
            item.detail,
        ),
    )
