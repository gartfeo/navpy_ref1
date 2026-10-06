"""Regression selector for vehicle, snap, and NavPy runtime ownership."""

from __future__ import annotations

import importlib
import inspect
import math
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import ANY, MagicMock, patch

import pytest

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup
from navpy.logger.navigation_snap import calc_distance, calc_h_v_dist
from navpy.logger.navigation_log_streams import NavigationLogStreams
from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.mav_bus import MavBus
from navpy.modules.vehicle.vehicle_composition import build_vehicle_parts
from navpy.modules.vehicle.vehicle_lifecycle import VehicleLifecycle


REPO_ROOT = Path(__file__).resolve().parents[1]


def _fake_bus(*, falsey: bool = False, release_error: BaseException | None = None):
    bus = MagicMock()
    bus.__bool__.return_value = not falsey
    bus.conn = SimpleNamespace(mav=MagicMock())
    bus.send_lock = threading.RLock()
    bus.heartbeats = set()
    lease = MagicMock()
    if release_error is not None:
        lease.release.side_effect = release_error
    bus.reserve.return_value = lease
    return bus, lease


def _build(bus):
    return build_vehicle_parts(
        "unused",
        7,
        115200,
        MagicMock(),
        True,
        False,
        False,
        1.0,
        3.0,
        18,
        191,
        bus,
    )


def _flatten(group: BaseException) -> list[BaseException]:
    if isinstance(group, BaseExceptionGroup):
        return [leaf for child in group.exceptions for leaf in _flatten(child)]
    return [group]


def test_falsey_injected_bus_is_preserved_by_identity():
    bus, _lease = _fake_bus(falsey=True)
    with patch.object(
        MavBus,
        "get_or_create",
        side_effect=AssertionError("injected bus was discarded"),
    ):
        parts = _build(bus)

    try:
        assert parts.runtime.lifetime is not None
        bus.reserve.assert_called_once_with(7)
    finally:
        parts.runtime.lifetime.close()


def test_vehicle_build_rollback_aggregates_primary_and_cleanup_failures():
    transaction = importlib.import_module(
        "navpy.modules.vehicle.vehicle_build_transaction"
    )
    bus, lease = _fake_bus(release_error=OSError("lease release failed"))
    with patch.object(
        transaction,
        "build_protocol_parts",
        side_effect=RuntimeError("protocol build failed"),
    ):
        with pytest.raises(ExceptionGroup) as caught:
            _build(bus)

    errors = _flatten(caught.value)
    assert [(type(error), str(error)) for error in errors] == [
        (RuntimeError, "protocol build failed"),
        (OSError, "lease release failed"),
    ]
    lease.release.assert_called_once_with()


@pytest.mark.parametrize(
    "failure_stage",
    ("foundation", "protocol", "capability", "activate"),
)
def test_vehicle_transaction_releases_lease_after_every_reserved_failure(
    failure_stage,
):
    transaction_module = importlib.import_module(
        "navpy.modules.vehicle.vehicle_build_transaction"
    )
    types = importlib.import_module("navpy.modules.vehicle.vehicle_build_types")
    bus, lease = _fake_bus()
    request = types.VehicleBuildRequest(
        link=types.VehicleLinkRequest("unused", 7, 115200, 18, 191, bus),
        startup=types.VehicleStartupPolicy(True, False, False, 1.0, 3.0),
        logger=MagicMock(),
    )
    lifecycle = MagicMock()
    lifecycle.close.side_effect = lease.release
    foundation = object()
    protocols = SimpleNamespace(lifecycle=lifecycle)
    parts = object()
    builders = {
        "foundation": MagicMock(return_value=foundation),
        "protocol": MagicMock(return_value=protocols),
        "capability": MagicMock(return_value=object()),
    }
    builders[failure_stage if failure_stage != "activate" else "capability"].side_effect = (
        None if failure_stage == "activate" else RuntimeError(failure_stage)
    )
    if failure_stage == "activate":
        builders["capability"].return_value = object()
    transaction = transaction_module.VehicleBuildTransaction(request)

    with (
        patch.object(
            transaction_module,
            "build_vehicle_foundation",
            builders["foundation"],
        ),
        patch.object(
            transaction_module,
            "build_protocol_parts",
            builders["protocol"],
        ),
        patch.object(
            transaction_module,
            "build_capability_parts",
            builders["capability"],
        ),
        patch.object(
            transaction_module,
            "assemble_vehicle_parts",
            return_value=parts,
        ),
        patch.object(
            transaction,
            "_activate",
            side_effect=(
                RuntimeError("activate")
                if failure_stage == "activate"
                else None
            ),
        ),
    ):
        with pytest.raises(RuntimeError, match=failure_stage):
            transaction.build()

    lease.release.assert_called_once_with()
    assert transaction.phase == "rolled_back"
    assert not transaction.has_pending_ownership
    expected_lifecycle_closes = int(
        failure_stage in {"capability", "activate"}
    )
    assert lifecycle.close.call_count == expected_lifecycle_closes


def test_vehicle_transaction_retains_failed_rollback_for_explicit_retry():
    transaction_module = importlib.import_module(
        "navpy.modules.vehicle.vehicle_build_transaction"
    )
    types = importlib.import_module("navpy.modules.vehicle.vehicle_build_types")
    bus, lease = _fake_bus()
    lease.release.side_effect = [OSError("release failed"), None]
    request = types.VehicleBuildRequest(
        link=types.VehicleLinkRequest("unused", 7, 115200, 18, 191, bus),
        startup=types.VehicleStartupPolicy(True, False, False, 1.0, 3.0),
        logger=MagicMock(),
    )
    transaction = transaction_module.VehicleBuildTransaction(request)

    with patch.object(
        transaction_module,
        "build_vehicle_foundation",
        side_effect=RuntimeError("foundation failed"),
    ):
        with pytest.raises(ExceptionGroup):
            transaction.build()

    assert transaction.phase == "rollback_failed"
    assert transaction.has_pending_ownership
    transaction.retry_rollback()
    assert transaction.phase == "rolled_back"
    assert not transaction.has_pending_ownership
    assert lease.release.call_count == 2


def test_vehicle_lifecycle_defers_dependencies_until_heartbeat_quiesces():
    heartbeat = MagicMock()
    heartbeat.close.side_effect = [RuntimeError("heartbeat failed"), None]
    closer = MagicMock()
    closer.close.side_effect = OSError("snapshot failed")
    lease = MagicMock()
    lease.release.side_effect = ValueError("lease failed")
    lifecycle = VehicleLifecycle(
        MagicMock(heartbeats={}),
        lease,
        7,
        heartbeat,
        MagicMock(),
        SimpleNamespace(value=MagicMock()),
        closers=(closer,),
    )

    with pytest.raises(RuntimeError, match="heartbeat failed"):
        lifecycle.close()

    closer.close.assert_not_called()
    lease.release.assert_not_called()

    with pytest.raises(ExceptionGroup) as caught:
        lifecycle.close()

    assert [(type(error), str(error)) for error in caught.value.exceptions] == [
        (OSError, "snapshot failed"),
        (ValueError, "lease failed"),
    ]
    assert heartbeat.close.call_count == 2
    closer.close.assert_called_once_with()
    lease.release.assert_called_once_with()


def test_vehicle_interface_facade_preserves_exact_class_objects():
    facade = importlib.import_module("navpy.modules.vehicle.vehicle_interface")
    capabilities = importlib.import_module(
        "navpy.modules.vehicle.vehicle_capability_interfaces"
    )
    contract = importlib.import_module("navpy.modules.vehicle.vehicle_contract")

    for name in (
        "VehicleIdentityAccess",
        "VehicleMotionTelemetry",
        "VehiclePowerTelemetry",
        "VehicleLimits",
        "VehicleFlightControl",
        "VehicleMissionAccess",
        "VehicleParameters",
        "VehicleSimulationAccess",
        "VehicleMessaging",
        "VehicleLifetime",
        "VehicleLogging",
    ):
        assert getattr(facade, name) is getattr(capabilities, name)
    assert facade.IVehicle is contract.IVehicle


def test_composite_vehicle_preserves_legacy_constructor_and_abstract_surface():
    facade = importlib.import_module("navpy.modules.vehicle.vehicle_interface")
    legacy_abstracts = {
        "air_speed",
        "attitude",
        "battery_level",
        "clear_mission",
        "close",
        "disarm",
        "download_mission",
        "get_mission_item",
        "get_mission_item_location",
        "get_mode",
        "get_parameter",
        "goto",
        "ground_speed",
        "ground_speed_ned",
        "heading",
        "home_location",
        "is_armed",
        "lim_roll",
        "location",
        "max_pitch",
        "min_pitch",
        "mission_items_count",
        "mission_items_next",
        "on_message",
        "register_rc_channel",
        "restart_mission",
        "send_mavlink_message",
        "send_status_text",
        "set_attitude",
        "set_current",
        "set_logger",
        "set_mode",
        "set_parameter",
        "update_mission_item",
        "update_mission_item_location",
        "upload_mission",
        "velocity",
        "wind",
    }
    assert set(facade.IVehicle.__abstractmethods__) == legacy_abstracts
    implementations = {}
    for name in legacy_abstracts:
        descriptor = inspect.getattr_static(facade.IVehicle, name)
        implementations[name] = (
            property(lambda self: None)
            if isinstance(descriptor, property)
            else lambda self, *args, **kwargs: None
        )
    legacy_vehicle = type("LegacyVehicle", (facade.IVehicle,), implementations)

    vehicle = legacy_vehicle(7)

    assert tuple(inspect.signature(facade.IVehicle).parameters) == (
        "target_system",
    )
    assert vehicle.target_system == 7
    assert vehicle.source_system == 7


def test_navigation_snap_facade_preserves_exact_class_and_function_objects():
    facade = importlib.import_module("navpy.logger.navigation_snap")
    geometry = importlib.import_module("navpy.logger.navigation_snap_geometry")
    tracker = importlib.import_module("navpy.logger.navigation_snap_tracker")
    types = importlib.import_module("navpy.logger.navigation_snap_types")

    assert facade.ClosestSnap is types.ClosestSnap
    assert facade.ClosestPointComponents is types.ClosestPointComponents
    assert facade.ClosestApproachTracker is tracker.ClosestApproachTracker
    for name in (
        "calc_h_v_dist",
        "calc_distance",
        "closest_point_components_on_segment",
        "closest_on_segment",
        "closest_horizontal_on_segment",
    ):
        assert getattr(facade, name) is getattr(geometry, name)


@pytest.mark.parametrize(
    ("first", "second"),
    [
        (None, Location(40.0, 44.0, 100.0)),
        (Location(40.0, 44.0, 100.0), None),
        (Location(math.nan, 44.0, 100.0), Location(40.0, 44.0, 100.0)),
    ],
)
def test_missing_or_invalid_snap_geo_is_not_scored_as_zero(first, second):
    assert math.isinf(calc_distance(first, second))
    horizontal, vertical = calc_h_v_dist(first, second)
    assert math.isinf(horizontal)
    assert math.isinf(vertical)


def test_navigation_stream_close_surfaces_all_failures_after_attempting_both():
    compact = MagicMock()
    compact.flush.side_effect = OSError("compact flush failed")
    debug = MagicMock()
    debug.close.side_effect = OSError("debug close failed")
    streams = NavigationLogStreams(compact, debug)

    with pytest.raises(ExceptionGroup) as caught:
        streams.close()

    assert [str(error) for error in caught.value.exceptions] == [
        "compact flush failed",
        "debug close failed",
    ]
    compact.close.assert_called_once_with()
    debug.flush.assert_called_once_with()
    debug.close.assert_called_once_with()


def test_navigation_stream_retains_only_close_failure_for_retry():
    compact = MagicMock()
    compact.close.side_effect = [OSError("compact close failed"), None]
    debug = MagicMock()
    streams = NavigationLogStreams(compact, debug)

    with pytest.raises(OSError, match="compact close failed"):
        streams.close()
    assert not streams.has_compact()
    streams.close()

    assert compact.flush.call_count == 2
    assert compact.close.call_count == 2
    debug.flush.assert_called_once_with()
    debug.close.assert_called_once_with()


def test_runtime_resource_stack_attempts_all_cleanup_and_retries_only_failures():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    attempts = 0

    def flaky() -> None:
        nonlocal attempts
        attempts += 1
        events.append("network")
        if attempts == 1:
            raise OSError("network close failed")

    stack = resources.RuntimeResourceStack()
    stack.own("logger", lambda: events.append("logger"))
    stack.own("network", flaky)
    stack.own("controller", lambda: events.append("controller"))

    with pytest.raises(ExceptionGroup, match="runtime cleanup failed"):
        stack.close()
    assert events == ["controller", "network", "logger"]

    stack.close()
    assert events == ["controller", "network", "logger", "network"]


def test_failed_quiescence_defers_dependents_but_closes_independent_resources():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    controller_attempts = iter((False, True))
    stack = resources.RuntimeResourceStack()
    stack.own("logger", lambda: events.append("logger"))
    stack.own_independent("windows", lambda: events.append("windows"))
    for label in ("vehicle", "network", "vision", "navigation"):
        stack.own(label, lambda label=label: events.append(label))
    stack.own_quiescence(
        "controller",
        lambda: events.append("controller") or next(controller_attempts),
    )

    with pytest.raises(ExceptionGroup, match="runtime cleanup failed") as caught:
        stack.close()

    assert events == ["controller", "windows"]
    assert [str(error) for error in _flatten(caught.value)] == [
        "controller reported incomplete cleanup",
    ]

    stack.close()

    assert events == [
        "controller",
        "windows",
        "controller",
        "navigation",
        "vision",
        "network",
        "vehicle",
        "logger",
    ]


def test_quiescence_and_independent_failures_are_aggregated_and_retryable():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    controller_attempts = iter((OSError("controller stuck"), None))
    window_attempts = iter((OSError("window close failed"), None))
    stack = resources.RuntimeResourceStack()

    def controller_stop() -> None:
        events.append("controller")
        failure = next(controller_attempts)
        if failure is not None:
            raise failure

    def close_windows() -> None:
        events.append("windows")
        failure = next(window_attempts)
        if failure is not None:
            raise failure

    stack.own_independent("windows", close_windows)
    stack.own("navigation", lambda: events.append("navigation"))
    stack.own_quiescence("controller", controller_stop)

    with pytest.raises(ExceptionGroup) as caught:
        stack.close()

    assert events == ["controller", "windows"]
    assert [str(error) for error in _flatten(caught.value)] == [
        "controller stuck",
        "window close failed",
    ]

    stack.close()

    assert events == [
        "controller",
        "windows",
        "controller",
        "navigation",
        "windows",
    ]


def test_runtime_resource_stack_reentrant_close_does_not_double_run_action():
    resources = importlib.import_module("navpy.runtime_resources")
    stack = resources.RuntimeResourceStack()
    events: list[str] = []
    entered = False

    def reentrant_cleanup() -> None:
        nonlocal entered
        events.append("cleanup")
        if not entered:
            entered = True
            stack.close()

    stack.own("reentrant", reentrant_cleanup)
    stack.close()

    assert events == ["cleanup"]


def test_runtime_resource_stack_serializes_concurrent_close():
    resources = importlib.import_module("navpy.runtime_resources")
    stack = resources.RuntimeResourceStack()
    entered = threading.Event()
    release = threading.Event()
    events: list[str] = []
    errors: list[BaseException] = []

    def blocking_cleanup() -> None:
        events.append("cleanup")
        entered.set()
        release.wait(timeout=1.0)

    def close() -> None:
        try:
            stack.close()
        except BaseException as error:
            errors.append(error)

    stack.own("blocking", blocking_cleanup)
    first = threading.Thread(target=close, daemon=True)
    second = threading.Thread(target=close, daemon=True)
    first.start()
    assert entered.wait(timeout=0.5)
    second.start()
    second.join(timeout=0.05)
    assert second.is_alive()

    release.set()
    first.join(timeout=0.5)
    second.join(timeout=0.5)

    assert not errors
    assert not first.is_alive()
    assert not second.is_alive()
    assert events == ["cleanup"]


def test_runtime_cleanup_stops_controller_before_navigation_and_later_resources():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    stack = resources.RuntimeResourceStack()
    for label in ("logger", "vehicle", "network", "vision", "navigation"):
        stack.own(label, lambda label=label: events.append(label))
    stack.own_quiescence("controller", lambda: events.append("controller"))

    stack.close()

    assert events == [
        "controller",
        "navigation",
        "vision",
        "network",
        "vehicle",
        "logger",
    ]


def test_runtime_composition_registers_live_workers_as_quiescence(monkeypatch):
    composition = importlib.import_module("navpy.runtime_composition")
    registrations: list[tuple[str, str, object]] = []

    class ResourceSpy:
        @staticmethod
        def own(label, action):
            registrations.append(("dependent", label, action))

        @staticmethod
        def own_quiescence(label, action):
            registrations.append(("quiescence", label, action))

    vision = SimpleNamespace(
        start=MagicMock(),
        stop=MagicMock(),
        ui_step=MagicMock(),
        geo_ref=object(),
        coordination=object(),
        approach_kind=object(),
        profile=object(),
        is_quiescent=True,
        raise_if_failed=MagicMock(),
    )
    def start_navigation() -> None:
        assert registrations[-1][:2] == ("quiescence", "navigation")

    navigation = SimpleNamespace(
        start=MagicMock(side_effect=start_navigation),
        stop=MagicMock(),
        raise_if_failed=MagicMock(),
    )
    def set_network(_network) -> None:
        assert registrations[-1][:2] == ("quiescence", "controller")

    controller = SimpleNamespace(
        start=MagicMock(),
        stop=MagicMock(),
        raise_if_failed=MagicMock(),
        set_network=MagicMock(side_effect=set_network),
    )
    vehicle = MagicMock()
    vehicle.get_param_or_default.return_value = 50
    foundation = SimpleNamespace(
        vehicle=vehicle,
        vision_args=object(),
        logger=MagicMock(),
        zc_util=None,
        navigation_args=object(),
        scheduler_cadence=None,
        network=MagicMock(),
        vehicle_close_gate=SimpleNamespace(stop_vision=MagicMock()),
    )
    monkeypatch.setattr(composition, "VisionController", lambda *_args, **_kw: vision)
    monkeypatch.setattr(composition, "MissionPlanner", lambda *_args: object())
    monkeypatch.setattr(composition, "MissionPlannerArgs", lambda _args: object())
    monkeypatch.setattr(composition, "Navigation", lambda *_args, **_kw: navigation)
    monkeypatch.setattr(composition, "NavController", lambda *_args, **_kw: controller)
    monkeypatch.setattr(composition, "NavArgs", lambda *_args: object())

    session = composition._build_session(object(), foundation, ResourceSpy())

    assert [(role, label) for role, label, _action in registrations] == [
        ("quiescence", "vision"),
        ("quiescence", "navigation"),
        ("quiescence", "controller"),
    ]
    assert registrations[-1][2] is controller.stop
    navigation.start.assert_called_once_with()
    session._health_check()
    vision.raise_if_failed.assert_called_once_with()
    navigation.raise_if_failed.assert_called_once_with()
    controller.raise_if_failed.assert_called_once_with()
    registrations[0][2]()
    foundation.vehicle_close_gate.stop_vision.assert_called_once_with(
        vision.stop,
        ANY,
    )


def test_shared_cadence_is_lazy_and_independent_of_detector_kind(monkeypatch):
    composition = importlib.import_module("navpy.runtime_composition")
    registrations: list[tuple[str, str]] = []

    class ResourceSpy:
        @staticmethod
        def own(label, _action):
            registrations.append(("dependent", label))

        @staticmethod
        def own_independent(label, _action):
            registrations.append(("independent", label))

    status_logger = MagicMock()
    logger = MagicMock()
    vehicle = MagicMock(source_system=121)
    vehicle.is_simulated_autopilot.return_value = False
    network = MagicMock()
    cadence = MagicMock()
    navigation_args = SimpleNamespace(use_terrain=False)

    monkeypatch.setattr(
        composition.GroundStationLoggerNetwork,
        "create",
        MagicMock(return_value=status_logger),
    )
    monkeypatch.setattr(composition, "ConnArgs", lambda _args: SimpleNamespace(
        source_system=121,
    ))
    monkeypatch.setattr(composition, "LoggerArgs", lambda _args: object())
    monkeypatch.setattr(composition, "initialize_logger", lambda *_args: logger)
    monkeypatch.setattr(composition, "create_vehicle", lambda *_args: vehicle)
    monkeypatch.setattr(composition, "NetworkArgs", lambda _args: object())
    monkeypatch.setattr(composition, "create_network", lambda *_args: network)
    monkeypatch.setattr(composition, "NavigationArgs", lambda *_args: navigation_args)
    monkeypatch.setattr(
        composition.VisionArgs,
        "from_args",
        MagicMock(return_value=SimpleNamespace(detector_type="real")),
    )
    cadence_type = MagicMock(return_value=cadence)
    monkeypatch.setattr(composition, "SchedulerCadence", cadence_type)
    monkeypatch.setattr(composition, "_log_configuration", MagicMock())

    foundation = composition._build_foundation(object(), ResourceSpy())

    assert foundation.scheduler_cadence is cadence
    cadence_type.assert_called_once_with(vehicle.sim_speedup)
    vehicle.is_simulated_autopilot.assert_not_called()
    assert ("dependent", "simulation cadence") in registrations


def test_unsafe_vision_stop_retains_runtime_dependencies_until_retry():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    attempts = iter((False, True))
    gate = resources.VehicleCloseGate(lambda: events.append("vehicle"))
    stack = resources.RuntimeResourceStack()
    stack.own("logger", lambda: events.append("logger"))
    stack.own_independent("windows", lambda: events.append("windows"))
    stack.own("vehicle", gate.close_vehicle)
    stack.own("network", lambda: events.append("network"))
    stack.own("cadence", lambda: events.append("cadence"))
    stack.own_quiescence(
        "vision",
        lambda: events.append("vision")
        or gate.stop_vision(lambda: next(attempts)),
    )
    stack.own("navigation", lambda: events.append("navigation"))
    stack.own_quiescence("controller", lambda: events.append("controller"))

    with pytest.raises(ExceptionGroup) as caught:
        stack.close()

    assert events == ["controller", "navigation", "vision", "windows"]
    messages = [str(error) for error in _flatten(caught.value)]
    assert any("vision stop reported incomplete" in message for message in messages)

    stack.close()

    assert events == [
        "controller",
        "navigation",
        "vision",
        "windows",
        "vision",
        "cadence",
        "network",
        "vehicle",
        "logger",
    ]


def test_completed_vision_retry_releases_retained_vehicle_ownership():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    attempts = iter((False, True))
    gate = resources.VehicleCloseGate(lambda: events.append("vehicle"))
    stack = resources.RuntimeResourceStack()
    stack.own("vehicle", gate.close_vehicle)
    stack.own("vision", lambda: gate.stop_vision(lambda: next(attempts)))

    with pytest.raises(ExceptionGroup):
        stack.close()
    stack.close()

    assert events == ["vehicle"]


def test_runtime_ready_signal_follows_initial_health_and_precedes_ui():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    session = resources.RuntimeSession(
        resources=resources.RuntimeResourceStack(),
        start_vision=lambda: events.append("vision"),
        start_controller=lambda: events.append("controller"),
        health_check=lambda: events.append("health"),
        ready_signal=lambda: events.append("ready"),
        ui_step=lambda: events.append("ui") or False,
        sleep=lambda _seconds: None,
    )

    assert session.run() == 0
    assert events[:5] == ["vision", "controller", "health", "ready", "ui"]
    assert events.count("ready") == 1


def test_runtime_ready_signal_is_not_emitted_when_initial_health_fails():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    failure = RuntimeError("initial health failed")
    session = resources.RuntimeSession(
        resources=resources.RuntimeResourceStack(),
        start_vision=lambda: events.append("vision"),
        start_controller=lambda: events.append("controller"),
        health_check=lambda: (_ for _ in ()).throw(failure),
        ready_signal=lambda: events.append("ready"),
        ui_step=lambda: events.append("ui") or False,
        sleep=lambda _seconds: None,
    )

    with pytest.raises(RuntimeError, match="initial health failed"):
        session.run()
    assert "ready" not in events
    assert "ui" not in events


def test_persistent_vision_stop_error_survives_runtime_retry():
    resources = importlib.import_module("navpy.runtime_resources")
    sentinel = OSError("physical gimbal disconnect failed")
    gate = resources.VehicleCloseGate(lambda: None)
    stack = resources.RuntimeResourceStack()
    stack.own("vehicle", gate.close_vehicle)
    stack.own_quiescence(
        "vision",
        lambda: gate.stop_vision(
            lambda: (_ for _ in ()).throw(sentinel)
        ),
    )
    session = resources.RuntimeSession(
        resources=stack,
        start_vision=lambda: None,
        start_controller=lambda: None,
        health_check=lambda: None,
        ui_step=lambda: False,
        sleep=lambda _seconds: None,
    )

    with pytest.raises(ExceptionGroup) as caught:
        session.run()

    leaves = _flatten(caught.value)
    assert sentinel in leaves
    assert not any(
        "vision stop reported incomplete" in str(error) for error in leaves
    )


def test_runtime_session_aggregates_run_and_cleanup_failures():
    resources = importlib.import_module("navpy.runtime_resources")
    stack = resources.RuntimeResourceStack()
    stack.own("logger", lambda: (_ for _ in ()).throw(OSError("close failed")))
    session = resources.RuntimeSession(
        resources=stack,
        start_vision=lambda: None,
        start_controller=lambda: None,
        health_check=lambda: None,
        ui_step=lambda: (_ for _ in ()).throw(RuntimeError("pump failed")),
        sleep=lambda _seconds: None,
    )

    with pytest.raises(ExceptionGroup) as caught:
        session.run()

    assert [(type(error), str(error)) for error in _flatten(caught.value)] == [
        (RuntimeError, "pump failed"),
        (OSError, "close failed"),
    ]


def test_runtime_session_retries_retained_quiescence_before_failing():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    vision_attempts = iter((False, True))
    stack = resources.RuntimeResourceStack()
    stack.own("logger", lambda: events.append("logger"))
    stack.own("vehicle", lambda: events.append("vehicle"))
    stack.own("network", lambda: events.append("network"))
    stack.own("cadence", lambda: events.append("cadence"))
    stack.own_quiescence(
        "vision",
        lambda: events.append("vision") or next(vision_attempts),
    )
    stack.own("navigation", lambda: events.append("navigation"))
    stack.own_quiescence("controller", lambda: events.append("controller"))
    session = resources.RuntimeSession(
        resources=stack,
        start_vision=lambda: None,
        start_controller=lambda: None,
        health_check=lambda: None,
        ui_step=lambda: False,
        sleep=lambda _seconds: None,
    )

    assert session.run() == 0
    assert events == [
        "controller",
        "navigation",
        "vision",
        "vision",
        "cadence",
        "network",
        "vehicle",
        "logger",
    ]


def test_runtime_session_keyboard_interrupt_is_clean_exit_after_cleanup():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    stack = resources.RuntimeResourceStack()
    stack.own("controller", lambda: events.append("controller"))
    session = resources.RuntimeSession(
        resources=stack,
        start_vision=lambda: None,
        start_controller=lambda: None,
        health_check=lambda: None,
        ui_step=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
        sleep=lambda _seconds: None,
    )

    assert session.run() == 0
    assert events == ["controller"]


def test_runtime_session_stops_pumping_when_controller_health_fails():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    failure = TypeError("navigation worker failed")
    checks = iter((None, failure, failure))
    stack = resources.RuntimeResourceStack()
    stack.own("cleanup", lambda: events.append("cleanup"))

    def check_health() -> None:
        events.append("health")
        failure = next(checks)
        if failure is not None:
            raise failure

    session = resources.RuntimeSession(
        resources=stack,
        start_vision=lambda: events.append("vision"),
        start_controller=lambda: events.append("controller"),
        health_check=check_health,
        ui_step=lambda: events.append("ui") or True,
        sleep=lambda _seconds: events.append("sleep"),
    )

    with pytest.raises(TypeError, match="navigation worker failed"):
        session.run()

    assert events == [
        "vision",
        "controller",
        "health",
        "ui",
        "health",
        "cleanup",
        "health",
    ]


def test_runtime_session_checks_health_after_controller_cleanup():
    resources = importlib.import_module("navpy.runtime_resources")
    events: list[str] = []
    worker_failed = False
    stack = resources.RuntimeResourceStack()

    def stop_controller() -> None:
        nonlocal worker_failed
        events.append("cleanup")
        worker_failed = True

    def check_health() -> None:
        events.append("health")
        if worker_failed:
            raise ValueError("late navigation failure")

    stack.own_quiescence("controller", stop_controller)
    session = resources.RuntimeSession(
        resources=stack,
        start_vision=lambda: None,
        start_controller=lambda: None,
        health_check=check_health,
        ui_step=lambda: events.append("ui") or False,
        sleep=lambda _seconds: events.append("sleep"),
    )

    with pytest.raises(ValueError, match="late navigation failure"):
        session.run()

    assert events == ["health", "ui", "health", "cleanup", "health"]


def test_runtime_session_aggregates_ui_and_late_worker_failures():
    resources = importlib.import_module("navpy.runtime_resources")
    ui_error = RuntimeError("UI pump failed")
    worker_error = TypeError("late navigation failure")
    worker_failed = False
    stack = resources.RuntimeResourceStack()

    def stop_controller() -> None:
        nonlocal worker_failed
        worker_failed = True

    def check_health() -> None:
        if worker_failed:
            raise worker_error

    stack.own_quiescence("controller", stop_controller)
    session = resources.RuntimeSession(
        resources=stack,
        start_vision=lambda: None,
        start_controller=lambda: None,
        health_check=check_health,
        ui_step=lambda: (_ for _ in ()).throw(ui_error),
        sleep=lambda _seconds: None,
    )

    with pytest.raises(ExceptionGroup) as caught:
        session.run()

    assert caught.value.exceptions == (ui_error, worker_error)


def test_main_delegates_to_composed_runtime_and_propagates_programming_error(monkeypatch):
    main_module = importlib.import_module("navpy.main")

    class FailingRuntime:
        def run(self) -> int:
            raise TypeError("programming defect")

    monkeypatch.setattr(
        main_module,
        "compose_runtime",
        lambda args: FailingRuntime(),
    )

    with pytest.raises(TypeError, match="programming defect"):
        main_module.main(object())


def test_main_returns_runtime_exit_code_without_calling_sys_exit(monkeypatch):
    main_module = importlib.import_module("navpy.main")
    runtime = SimpleNamespace(run=MagicMock(return_value=7))
    monkeypatch.setattr(main_module, "compose_runtime", lambda args: runtime)

    assert main_module.main(object()) == 7
    runtime.run.assert_called_once_with()


def test_refactored_facades_and_main_are_thin_physical_boundaries():
    expected_limits = {
        "src/navpy/modules/vehicle/vehicle_composition.py": 80,
        "src/navpy/modules/vehicle/vehicle_interface.py": 80,
        "src/navpy/logger/navigation_snap.py": 80,
        "src/navpy/main.py": 40,
    }
    for relative, limit in expected_limits.items():
        lines = (REPO_ROOT / relative).read_text(encoding="utf-8").splitlines()
        assert len(lines) <= limit, f"{relative} has {len(lines)} lines"

    main_module = importlib.import_module("navpy.main")
    assert tuple(inspect.signature(main_module.main).parameters) == ("args",)
