import ast
import inspect
import textwrap
from contextlib import nullcontext
from pathlib import Path
from unittest.mock import MagicMock

from navpy.modules.navigation.nav.vision_nav import command_executor
from navpy.modules.navigation.nav.vision_nav import command_freshness
from navpy.modules.navigation.nav.vision_nav import command_hold
from navpy.modules.navigation.nav.vision_nav import command_transaction
from navpy.modules.navigation.nav.vision_nav import law
from navpy.modules.navigation.nav.vision_nav import rate_filter
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    TerminalFrameProjector,
)
from navpy.modules.navigation.nav.vision_nav.ingress import TerminalIngress
from navpy.modules.nav.terminal_navigation import (
    NavSourceBatch,
    TerminalCommandDispatch,
    TerminalCommandPorts,
    TerminalNavPorts,
    TerminalNavWorkflow,
)


def _names_and_attrs(value):
    tree = ast.parse(textwrap.dedent(inspect.getsource(value)))
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            found.add(node.id)
        elif isinstance(node, ast.Attribute):
            found.add(node.attr)
        elif isinstance(node, ast.Call) and len(node.args) >= 2:
            name = getattr(node.func, "id", getattr(node.func, "attr", None))
            key = node.args[1]
            if name in {"getattr", "hasattr", "setattr"} and isinstance(key, ast.Constant):
                found.add(key.value)
    return found


def test_command_core_has_no_forbidden_hardware_or_truth_sources():
    forbidden = {
        "heading", "yaw", "velocity", "ground_speed", "ground_speed_ned",
        "location", "altitude", "relative_altitude", "wind", "reference_height_m",
        "bbox_cxcywh", "tracking_bbox_cxcywh", "t_g_loc_debug", "c_g_loc",
        "is_simulation", "DetectedObject", "uas_body_rates_rad_s",
    }
    for module in (
        law,
        rate_filter,
        command_transaction,
        command_executor,
        command_freshness,
        command_hold,
    ):
        source = Path(module.__file__).read_text(encoding="utf-8")
        names = _names_and_attrs(module)
        assert forbidden.isdisjoint(names), module.__name__
        assert "DetectedObject" not in source


def test_aircraft_projection_helper_reads_pitch_roll_only():
    source = inspect.getsource(TerminalFrameProjector.project)
    names = _names_and_attrs(TerminalFrameProjector.project)
    assert "aircraft_pitch_deg" in names
    assert "aircraft_roll_deg" in names
    assert "aircraft_yaw_deg" not in names
    assert "heading" not in names
    assert "yaw=0.0" in source


def test_ingress_delegates_all_rich_target_reads_to_projector():
    names = _names_and_attrs(TerminalIngress.nav)
    assert "g_data" not in names
    assert "uas_att" not in names
    assert "timestamp" not in names


def test_nav_dispatch_has_no_gap_command_capability():
    assert "nav_gap" not in TerminalCommandPorts.__dataclass_fields__
    terminal_nav_files = (
        "src/navpy/modules/nav/terminal_command_dispatch.py",
        "src/navpy/modules/nav/terminal_navigation.py",
        "src/navpy/modules/nav/terminal_nav_workflow.py",
        "src/navpy/modules/nav/terminal_source_admission.py",
    )
    for relative in terminal_nav_files:
        assert "nav_without_detection" not in Path(relative).read_text(
            encoding="utf-8"
        )


def test_non_source_dispatch_passes_only_fresh_detection():
    fresh = MagicMock()
    nav = MagicMock(return_value=True)
    commands = TerminalCommandDispatch(
        TerminalCommandPorts(
            nav=nav,
            mark_failed=MagicMock(),
            logger=MagicMock(),
        ),
        MagicMock(),
    )
    commands.dispatch(
        NavSourceBatch(False, None, (), False),
        fresh,
        MagicMock(),
    )
    nav.assert_called_once_with(fresh)


def test_source_driven_tick_without_event_waits_without_failure_or_command():
    target = MagicMock()
    source = MagicMock()
    batch = NavSourceBatch(True, None, (), False)
    source.collect.return_value = batch
    source.frame_ready.return_value = False
    commands = MagicMock()
    event_pump = MagicMock()
    event_pump.is_open = False
    workflow = TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: __import__(
                "navpy.modules.vehicle.flight_mode", fromlist=["FlightMode"]
            ).FlightMode.GUIDED,
            request_guided=MagicMock(),
            mark_guided_session=MagicMock(),
            active_target=lambda: target,
            vision_nav_active=lambda: True,
            command_liveness_failed=lambda: False,
        ),
        source,
        commands,
        MagicMock(),
        MagicMock(),
        event_pump,
        nullcontext,
    )
    workflow.act_nav()
    commands.mark_no_detection.assert_not_called()
    commands.dispatch.assert_not_called()


def test_mode_loss_closes_source_pump_before_requesting_guided_again():
    request_guided = MagicMock()
    pump = MagicMock()
    pump.is_open = True
    workflow = TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: __import__(
                "navpy.modules.vehicle.flight_mode", fromlist=["FlightMode"]
            ).FlightMode.AUTO,
            request_guided=request_guided,
            mark_guided_session=MagicMock(),
            active_target=MagicMock(),
            vision_nav_active=lambda: True,
            command_liveness_failed=lambda: False,
        ),
        MagicMock(),
        MagicMock(),
        MagicMock(),
        MagicMock(),
        pump,
        nullcontext,
    )

    workflow.act_nav()

    request_guided.assert_called_once_with()
    pump.close.assert_called_once_with()


def test_command_liveness_failure_stops_open_source_session():
    target = MagicMock()
    source = MagicMock()
    commands = MagicMock()
    peers = MagicMock()
    pump = MagicMock()
    pump.is_open = True
    workflow = TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: __import__(
                "navpy.modules.vehicle.flight_mode", fromlist=["FlightMode"]
            ).FlightMode.GUIDED,
            request_guided=MagicMock(),
            mark_guided_session=MagicMock(),
            active_target=lambda: target,
            vision_nav_active=lambda: True,
            command_liveness_failed=lambda: True,
        ),
        source,
        commands,
        MagicMock(),
        peers,
        pump,
        nullcontext,
    )

    workflow.act_nav()

    commands.mark_source_liveness_expired.assert_called_once_with(target)
    pump.close.assert_called_once_with()
    source.collect.assert_not_called()
    peers.submit.assert_not_called()


def test_empty_current_publication_fails_without_gap_or_confirmation_deferral():
    target = MagicMock(task_id=1, obj_id=2)
    ports = TerminalCommandPorts(
        nav=MagicMock(),
        mark_failed=MagicMock(),
        logger=MagicMock(),
    )
    source = MagicMock()
    source.source_event_target.return_value = None
    commands = TerminalCommandDispatch(ports, source)
    event = MagicMock()
    commands.dispatch(NavSourceBatch(True, event, (), False), None, target)
    ports.mark_failed.assert_called_once_with()
    ports.nav.assert_not_called()


def test_successful_vision_source_bootstrap_opens_exclusive_pump():
    target = MagicMock()
    event = MagicMock()
    batch = NavSourceBatch(True, event, (), False)
    source = MagicMock()
    source.find_active_target_detection.return_value = target
    order = []
    source.collect.side_effect = lambda *_args: order.append("collect") or batch
    source.frame_ready.return_value = True
    source.consume.return_value = True
    commands = MagicMock()
    commands.dispatch.side_effect = (
        lambda *_args: order.append("dispatch") or True
    )
    record = MagicMock()
    record.commit.return_value = True
    pump = MagicMock()
    pump.is_open = False
    pump.prepare.side_effect = lambda: order.append("prepare") or True
    pump.activate.side_effect = lambda: order.append("activate") or True
    workflow = TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: __import__(
                "navpy.modules.vehicle.flight_mode", fromlist=["FlightMode"]
            ).FlightMode.GUIDED,
            request_guided=MagicMock(),
            mark_guided_session=MagicMock(),
            active_target=lambda: target,
            vision_nav_active=lambda: True,
            command_liveness_failed=lambda: False,
        ),
        source,
        commands,
        record,
        MagicMock(),
        pump,
        nullcontext,
    )

    workflow.act_nav()

    pump.prepare.assert_called_once_with()
    pump.activate.assert_called_once_with()
    pump.open.assert_not_called()
    assert order == ["collect", "prepare", "dispatch", "activate"]


def test_polling_vision_dispatch_stays_on_worker_path_without_event_pump():
    target = MagicMock()
    batch = NavSourceBatch(False, None, (), False)
    source = MagicMock()
    source.find_active_target_detection.return_value = target
    source.collect.return_value = batch
    source.frame_ready.return_value = True
    commands = MagicMock()
    commands.dispatch.return_value = True
    record = MagicMock()
    record.commit.return_value = True
    pump = MagicMock()
    pump.is_open = False
    workflow = TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: __import__(
                "navpy.modules.vehicle.flight_mode", fromlist=["FlightMode"]
            ).FlightMode.GUIDED,
            request_guided=MagicMock(),
            mark_guided_session=MagicMock(),
            active_target=lambda: target,
            vision_nav_active=lambda: True,
            command_liveness_failed=lambda: False,
        ),
        source,
        commands,
        record,
        MagicMock(),
        pump,
        nullcontext,
    )

    workflow.act_nav()

    commands.dispatch.assert_called_once_with(batch, target, target)
    pump.open.assert_not_called()


def test_source_driven_legacy_bootstrap_does_not_open_pump():
    target = MagicMock()
    batch = NavSourceBatch(True, MagicMock(), (), False)
    source = MagicMock()
    source.find_active_target_detection.return_value = target
    source.collect.return_value = batch
    source.frame_ready.return_value = True
    source.consume.return_value = True
    commands = MagicMock()
    commands.dispatch.return_value = True
    record = MagicMock()
    record.commit.return_value = True
    pump = MagicMock()
    pump.is_open = False
    workflow = TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: __import__(
                "navpy.modules.vehicle.flight_mode", fromlist=["FlightMode"]
            ).FlightMode.GUIDED,
            request_guided=MagicMock(),
            mark_guided_session=MagicMock(),
            active_target=lambda: target,
            vision_nav_active=lambda: False,
            command_liveness_failed=lambda: False,
        ),
        source,
        commands,
        record,
        MagicMock(),
        pump,
        nullcontext,
    )

    workflow.act_nav()

    pump.open.assert_not_called()
