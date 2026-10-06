"""Synchronous adapter around the existing renderer and navigation runtime.

Only the opt-in SITL peer uses this adapter. The ordinary vehicle path and
navigation law are unchanged. Truth is used at the rendering boundary only.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time

from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.navigation_log_streams import NavigationLogStreams
from navpy.logger.navigation_logger import NavigationLogger
from navpy.logger.logger_api import ConsoleLogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw
from navpy.modules.navigation.nav.vision_nav.law_config import (
    FixedFinalApproachLawConfigProvider,
)
from navpy.modules.navigation.nav.vision_nav.runtime_composition import (
    FinalApproachVehicleActuator, FinalApproachVehicleDiagnosticReader, compose_final_approach_runtime,
)
from navpy.modules.vehicle.attitude_command import euler_to_quaternion
from navpy.modules.vision.sim.direct_pixel_render import DirectPixelRenderer
from navpy.modules.vision.sim.pose_associator import AssociatedPose
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from scripts.simtime_navigation_config import resolve_config
from scripts.simtime_navigation_protocol import ATTITUDE, CameraSchedule, Snapshot, StepCommand


@dataclass(frozen=True)
class BoundaryAttitude:
    attitude: Attitude
    time_boot_s: float
    receipt_time_s: float
    body_rates_rad_s: tuple[float, float, float]


@dataclass(frozen=True)
class BoundaryTruth:
    location: Location
    attitude: Attitude
    receipt_time_s: float


class StepActuator:
    """Queue one command; transport acceptance is checked at the next tick."""

    def __init__(self) -> None:
        self.command = StepCommand()

    def set_attitude(self, roll: float, pitch: float, *, yaw: None, thr: float | None) -> None:
        if yaw is not None or self.command.kind != 0:
            raise ValueError("nonexclusive step actuator")
        self.command = StepCommand(
            ATTITUDE, 196 if thr is None else 132,
            tuple(euler_to_quaternion(roll, pitch, 0.0)), 0.0 if thr is None else thr)


class SynchronousNavigation:
    def __init__(self, dock: Location, *, speedup: float,
                 camera_fraction: tuple[int, int] = (4, 5),
                 wall_now: Callable[[], float] = time.monotonic,
                 output: Path | None = None,
                 trim_throttle_percent: float | None = None,
                 configured_throttle_percent: float | None = None) -> None:
        if not math.isfinite(speedup) or speedup <= 0:
            raise ValueError("invalid speedup")
        self._wall_now, self._speedup = wall_now, speedup
        self._trim_throttle_percent = trim_throttle_percent
        self._configured_throttle_percent = configured_throttle_percent
        self._source_s = 0.0
        self._camera = CameraSchedule(*camera_fraction)
        self._actuator = StepActuator()
        self._composition = None
        self._logger: NavigationLogger | None = None
        self._output = output
        self._limits: tuple[float, ...] | None = None
        self.seed_step: int | None = None
        self.command_count = 0
        self._drained = False
        self._renderer = DirectPixelRenderer(
            dock, source_name="simtime-ideal360", aircraft_sequence="ZYX",
            aircraft_degrees=True, frame_size=FrameSize(1920, 1080),
            source_now_s=lambda: self._source_s, wall_now_s=wall_now)

    def _compose(self, snapshot: Snapshot) -> None:
        self._limits = snapshot.limits
        config = resolve_config(snapshot.limits, self._trim_throttle_percent, self._configured_throttle_percent)
        streams = NavigationLogStreams(
            None if self._output is None else (self._output / "navigation_compact.csv").open("x"),
            None if self._output is None else (self._output / "navigation_debug.csv").open("x"))
        self._logger = NavigationLogger(
            snapshot.identity.vehicle, ConsoleLogger(CacheLogLevel.ERROR),
            time_source=lambda: self._source_s, wall_time=self._wall_now,
            timestamp=lambda: f"{self._source_s:.6f}", streams=streams)
        lock = threading.RLock()
        self._composition = compose_final_approach_runtime(
            lock=lock, slot=NavigationCommandSlot(lock, threading.Event()),
            sys_id=snapshot.identity.vehicle,
            actuator=FinalApproachVehicleActuator(self._actuator.set_attitude),
            diagnostic_reader=FinalApproachVehicleDiagnosticReader(lambda: None, lambda: None),
            law=VisionNavLaw(FixedFinalApproachLawConfigProvider(config)),
            aircraft_roll_deg=lambda: 0., aircraft_sequence="ZYX", aircraft_degrees=True,
            navigation_logger=self._logger, wall_period_s=lambda period: period / self._speedup)

    def advance(self, snapshot: Snapshot, receipt_s: float) -> tuple[StepCommand, dict]:
        if self._drained:
            raise ValueError("navigation already drained")
        age = self._wall_now() - receipt_s
        if not math.isfinite(age) or not 0 <= age <= 1.0 / self._speedup:
            raise ValueError("receipt liveness exceeded")
        self._source_s = snapshot.identity.source_us / 1_000_000
        self._actuator.command = StepCommand()
        if snapshot.mode != 15 or not snapshot.armed:  # ArduPlane GUIDED=15
            if self._composition is not None:
                raise ValueError("navigation phase lost mode/arming")
            return self._actuator.command, {"active": False}
        if self._composition is None:
            self._compose(snapshot)
        if snapshot.limits != self._limits:
            raise ValueError("law configuration changed")
        runtime = self._composition.runtime
        evidence = {"active": True, "captured": self._camera.due()}
        if evidence["captured"]:
            obs, truth = snapshot.observation, snapshot.truth
            attitude = BoundaryAttitude(
                Attitude(math.degrees(obs.pitch_rad), 0., math.degrees(obs.roll_rad)),
                self._source_s, receipt_s, obs.rates_rad_s)
            render = BoundaryTruth(
                Location(truth.latitude, truth.longitude, truth.altitude, is_absolute=True),
                Attitude(truth.pitch_deg, truth.yaw_deg, truth.roll_deg), receipt_s)
            associated = AssociatedPose(attitude, render, self._source_s, receipt_s,
                                        receipt_s, obs.airspeed_mps)
            detection = self._renderer.render(associated, (render.location, render.attitude))
            if detection is None:
                raise ValueError("camera geometry unavailable")
            evidence["pixels"] = [detection.pixel.u_px, detection.pixel.v_px]
            if self.seed_step is None:
                if not self._composition.confirmation.record_final_approach_confirmed_detection(detection):
                    raise ValueError("confirmation seed unavailable")
                self.seed_step = snapshot.identity.step
            if not runtime.nav(detection):
                raise ValueError("frame admission rejected")
        work = runtime.take_work()
        if work is not None:
            try:
                runtime.execute_work(work)
                postprocess = runtime.postprocess_job(work)
                if postprocess is not None:
                    postprocess()
            finally:
                runtime.finish_work(work)
        if runtime.consume_command_liveness_failure():
            raise ValueError("runtime receipt liveness failure")
        evidence["passed"] = runtime.poi_passed_override()
        if self._actuator.command.kind == ATTITUDE:
            self.command_count += 1
        return self._actuator.command, evidence

    def close(self) -> None:
        if self._composition is not None:
            self._composition.runtime.invalidate_commands()
        if self._logger is not None:
            self._logger.close()

    def drain(self) -> dict:
        """Stop admission before the last command is acknowledged."""
        self._drained = True
        evidence = {"active": self._composition is not None, "captured": False,
                    "draining": True}
        if self._composition is not None:
            evidence["passed"] = self._composition.runtime.poi_passed_override()
            self._composition.runtime.invalidate_commands()
        return evidence
