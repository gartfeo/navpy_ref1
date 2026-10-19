"""Source-time debug streams and their CSV headers (``pose_cadence_debug``).

Event streams, keyed by ``(sys_id, stream)``:

- ``attitude_arrival`` -- one row per ATTITUDE that is ACCEPTED and updates the
  pose cache (``vehicle_mav._process_message``, after
  ``_accept_telemetry_sample``). ``boot_ms`` is the autopilot sample time
  (sim-time under SIM_SPEEDUP); the ``boot_ms`` delta approximates ArduPilot's
  accepted emission cadence in SIM time, the ``wall_s`` delta the
  transport/reader cadence.
- ``position_arrival`` -- same, for GLOBAL_POSITION_INT (the position half of
  the pose the sim detector reads without a timestamp).
- ``telemetry_reject`` -- one row per autopilot-telemetry packet that
  ``_accept_telemetry_sample`` refused (foreign component / stale boot time),
  making the drop path countable instead of silent.
- ``detector_pose`` -- source-pose admission and final-approach detector outcomes:
  raw attitude boot time, accepted poses, emitted frames, explicit queue /
  reset / invalidation drops, missing timestamps, duplicates, and reordering.
  Only an actual frame publication is ``emitted``; its unchanged raw source
  timestamp appears in ``frame_ts``.
- ``detector_poll`` -- one row per wall-paced detector poll iteration, empty
  reads included (``sim.detection.detector_poll_timer``): work and thread-CPU
  time of one scope, requested and actual wait, stage costs and the previous
  iteration's residual. ``detector_pose`` rows skip polls that read nothing.
- ``observation`` -- one row per detection delivered to final approach
  (``runtime.nav``): the frame's source timestamp, the source clock's "now"
  read through the detection's own provider, and the hand-off ``outcome``
  (fresh / duplicate / regression / invalid_*) and the frame's source
  ``epoch`` (the final-approach source generation, empty without a frame).
- ``worker`` -- one row per final-approach command actually SENT
  (``execute_final_approach_command``, every command -- not decimated like the
  FINAL_APPROACH_CMD log event): raw observation source timestamp, matching source-
  clock now, diagnostic execution time, and the compatibility marker
  ``measured``, and whether the 25 Hz tick used a ``fresh`` command or a
  zero-order ``held`` primitive command, plus ``work_cpu_ms`` (thread CPU time
  of the ``exec_ms`` scope) and the frame's source ``epoch``. The current
  final-approach runtime does not issue predictions.
- ``frame_lineage`` -- what became of each fresh frame after the observation
  hop, keyed (``epoch``, ``frame_ts``): ``replaced`` in the newest-only slot
  before the worker took it, or the worker's ``consumed`` (issued) /
  ``unissued`` / ``stale`` (not executed). A fresh observation with no row was
  still unfinished when the log ended.

Emitted ``detector_pose`` rows carry an ``epoch`` too: the count of
source-discontinuity publications drained before that frame, which matches
the final-approach generation while every emitted discontinuity reaches it.
"""

HEADERS = {
    "attitude_arrival": "wall_s,boot_ms\n",
    "position_arrival": "wall_s,boot_ms\n",
    "telemetry_reject": "wall_s,mtype,reason\n",
    "detector_pose": "wall_s,boot_s,outcome,frame_ts,epoch\n",
    "detector_poll": (
        "start_wall_s,outcome,work_ms,work_cpu_ms,requested_wait_ms,"
        "actual_wait_ms,read_ms,project_ms,publish_ms,log_ms,residual_before_ms\n"
    ),
    "observation": "wall_s,obs_ts,src_now_s,outcome,epoch\n",
    "worker": "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome,work_cpu_ms,epoch\n",
    "frame_lineage": "wall_s,stage,epoch,frame_ts,disposition\n",
}

__all__ = ["HEADERS"]
