"""Names the lockstep firmware defines; the harness must spell them exactly.

The firmware (`ArduPlane/NavPyGuidanceStep.cpp`, `NavPyStepProbe.cpp` in the
lockstep firmware root) reads these environment variables, prints these log
markers and creates these files in the instance directory. Nothing else in the
harness may spell them as a literal. `tests/scripts/test_simtime_firmware_contract.py`
checks each one against the firmware binary.
"""

# Guidance (closed-loop) window.
GUIDANCE_MODE_ENV = "NAVPY_GUIDANCE_MODE"
GUIDANCE_START_US_ENV = "NAVPY_GUIDANCE_START_US"
GUIDANCE_COUNT_ENV = "NAVPY_GUIDANCE_COUNT"
GUIDANCE_MODE_CLOSED_LOOP = "closed-loop"
GUIDANCE_COMPLETE_MARKER = "NAVPY_GUIDANCE_COMPLETE"
GUIDANCE_INVALID_MARKER = "NAVPY_GUIDANCE_INVALID"
GUIDANCE_SOCKET = "navpy-guidance.sock"
GUIDANCE_CSV = "navpy-guidance.csv"

# Pose capture (valid only with the guidance window).
POSE_CAPTURE_ENV = "NAVPY_POSE_CAPTURE"
RENDER_POSE_ENV = "NAVPY_RENDER_POSE"
POSE_CSV = "navpy-pose.csv"

# Step probe (observe / handshake).
STEP_MODE_ENV = "NAVPY_STEP_MODE"
STEP_START_US_ENV = "NAVPY_STEP_START_US"
STEP_COMPLETE_MARKER = "NAVPY_STEP_COMPLETE"
STEP_INVALID_MARKER = "NAVPY_STEP_INVALID"
STEP_SOCKET = "navpy-step.sock"
STEP_CSV = "navpy-step.csv"

# Everything the contract test must find in the firmware binary.
FIRMWARE_NAMES = (
    GUIDANCE_MODE_ENV, GUIDANCE_START_US_ENV, GUIDANCE_COUNT_ENV, GUIDANCE_MODE_CLOSED_LOOP,
    GUIDANCE_COMPLETE_MARKER, GUIDANCE_INVALID_MARKER, GUIDANCE_SOCKET, GUIDANCE_CSV,
    POSE_CAPTURE_ENV, RENDER_POSE_ENV, POSE_CSV,
    STEP_MODE_ENV, STEP_START_US_ENV, STEP_COMPLETE_MARKER, STEP_INVALID_MARKER,
    STEP_SOCKET, STEP_CSV,
)

# Harness-internal: printed by this repo's Python peers, not by the firmware.
GUIDANCE_PEER_READY_MARKER = "NAVPY_GUIDANCE_PEER_READY"
STEP_PEER_READY_MARKER = "NAVPY_STEP_PEER_READY"
