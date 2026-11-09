"""The GCS's link rule for a connected vehicle."""

# A vehicle is lost when its autopilot has sent no heartbeat for this long
# (VehicleMav heartbeat_timeout, link_ok). The same rule ages its telemetry
# streams for operator display: a stream silent this long is gone too, even
# while heartbeats still arrive.
VEHICLE_LINK_TIMEOUT_S = 5.0
