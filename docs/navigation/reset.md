# Navigation RESET

`RESET` is a deliberate fresh start. It clears POI statuses, task ids,
cooldowns and the UAV's auction state, then restarts the AUTO mission at
waypoint 1. This repeats the search without another takeoff, so picking the
same docks again afterwards is expected.

Accepted corner case: a UAV that resets may pick a dock that a peer is still
serving from before the RESET.
