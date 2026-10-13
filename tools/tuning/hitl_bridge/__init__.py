"""PC-side JSBSim bridge for hardware-in-the-loop runs (HIL link contract v2 of the fork's SIM_HIL backend).

physics: pure logic (elevon unmix, the Launch and Plant interfaces);
board_clock: physics time on the board clock (whole steps, the contract's causal grid rule,
  reboot, GAP, retransmission and conflict detection);
rail: the catapult rail as measured on the aircraft's launcher (the default launch);
launch_profile: the private launch data (rail and rest pose) the bridge reads with --launch-file;
catapult: the earlier simple launcher (a body-x force from rest, held or on the skids);
state_fields: a plant state to the HIL_STATE_QUATERNION fields, and the state on the simple launcher;
plant_options: SITL's servo lag, added by both plants, and the start pose (rail release, launcher or ground);
environment: a constant wind on the board clock (calm for 5 s after boot, as SITL);
transport: serial, TCP client and TCP listener byte transports (stdlib + pyserial);
link: MAVLink 2 over a transport with reconnect;
jsbsim_process: the JSBSim program (/usr/games/JSBSim in WSL) driven over its sockets (default),
  with jsbsim_files (run folder), jsbsim_protocol (lines and replies), jsbsim_program (process);
jsbsim_plant: the jsbsim Python package in-process (--plant package), and the property names;
frame_loop: one reply and one CSV row per actuator frame;
bridge: the command line (python -m tools.tuning.hitl_bridge);
relay, relay_endpoint: the Windows COM port <-> TCP relay to a bridge in WSL
  (python -m tools.tuning.hitl_bridge.relay).
"""
