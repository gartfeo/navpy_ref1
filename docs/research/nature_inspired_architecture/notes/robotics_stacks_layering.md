# Layering, missions, skills and plug-in modularity in open robotics and drone stacks

Scope: how Nav2, BehaviorTree.CPP, PX4, ArduPilot, Aerostack2, MOOS-IvP, skill frameworks (SkiROS2, RAFCON) and run-time assurance (Simplex, ASTM F3269) split mission logic, reusable actions, motion primitives, safety overrides and swappable components. The aim is to judge the proposed design: Mission -> Skills (search, confirm, observe, acquire, approach) -> Moves (goto, hold, orbit, climb; autopilot plugin) -> Reflexes (failsafes, geofence, supervisor hold), with perception plugins, approach-law plugins that only map inputs to commands, and plugins chosen at mission setup. All sources were accessed on 2026-10-09. Version tags are given in [brackets].

## 1. ROS 2 Nav2: BT navigator, BT nodes, plugin interfaces, lifecycle nodes, recoveries and collision-monitor override

### Takeaway
Nav2 has three tiers. A BT navigator (mission/task orchestration) ticks BT nodes that call ROS 2 action servers (planner, controller, behavior/recovery). Those servers host swappable algorithm plugins selected by parameter. Below the controller sits an independent Collision Monitor that filters the velocity command and always applies the most aggressive triggered action. Lifecycle nodes plus a bond-based lifecycle manager give deterministic bring-up and take the whole system down if a server dies.

### Cited Findings
- Nav2 (Navigation2) "uses a behavior tree for navigator task orchestration" and is built on ROS 2 [IROS 2020 paper] — [Macenski et al., The Marathon 2](https://arxiv.org/abs/2003.00368)
- The BT Navigator implements the NavigateToPose and NavigateThroughPoses task interfaces. It is "intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors, including recovery." Navigators are themselves plugins implementing `nav2_core::BehaviorTreeNavigator` (defaults: navigate_to_pose, navigate_through_poses) — [Nav2 BT Navigator config](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)
- `NavigatorBase` [Jazzy] declares configure/activate/deactivate hooks. Each navigator creates its own BT action server with goal-received, loop, preempt and completion callbacks. A mutex rejects overlapping navigations, and a second request logs "likely occurred from an incorrect implementation of a navigator plugin" — [Nav2 API Jazzy NavigatorBase](https://api.nav2.org/nav2-jazzy/html/classnav2__core_1_1NavigatorBase.html), [BtNavigator Jazzy](https://api.nav2.org/nav2-jazzy/html/classnav2__bt__navigator_1_1BtNavigator.html)
- `nav2_behavior_tree` uses BehaviorTree.CPP and provides a C++ template (`BtActionNode`) that wraps a ROS 2 action as a BT node registered with the factory. Custom BT node libraries are listed in `plugin_lib_names`. From Jazzy on, built-in Nav2 BT libraries load automatically and only custom ones need listing — [nav2_behavior_tree package docs](https://docs.ros.org/en/iron/p/nav2_behavior_tree), [BT Navigator config](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)
- BT node catalogue [Jazzy docs branch]:
  - Control nodes: `PipelineSequence`, `RoundRobin`, `RecoveryNode`.
  - Decorators: `RateController`, `DistanceController`, `SpeedController`, `GoalUpdater`.
  - Conditions: `GoalReached`, `IsStuck`, `TimeExpired`, `IsPathValid`.
  - Actions: `ComputePathToPose`, `FollowPath`, `Spin`, `BackUp`.
  - The minimal example tree is `PipelineSequence` + `DistanceController` + `ComputePathToPose` + `FollowPath`.
  - The page says Nav2 "allows users to set many different plugin types, across behavior trees, core algorithms, status checkers, and more" — [Nav2 Behavior Trees (Jazzy)](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/)
- Task servers host algorithm plugins:
  - **Planner server:** runs the named planner plugin (NavFn, Smac 2D/Hybrid-A*/Lattice, Theta*) — [Planner Server](https://docs.nav2.org/configuration/packages/configuring-planner-server.html)
  - **Controller server:** DWB uses trajectory-generator plus critic plugins; MPPI, RPP and TEB are alternatives — [Setting Up Navigation Plugins](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)
  - **Behavior server:** hosts recovery/docking behavior plugins (spin, backup, drive-on-heading, wait, assisted teleop) that share costmaps and TF to lower the cost of new behaviors [Rolling] — [Behavior Server](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/)
- Which algorithm runs for a given task "can be done through the behavior tree of the navigation system or application server", so the BT chooses among plugins at runtime by ID — [Navigation Concepts (Rolling)](https://docs.nav2.org/rolling/getting_started/navigation_concepts/)
- Lifecycle:
  - Nodes move through Unconfigured, Inactive, Active and Finalized. The lifecycle manager configures and activates nodes in list order and shuts them down in reverse.
  - Bond connections let the manager "take all nodes down if one is non-responsive or has crashed" (bond_timeout default 4.0 s, about 10 s reconnect window).
  - [Lyrical, 2026] The `bond_heartbeat_period` default changed from 0.1 to 0.25 s.
  - Sources: [Lifecycle Manager](https://docs.nav2.org/configuration/packages/configuring-lifecycle.html), [Navigation Concepts](https://docs.nav2.org/rolling/getting_started/navigation_concepts/), [Kilted to Lyrical migration](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/kilted/Kilted/)
- Collision Monitor [Jazzy]: "a node providing an additional level of robot safety" that works on raw sensor data, "bypassing the costmaps and trajectory planners", to "monitor for and prevent potential collisions at the emergency-stop level" by limiting velocity commands — [Collision Monitor config (Jazzy)](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/)
- Collision Monitor details, via search snippets of the official tutorial and README:
  - It filters the controller's cmd_vel (input cmd_vel_smoothed, output cmd_vel).
  - Zones are polygons or circles with stop, slowdown and approach (time-to-collision) models.
  - "When multiple zones trigger at once, the most aggressive one is used."
  - It "does not provide hard real-time safety certification".
  - A sibling Collision Detector only reports and does not modify commands.
  - Sources: [Using Collision Monitor tutorial](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html), [README (Jazzy)](https://api.nav2.org/nav2-jazzy/html/md_nav2_collision_monitor_README.html)
- The Collision Monitor was added in Iron and was not in the default nav2_bringup at that time — [Humble to Iron migration](https://navigation.ros.org/migration/Humble.html)
- Current Nav2 API docs exist for Humble, Jazzy, Kilted and Lyrical (the 2026 distro) — [api.nav2.org Lyrical BtNavigator](https://api.nav2.org/nav2-lyrical/html/classnav2__bt__navigator_1_1BtNavigator.html)

### Inferences
- Nav2 maps closely onto the proposal:
  - BT navigator = Mission.
  - BT action nodes + task servers = Skills.
  - Controller/planner/behavior plugins = Moves / approach laws.
  - Collision Monitor = Reflex.
- The key safety pattern is placement, not intelligence. The reflex sits between the command producer and the actuator as a pure filter, needs no mission knowledge, and resolves conflicts by "most aggressive wins". A NavPy supervisor hold should likewise filter or replace the Move output, not be a branch inside the mission tree.
- Nav2 recoveries are ordinary BT branches (`RecoveryNode`) calling behavior-server plugins, so they are *mission-level* recovery and distinct from *reflex-level* override. The proposal should keep this distinction explicit. "Supervisor hold" as a reflex differs from a skill-level "retry search".
- Nav2 has no per-plugin conformance test suite in the sources reviewed. Interface enforcement comes from C++ base classes plus lifecycle hooks plus a mutex rejecting overlapping tasks.

### Gaps
- I did not retrieve a Nav2 page that explicitly names `pluginlib`/`PLUGINLIB_EXPORT_CLASS` (the plugin-tutorial URL returned 404). It is widely known that Nav2 loads plugins via pluginlib, but that was not confirmed from a fetched source in this session.
- The default NavigateToPose XML (exact `RecoveryNode`/`RoundRobin` structure) was not retrieved.
- I did not fetch the exact Collision Monitor tutorial wording; those claims come from search snippets of the official pages.

## 2. BehaviorTree.CPP and BTs vs FSMs / HSMs / graphs for mission logic

### Takeaway
BTs win where reactivity plus modular reuse of sub-behaviors matters (Nav2, Aerostack2, SkiROS2 all use BehaviorTree.CPP-style trees). State machines remain common for explicit sequential mission phases (PX4 mode executors, RAFCON HSMs, MOOS-IvP hierarchical modes). The largest empirical study (Ghzouli et al., TSE 2023) found BTs and state machines are used with *similar* structure and reuse patterns in real ROS projects. The choice matters less than the outcome contract between layers.

### Cited Findings
- BehaviorTree.CPP [4.8 docs; 3.8 also listed]:
  - Nodes declare typed input, output and bidirectional ports in a static `providedPorts()`. Ports can point to blackboard entries.
  - Connecting ports of different types makes `createTreeFromFile` "throw an exception" (load-time type checking).
  - Nodes are registered with `BehaviorTreeFactory::registerNodeType<T>()`.
  - Source: [BehaviorTree.CPP ports tutorial](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_ports)
- Colledanchise & Ögren:
  - The book [CRC Press 2018; arXiv v6 Oct 2022] defines a BT as "a way to structure the switching between different tasks in an autonomous agent".
  - It claims BTs are "a very efficient way of creating complex systems that are both modular and reactive" and "in many cases generalize, earlier switching structures".
  - Source: [Colledanchise & Ögren, arXiv 1709.00084](https://arxiv.org/abs/1709.00084)
- Iovino et al. survey:
  - Published in Robotics and Autonomous Systems 2022, art. 104096, DOI 10.1016/j.robot.2022.104096; reviewed over 160 papers.
  - BTs were "invented as a tool to enable modular AI in computer games". Game developers found FSMs "scaled poorly and were difficult to extend, adapt and reuse".
  - BTs place transition logic in a hierarchical tree with states as leaves, which "has a significant effect on modularity".
  - Sources: [arXiv 2005.05842](https://arxiv.org/abs/2005.05842), [KTH bib record (RAS2022)](https://www.csc.kth.se/~ccs/Publications/RAS2022_BTSurvey.bib)
- Ghzouli et al. 2020 [SLE 2020]: "Behavior Trees are a pragmatic language, not fully specified", "allowing projects to extend it even for just one model". BTs "clearly resemble the models-at-runtime paradigm" — [arXiv 2010.06256](https://arxiv.org/abs/2010.06256)
- Ghzouli et al. 2023 [IEEE TSE 49(9), 2023; arXiv rev. Oct 2025]:
  - "usage of behavior-tree DSLs in open-source projects is increasing rapidly".
  - The authors "observed similar usage patterns at model structure and at code reuse in the behavior-tree and state-machine models" and "identify similarities between behavior trees and state machines in terms of language design".
  - Source: [arXiv 2208.04211](https://arxiv.org/abs/2208.04211)
- RAFCON (DLR) [IROS 2016]:
  - Missions are modeled as hierarchical state machines with concurrency (later preemptive and barrier concurrency, library states).
  - Python-based, so state machines can be edited while running.
  - State machines are stored as a JSON folder tree that is "version controllable".
  - Used for SpaceBotCamp and the ROBEX Mt. Etna seismometer deployment.
  - Sources: [DLR elib record](https://elib.dlr.de/112067), [arXiv 1605.09185](https://arxiv.org/html/1605.09185), [humanoid loco-manipulation paper 2026](https://arxiv.org/pdf/2606.26425), [Brunner talk, York](https://www.cs.york.ac.uk/robostar/robosoft/talks/sebastian-brunner/)

### Inferences
- **For NavPy's Mission layer:** a sequential FSM/HSM is adequate for phase flow (search -> confirm -> observe -> acquire -> approach -> return). A BT pays off when skills need continuous condition re-checking and fallback (for example "POI still confirmed?" guarding approach, or fallback from acquire to search). Both are defensible. What matters is that every Skill returns a uniform outcome (running/success/failure, plus a reason) so either executor can drive it.
- BehaviorTree.CPP's typed ports are a cheap, proven way to enforce "approach-law plugins only map declared inputs to commands". A plugin's allowed inputs are declared and checked at load time. This could mechanically enforce input whitelists such as the project's pure-vision constraint.
- RAFCON's JSON-tree mission files and Aerostack2's JSON mission interpreter (section 5) suggest that missions as data, versioned alongside code, are mainstream.

### Gaps
- The Colledanchise & Ögren full text (the "goto"/one-way control transfer argument against FSMs, and HFSM drawbacks) could not be fetched; the PDF exceeded the size limit. Only abstract-level claims are cited.
- The Ghzouli TSE full text (quantitative counts, hybrid BT+FSM usage, when each is preferred) was not retrieved. Only abstract claims are cited.
- I did not retrieve BehaviorTree.CPP plugin and shared-library loading docs (`BT_REGISTER_NODES`), or its versioning policy.

## 3. PX4: flight modes, ROS 2 Interface Library (external modes, mode executors), flight tasks, failsafe state machine; MAVSDK

### Takeaway
PX4 is the closest analogue to the proposed Moves + Reflexes split:
- External ROS 2 modes register as first-class flight modes, and can even replace internal modes.
- A mode executor (a state machine owning one mode) sequences modes. It loses control on any user mode switch or failsafe and is reactivated when the failsafe clears.
- The failsafe state machine always overrides, picking the most severe action when several trigger.
- A crashed or unresponsive external mode itself triggers a failsafe.

### Cited Findings
- **ROS 2 Control Interface** [introduced as "PX4 v1.15 Experimental"; FW lateral/longitudinal and rover setpoints added v1.17]:
  - External modes register at runtime (`doRegister()`) and appear to the GCS as normal modes.
  - A mode "must be activated by the user (through RC/GCS), the flight controller in a failsafe situation" or an executor.
  - Modes "can even replace the default modes in PX4 with enhanced ROS 2 versions". The internal mode is "only used as a fallback when the external one becomes unresponsive or crashes".
  - Source: [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)
- **Mode executor:**
  - "An optional component for scheduling modes", "a state machine that can activate modes, and wait for their completion". It uses `onActivate()` and `scheduleMode()`, and the next state runs only on a success `Result`.
  - "a mode can be owned by at most one executor". "Executors cannot activate other executors."
  - The executor stays in charge "until the user switches modes (by RC or from a GCS), or a failsafe triggers a mode switch". When the failsafe clears, "the executor gets reactivated".
  - Source: [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)
- **Failsafe integration** — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface):
  - "Modes are integrated with the failsafe state machine and arming checks."
  - Modes declare requirements (for example a valid position estimate). If unmet, "arming is not allowed, while the mode is selected", and once armed "the relevant failsafe is triggered".
  - "A failsafe is also triggered when the mode crashes or becomes unresponsive while it is selected."
  - A mode or executor "can temporarily defer non-essential failsafes" (`deferFailsafesSync()`, `onFailsafeDeferred()`), for example to finish a winch operation.
- **Setpoint types** (the "Moves" PX4 exposes to external modes) — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface):
  - Go-to (MC), FW lateral/longitudinal (v1.17), direct actuators, and rover setpoints (v1.17).
  - "Only a few setpoint types have settled."
  - Architecture and mode/executor interfaces are "largely stable, and are tested in CI". "The API is not fully documented."
- **Failsafe behavior** [main docs; v1.17 stable; items tagged v1.18 and "main (PX4 v2.0)"]:
  - Most failsafes first "enter Hold for COM_FAIL_ACT_T seconds before performing an associated failsafe action". Stick movement in that window is not an override; overrides are done by mode switch.
  - Actions in increasing severity: Warning, Hold, Return, Land, Disarm, Terminate. "If multiple failsafes are triggered, the more severe action is taken."
  - Per-trigger parameters include RC loss, data-link loss (`COM_DLL_EXCEPT` per-mode exceptions), geofence (`GF_ACTION`), position loss, offboard loss (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`), traffic avoidance and high wind.
  - A separate failsafe state machine simulation page exists for testing combined triggers.
  - Source: [PX4 Safety/Failsafes](https://docs.px4.io/main/en/config/safety)
- **Flight tasks:**
  - Flight tasks are classes derived from `FlightTask` with `activate()` (smooth takeover from the previous setpoints) and `update()` (per-loop setpoint generation). They are "used within flight modes" (for example `MPC_POS_MODE` selects the task in Position mode).
  - New tasks are compiled into the `flight_mode_manager`, so they are compile-time, not runtime plugins.
  - Architecture videos reference v1.9 and v1.11.
  - Source: [PX4 Flight Tasks](https://docs.px4.io/main/en/concept/flight_tasks)
- **Interface versioning** [PX4 v1.16+, experimental]:
  - A ROS 2 message translation node uses versioned uORB/ROS messages (a `MESSAGE_VERSION` field; `msg/versioned/`).
  - It translates between versions so that mismatched ROS 2 apps and PX4 interoperate "without code changes on either side". Direct (1:1) and generic (n:m) translations are bidirectional.
  - Sources: [PX4 msg translation node (main)](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node), [v1.16 page](https://docs.px4.io/v1.16/en/ros2/px4_ros2_msg_translation_node.html)
- **MAVSDK** [C++ guide, main/v4 docs] — [MAVSDK Missions guide](https://mavsdk.mavlink.io/main/en/cpp/guide/missions.html):
  - Plugins are constructed per `System` (`Mission{system}`).
  - The Mission plugin "only supports a small subset of the full functionality of MAVLink missions" (waypoints, speed change, loiter time, gimbal control, camera actions) and "does not provide takeoff, land or 'return to launch' MissionItems". Those come from the Action plugin.
  - MissionRaw exposes the full MAVLink spec.

### Inferences
- PX4 already implements the proposed separation:
  - Mission sequencing = mode executor (a state machine).
  - Moves = settled setpoint types / internal modes.
  - Reflexes = commander failsafe state machine that preempts any external mode and treats an unresponsive companion as a failsafe trigger.
- The proposal should therefore **not re-implement autopilot failsafes** in the Reflex layer. It should:
  - (a) defer to them;
  - (b) add only companion-level reflexes the autopilot cannot know about (supervisor hold on vision loss, swarm-separation holds);
  - (c) expect to be preempted and design skills to resume or re-plan when "reactivated".
- PX4's "defer non-essential failsafes" is a deliberate, scoped escape hatch for payload operations (winch). A delivery/acquire skill in NavPy would need a comparable, explicit, auditable mechanism rather than ad hoc suppression.
- PX4 executor rules ("one executor per mode", "executors cannot activate executors", "user switch always wins") are a useful, concrete ownership contract for the Mission -> Skill boundary.

### Gaps
- No timeout value or heartbeat mechanism for external-mode liveness was found in the fetched portion. The doc says only "crashes or becomes unresponsive".
- No full diagram of PX4's internal failsafe state machine was retrieved.
- The MAVSDK Action plugin method list (goto_location, hold, etc.) was not confirmed from a fetched page.

## 4. ArduPilot: modes, GUIDED/companion control, Lua scripting, failsafes and fence priority

### Takeaway
ArduPilot exposes companion "Moves" through GUIDED mode (position/velocity/attitude targets), with a built-in command-stream timeout that stops or levels the vehicle. Lua scripts run sandboxed and can set modes and targets, but have no scheduling guarantees and are terminated as a group on any memory panic. Failsafes switch mode and stay there until the pilot changes mode. The docs fetched give no explicit global priority order among failsafes and fence.

### Cited Findings
- **GUIDED mode** [Copter docs] — [Copter Guided mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html):
  - "Guided Mode can also be used by LUA scripts and companion computers to command vehicle movement and navigation."
  - If no command arrives within `GUID_TIMEOUT` (default 3 s), "the vehicle will slow to a stop" for velocity/acceleration commands, or holds a level hover for attitude commands.
  - `GUID_OPTIONS` bits 4/5 disable XY position/velocity error correction "for external controllers that already correct for error". Bit 6 enables S-curve planning and object avoidance.
  - `Guided_NoGPS` accepts only attitude targets.
- **Lua scripting** [Lua 5.3.5; auto heap growth in 4.7] — [Copter Lua Scripts](https://ardupilot.org/copter/docs/common-lua-scripts.html):
  - Bindings include `set_mode`, `start_takeoff`, `set_target_location`, `set_target_velocity_NED`.
  - "Each script is run in its own 'sandboxed' environment" with a fixed VM-instruction time chunk. Scripts "are not guaranteed to be run on a reliable schedule".
  - "If scripts run out of memory (or panic for any reason) all currently running scripts are terminated."
  - Pre-arm checksums (`SCR_LD_CHECKSUM`, `SCR_RUN_CHECKSUM`) can block arming if scripts are missing or changed.
- **Failsafes** — [Copter failsafes](https://ardupilot.org/copter/docs/failsafe-landing-page.html):
  - Copter failsafes: radio, battery, GCS, EKF, dead reckoning, vibration, terrain data loss, crash check, parachute, independent watchdog.
  - After a radio/battery/GCS/terrain failsafe changes mode, the vehicle "remain[s] in that mode until the pilot changes the mode directly".
- **GCS failsafe** [Copter 4.0+] — [GCS Failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html):
  - Triggers after `FS_GCS_TIMEOUT` (default 5 s).
  - Actions: RTL, SmartRTL, Land, DO_LAND_START or Brake.
  - `FS_OPTIONS` bits allow continuing Auto (bit 1), continuing landing (bit 3), continuing pilot modes (bit 4) and gripper release (bit 5).
  - On GCS reconnect the copter stays in the failsafe mode.
- **Geofence** — [Copter Simple Geofence](https://ardupilot.org/copter/docs/common-ac2_simple_geofence.html):
  - After a breach, backup fences are "created 20m out from the previous breached fence", so a pilot override may face a second action soon after.
  - With the fence enabled, an EKF failsafe and GPS loss in flight disables the fence.

### Inferences
- ArduPilot's companion contract is a *stream of Move targets with a watchdog*. A NavPy autopilot "Moves" plugin should treat the autopilot's own timeout (GUID_TIMEOUT) as an outer reflex and keep its command cadence well inside it.
- Because ArduPilot failsafes latch (no automatic return to the prior mode), the Mission/Skill layer must observe a mode change it did not command and treat it as preemption. This parallels PX4's executor deactivation, but without automatic reactivation.
- Lua is suitable for small on-autopilot reflexes (it survives companion loss). It is not suitable for timing-critical approach laws, given the documented lack of schedule guarantees and all-scripts termination on panic.

### Gaps
- No fetched source stated a global priority order between fence breach and other failsafes, or whether a fence breach overrides companion-commanded GUIDED. The FENCE_ACTION values and FENCE_OPTIONS were not retrieved.
- I could not confirm whether ArduPilot 4.6/4.7 lets Lua scripts register new named flight modes ("scripted modes"). Searches returned nothing authoritative.

## 5. Aerostack2: behaviors as ROS 2 actions, plugins, mission interpreters, platform abstraction

### Takeaway
Aerostack2 is the closest published aerial analogue to the proposal: layered (platform -> robotic functions -> behaviors -> plan execution -> mission control), with "behaviors" (= skills/moves) exposed as ROS 2-action-compatible servers. Behaviors support start/pause/resume/stop and also *modify* (change parameters without stopping). Motion controllers and state estimators are run-time-switchable plugins, and an `AerialPlatform` abstraction makes logic agnostic to real or simulated vehicles.

### Cited Findings
- **Paper and docs:** Fernandez-Cortizas, Molina, Arias-Perez, Perez-Segui, Perez-Saura, Campoy, "Aerostack2: A Software Framework for Developing Multi-robot Aerial Systems". arXiv v1 31 Mar 2023, v2 3 Sep 2024, submitted to IEEE RA-L. It highlights "platform independence, a modular plugin architecture, and behavior-based mission control" — [arXiv 2303.18237](https://arxiv.org/abs/2303.18237)
- **Layers:** sensor-actuator interface, basic robotic functions, behaviors, plan execution control, mission control. "Components in one layer control the components of the layers below." — [Aerostack2 paper (HTML v2)](https://arxiv.org/html/2303.18237v2)
- **Behaviors:**
  - Each encapsulates a robot skill (takeoff, land, hover, path following) with a behavior monitor supervising a behavior executor.
  - Behaviors expose start, pause, resume and stop. "A service called `modify` is used to change the parameters of a behavior without the need to stop its execution."
  - Behaviors are "fully consistent with standard ROS 2 actions".
  - Source: [Aerostack2 paper (HTML v2)](https://arxiv.org/html/2303.18237v2)
- **Plugins:** motion control and state estimation are plugins with common interfaces under a function manager, and "can be switched at run-time". The motion controller negotiates control modes among plugin, platform and motion reference — [Aerostack2 paper (HTML v2)](https://arxiv.org/html/2303.18237v2)
- **Mission specification:** three options — a Python API (`DroneInterface`), a JSON Mission Interpreter that translates to Python API calls with conditionals, and Behavior Trees (BehaviorTree.CPP calling behaviors "through their ROS 2 action interface") — [Aerostack2 paper (HTML v2)](https://arxiv.org/html/2303.18237v2)
- **Platform abstraction:**
  - An `AerialPlatform` abstract class; "the logic modules remain agnostic to whether the system is operating on a real platform or in simulation".
  - Moving from simulation to real flight needed only changing "the platform and the state estimation component".
  - Supported platforms: Gazebo, multirotor simulator, Crazyflie, Tello, Pixhawk 4, MAVLink, DJI Matrice OSDK/PSDK, Betaflight.
  - The docs are titled "Aerostack2 1.0"; binaries are for ROS 2 Humble.
  - Sources: [Aerostack2 paper](https://arxiv.org/html/2303.18237v2), [Aerostack2 docs](https://aerostack2.github.io/_00_getting_started/index.html)
- **Multi-robot evidence:**
  - Real flights: heterogeneous Crazyflie and Tello gate crossing (mocap), and two DJI M300/M350 cooperative inspection of 4,500 m² with Jetson AGX Xavier.
  - The authors note "our initial experiments involved only two drones simultaneously"; scaling is future work.
  - Source: [Aerostack2 paper](https://arxiv.org/html/2303.18237v2)

### Inferences
- The proposal's Skills/Moves split is finer than Aerostack2's. Aerostack2 "behaviors" mix mission-meaningful skills (follow path) and motion primitives (takeoff, hover) at one level. Keeping Moves as a separate autopilot-plugin layer (as proposed) is closer to PX4's setpoint types and makes autopilot swap cleaner.
- The `modify` verb (re-parameterize without stopping) is worth copying for skills like observe/approach, where the POI or standoff changes in flight.
- Aerostack2's run-time-switchable plugins differ from the proposal's "missions choose plugins at setup". Setup-time binding is simpler to verify; run-time switching adds a negotiation step (control-mode negotiation) that Aerostack2 needed.
- Swarm scale beyond 2 vehicles is not demonstrated in the paper. Aerostack2 is evidence for layering, not for swarm coordination.

### Gaps
- I did not find the current Aerostack2 release number or ROS 2 distro support beyond "1.0 / Humble" on the getting-started page. Newer releases (Jazzy) may exist but were not confirmed.
- RA-L acceptance of the paper was not confirmed.

## 6. MOOS-IvP: objective-function behaviors, IvP helm arbitration, mode-based activation, field use

### Takeaway
MOOS-IvP takes the opposite arbitration approach to priority override. Every active behavior emits an IvP objective function over the decision space (heading, speed, depth), weighted by priority, and the helm solves a multi-objective optimization each cycle. Behaviors are switched in and out by hierarchical mode declarations driven by flags. It has long field use as "backseat driver" autonomy on AUVs and USVs.

### Cited Findings
- The IvP Helm is the MOOS app `pHelmIvP`. "IvP is short for interval programming - a technique for representing and solving multi-objective optimizations problems." Behaviors "are reconciled using multi-objective optimization when in competition with each other for influence of the vehicle" [helm guide release 13.2, Feb 2013] — [MOOS-IvP Helm 13.2 guide](https://oceanai.mit.edu/moos-ivp-pdf/moosivp-helm-13.2.pdf)
- "An IvP function is a piecewise linear approximation of an objective function, over a discrete decision space". The typical decision space is heading, speed and depth. "An IvP problem consists of a set of k functions, each with a priority weighting", solved by branch-and-bound — [Benjamin 2017 Vienna slides](https://ilp.mit.edu/sites/default/files/2020-01/Benjamin.2017.Vienna.pdf)
- The helm iteration runs: read mail, evaluate mode declarations, behavior participation, behavior reconciliation, publish results — [Helm 13.2 guide](https://oceanai.mit.edu/moos-ivp-pdf/moosivp-helm-13.2.pdf). Behavior states, flags and conditions are separate course topics — [MOOS-IvP lecture 03](https://oceanai.mit.edu/mc/talk_pdfs/lecture03.pdf)
- Hierarchical mode declarations [release 22.8 test case]:
  - Modes (INACTIVE, ACTIVE, SURVEYING, STATION-KEEPING, RETURNING) are set by brace conditions on flags and the parent mode.
  - Sources: [plug_hsd.bhv, 22.8](https://oceanai.mit.edu/svn/moos-ivp-aro/releases/moos-ivp-22.8/ivp/src/app_nsplug/testcases/plug_hsd.bhv), [HelmEngine.cpp 22.8](https://oceanai.mit.edu/svn/moos-ivp-aro/releases/moos-ivp-22.8/ivp/src/pHelmIvP/HelmEngine.cpp)
- Concurrent behaviors are grouped into a hierarchy of modes that can be turned on and off. For example, transit runs GoToWaypoint + AvoidCollision concurrently, and on arrival switches to a mode with only StationKeep [2024] — [Aquaticus maritime CTF paper](https://arxiv.org/pdf/2404.17038)
- Field use:
  - Bluefin-9 backseat-driver demo in Boston Harbor (2011).
  - Bluefin and MIT plug-n-play payload autonomy on Bluefin-9 and -21, where missions were "configure[d] and simulate[d] … in the lab, then upload[ed]" (2013).
  - Sources: [GD Mission Systems 2011](https://gdmissionsystems.com/articles/2011/07/15/moos-ivp-the-open-source-backseat-driver-software-successfully-demonstrated-on-bluefin-9), [GD Mission Systems 2013](https://gdmissionsystems.com/articles/2013/07/23/bluefin-and-mit-demonstrate-auv-plug-n-play-payload-autonomy)

### Inferences
- MOOS-IvP is a credible alternative to "one approach law outputs the command, a reflex overrides it". When several concerns must be *blended* rather than *preempted* (approach-to-POI + peer separation + no-fly margin in a swarm), objective-function arbitration avoids brittle priority chains.
- A hybrid fits the proposal well: hard safety (failsafe, geofence) as Simplex-style override, and soft concerns (separation, smoothing) as weighted objectives. This complicates the "approach-law plugins only map inputs to commands" contract, since plugins would emit preferences instead of commands.
- "Mode-based behavior activation" is equivalent to the proposal's Mission activating a set of Skills. MOOS-IvP shows that several behaviors can be concurrently active within a mode (for example transit + avoid), which a strict one-skill-at-a-time Mission would not allow.
- The backseat-driver pattern (vehicle OEM autopilot in front, MOOS-IvP autonomy behind on a separate computer) mirrors NavPy's companion-computer-over-autopilot split.

### Gaps
- Current MOOS-IvP release number for 2026 not confirmed (latest seen: 22.8 source tree; Aquaticus 2024 usage).
- The 13.2 guide PDF exceeded the fetch size limit; exact definitions of behavior states (idle/running/active/completed), run conditions and endflags were not quoted from a primary source.

## 7. Skill-based architectures: how a "skill" is defined

### Takeaway
In skill frameworks a skill is a parameterized, reusable capability with explicit **pre-conditions** (when it may start), **hold-conditions** (what must stay true while running) and **post-conditions/effects** (what it guarantees on success). It is composed by a BT or HSM and grounded in a world model. Aerostack2 adds a lifecycle verb set (start/pause/resume/modify/stop) consistent with ROS 2 actions.

### Cited Findings
- SkiROS2 [IROS 2023]:
  - "a skill-based robot control platform on top of ROS". "The skill formulation based on pre-, hold- and post-conditions allows to organize robot programs" from perception to low-level control.
  - Scheduling "builds on the extended behavior tree model that merges task-level planning and execution", with "a knowledge base for reasoning about the world state and entities".
  - Source: [Mayr, Rovida, Krueger, arXiv 2306.17030](https://arxiv.org/abs/2306.17030)
- Aerostack2 "behavior" = robot skill with a monitor/executor and start/pause/resume/stop/modify services, consistent with ROS 2 actions — [Aerostack2 paper](https://arxiv.org/html/2303.18237v2)
- PX4 mode requirements play the role of skill pre-conditions enforced by the platform: an unmet requirement blocks arming while the mode is selected, or triggers a failsafe in flight — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)
- RAFCON library states are reusable hierarchical sub-state-machines, which is the HSM equivalent of a skill library — [humanoid paper 2026](https://arxiv.org/pdf/2606.26425)

### Inferences
- The proposal's skills (search, confirm, observe, acquire, approach) should each declare:
  - **pre:** for example "POI selected", "dock class detected".
  - **hold:** for example "POI track valid within N frames", "fleet slot owned".
  - **post/effect:** for example "POI confirmed", "acquired".
  - **outcomes:** success, failure-with-reason, preempted.
- Hold-condition violations are the natural trigger for the skill's own fallback, distinct from reflex-level override.
- Effects should be stated as observable facts, not physical guarantees. For example "approach completed" is not "docked" (consistent with the project rule that approach accuracy does not establish docking).

### Gaps
- I did not retrieve formal definitions from industrial skill models (for example the Bøgh et al. skill model, PLCopen/OPC UA skills, or the CAPEC/CAMeL skill standards). Only SkiROS2's abstract-level definition was confirmed.
- SkiROS2 details (RDF/OWL world model, PDDL planner) were not confirmed from the fetched abstract.

## 8. Runtime assurance / Simplex: enforcing safety overrides over complex controllers

### Takeaway
Simplex and ASTM F3269 formalize the "Reflex" layer: an unverified complex function runs normally, and a simple, verifiable monitor plus switch hands control to an assured recovery/safety controller when a safety property is about to be violated. The monitor and recovery must themselves be trusted. Repeated recovery activations are treated as evidence of a defective complex function.

### Cited Findings
- Simplex components:
  - A complex controller ("advanced functionalities … potentially unverifiable due to its complexity").
  - A safety controller ("limited performance but is robust thanks to its simplicity. It can be exhaustively tested and verified to be safe").
  - A decision module that checks a safety envelope and switches.
  - Sources: [Container-based resilient control for UAVs, arXiv 1812.02834](https://arxiv.org/pdf/1812.02834); originating from Sha's 2001 work as described in [UIUC dissertation](https://www.ideals.illinois.edu/items/11503)
- Component-based Simplex: a pre-certified decision module "continually monitors the state of the plant and switches control of the plant to the BC should the plant be in imminent danger". It is increasingly important as controllers become "more adaptive with the use of unverified algorithms such as machine-learning's" — [arXiv 1704.04759](https://arxiv.org/pdf/1704.04759); a 2025 application to deep-learning autonomy is [arXiv 2509.21014](https://arxiv.org/pdf/2509.21014)
- ASTM F3269:
  - F3269-17 was scoped to "UAS containing complex function(s)" and is now historical. F3269-21 broadens to aircraft systems and is active.
  - Building blocks: input manager, safety monitor(s), recovery control function, switch, around the unassured complex function.
  - The premise is that complex functions are "too challenging to use conventional software assurance methods such as RTCA DO-178C".
  - "repeated invocation of an RCF during a single mission may be considered an indication of improper Complex Function performance".
  - Sources: [ASTM F3269-21](https://store.astm.org/f3269-21.html), [ASTM F3269-17](https://store.astm.org/f3269-17.html) (via search snippets; full text paywalled)
- NASA TM (2022) on RTA guidance: any RTA scheme "must itself be trusted before it can be deployed into use", and its recommendations largely cover the monitoring function — [NASA TM RTA guidance](https://ntrs.nasa.gov/api/citations/20220015734/downloads/tm-rta-guidance.pdf)
- An RTA based on F3269-17, with diverse run-time monitors, maintained safety "in the presence of defects" in a neural-network taxiing controller [NFM 2020] — [Springer DOI 10.1007/978-3-030-55754-6_21](https://dx.doi.org/10.1007/978-3-030-55754-6_21)
- Monitors can watch the learned component's inputs, internal activations, outputs, and system state such as geofence or flight envelope [Collins Aerospace presentation] — [Run-Time Assurance Architecture for Learning-Enabled Systems](https://sos-vo.org/node/69921)

### Inferences
- In F3269 terms:
  - Approach law and perception = complex function.
  - "Supervisor hold" and the autopilot's hold/RTL = recovery control function.
  - Reflex layer = monitor + switch.
  - The input manager (validity checks on sensor inputs) is a role the proposal lacks explicitly. Perception plugin output validation (stale frame, implausible LOS rate) belongs there.
- The proposal should require that the reflex/recovery path be simpler than, and independent of, the skill and approach-law code. It should run on a path that survives skill or perception crashes (cf. PX4 treating an unresponsive external mode as a failsafe, and ArduPilot GUID_TIMEOUT).
- Logging recovery activations per mission, and treating repeats as a defect signal (F3269-17), maps directly to evaluation/certification diagnostics.

### Gaps
- Sha's original 2001 paper ("Using simplicity to control complexity", IEEE Software) was not retrieved directly; it is described via secondary sources.
- The ASTM F3269 full text (timing/latency appendix, switching criteria) is paywalled and was not read.

## 9. Plugin/port patterns: declaring interfaces, loading, versioning, conformance

### Takeaway
Stacks enforce plugin contracts mostly by:
- (a) typed base classes or ports, checked at load time;
- (b) managed lifecycles with liveness monitoring;
- (c) platform-side requirement and arming checks.

Explicit interface versioning is rare and recent; PX4's v1.16 message versioning + translation node is the main example. Formal per-plugin conformance suites were not found in any stack reviewed; conformance is via CI, simulation and pre-arm checks.

### Cited Findings
- **Declaration:**
  - Nav2: C++ base interfaces (for example `nav2_core::BehaviorTreeNavigator`) with configure/activate/deactivate hooks — [NavigatorBase (Jazzy)](https://api.nav2.org/nav2-jazzy/html/classnav2__core_1_1NavigatorBase.html)
  - BehaviorTree.CPP: static `providedPorts()` with type-checked connections — [BT.CPP ports](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_ports)
  - PX4: `ModeBase` subclass with declared requirements and setpoint types — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)
  - PX4 flight tasks: `FlightTask` subclass with activate/update — [PX4 Flight Tasks](https://docs.px4.io/main/en/concept/flight_tasks)
  - Aerostack2: common plugin interfaces per function type — [Aerostack2 paper](https://arxiv.org/html/2303.18237v2)
- **Loading and selection:**
  - Nav2 selects plugins by name in parameters, and BT nodes pick among loaded plugins at runtime — [Navigation Concepts](https://docs.nav2.org/rolling/getting_started/navigation_concepts/)
  - Nav2 custom BT node libraries are listed in `plugin_lib_names` — [BT Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)
  - PX4 external modes register at runtime over DDS. Flight tasks are compiled in — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface), [Flight Tasks](https://docs.px4.io/main/en/concept/flight_tasks)
  - ArduPilot Lua loads from ROMFS or SD `APM/scripts` at boot — [Lua Scripts](https://ardupilot.org/copter/docs/common-lua-scripts.html)
  - Aerostack2 plugins are run-time switchable — [Aerostack2 paper](https://arxiv.org/html/2303.18237v2)
- **Versioning:**
  - PX4 v1.16+ uses versioned messages (`MESSAGE_VERSION`) with bidirectional translations so that old and new clients interoperate — [PX4 translation node](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node)
  - The PX4 ROS 2 interface library recommends using the latest main of PX4, px4_msgs and the library together, and prints "Checking message compatibility..." at startup — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)
  - Nav2 API signatures differ across distros (for example `nav2_util::LifecycleNode` in Jazzy vs `nav2::LifecycleNode` in Rolling) — [Nav2 API Jazzy](https://api.nav2.org/nav2-jazzy/html/classnav2__bt__navigator_1_1BtNavigator.html), [Nav2 API Lyrical](https://api.nav2.org/nav2-lyrical/html/classnav2__bt__navigator_1_1BtNavigator.html)
- **Liveness and integrity:**
  - Nav2 bond heartbeats, where one dead server brings the system down — [Lifecycle Manager](https://docs.nav2.org/configuration/packages/configuring-lifecycle.html)
  - PX4: a crashed or unresponsive external mode triggers a failsafe — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)
  - ArduPilot script checksums gate arming — [Lua Scripts](https://ardupilot.org/copter/docs/common-lua-scripts.html)
- **Conformance and testing:**
  - PX4 mode/executor interfaces are "tested in CI", and a failsafe state machine simulation page exists for combined triggers — [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface), [PX4 Safety](https://docs.px4.io/main/en/config/safety)
  - PX4 says new flight tasks should be tested "only in simulation" — [Flight Tasks](https://docs.px4.io/main/en/concept/flight_tasks)
  - Aerostack2 validates sim-to-real by swapping only the platform and state-estimation plugins — [Aerostack2 paper](https://arxiv.org/html/2303.18237v2)

### Inferences
- For NavPy, the evidence supports:
  - (1) a narrow typed port per plugin family (perception: frame -> LOS/pixels + validity; approach law: declared inputs -> command; autopilot Moves: goto/hold/orbit/climb -> ack/progress);
  - (2) load-time validation of declared inputs against an allow-list, enforcing constraints such as the pure-vision rule mechanically;
  - (3) liveness supervision of each plugin, feeding the Reflex layer;
  - (4) a version field on each port contract (PX4 style) once plugins ship separately;
  - (5) a shared conformance test kit per port (same scenarios run against every implementation in SITL). No surveyed stack ships such a kit, so this would be an addition rather than an established practice.
- Choosing plugins at mission setup (static binding) is easier to certify than Aerostack2's run-time switching, and matches Nav2's parameter-selected plugins. Nav2 still allows the BT to choose among *preloaded* plugins at runtime, a middle ground worth considering (for example preload two approach laws and let the skill pick by ID).

### Gaps
- No stack reviewed publishes a formal plugin conformance test suite or semantic-versioning policy for plugin interfaces. I found no reliable source on this.
- The pluginlib description-XML and export mechanics were not confirmed from a fetched page (see section 1 gaps).

## 10. Synthesis: how the proposed Mission -> Skills -> Moves -> Reflexes design compares

### Takeaway
The proposed four-layer split matches established practice. Mission maps to Nav2's BT navigator, PX4's mode executor, Aerostack2's mission control and MOOS-IvP's mode declarations. Skills map to BT action nodes/servers, Aerostack2 behaviors and SkiROS2 skills. Moves map to PX4 setpoint types/modes and ArduPilot GUIDED targets. Reflexes map to the Nav2 Collision Monitor, PX4/ArduPilot failsafes and Simplex/F3269 RTA.

The main risks:
- (a) duplicating autopilot failsafes instead of layering above them;
- (b) leaving preemption and resume semantics implicit;
- (c) a strict single-command approach-law contract that cannot blend concurrent concerns (swarm separation) the way MOOS-IvP does.

### Cited Findings
- Reflex-as-filter below the command producer, with most-aggressive-wins, is the Nav2 pattern — [Nav2 Collision Monitor tutorial](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)
- In PX4 the more severe failsafe wins and failsafes preempt external modes and executors, with executor reactivation after clearance — [PX4 Safety](https://docs.px4.io/main/en/config/safety), [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)
- ArduPilot failsafe mode changes latch until the pilot changes mode, and GUIDED stops after `GUID_TIMEOUT` without commands — [ArduPilot failsafes](https://ardupilot.org/copter/docs/failsafe-landing-page.html), [Guided mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html)
- Behaviors with start/pause/resume/modify/stop — [Aerostack2](https://arxiv.org/html/2303.18237v2); skills with pre/hold/post-conditions — [SkiROS2](https://arxiv.org/abs/2306.17030)
- Objective-function arbitration among concurrently active behaviors — [MOOS-IvP helm](https://oceanai.mit.edu/moos-ivp-pdf/moosivp-helm-13.2.pdf)
- RTA: monitor + switch + assured recovery, with an input manager — [ASTM F3269-21](https://store.astm.org/f3269-21.html)

### Inferences
- **Keep:**
  - The four-layer split.
  - Autopilot-plugin Moves, since a narrow Move vocabulary eases PX4/ArduPilot swap, mirroring PX4 setpoint types and ArduPilot GUIDED.
  - Approach laws as pure input -> command maps, analogous to Nav2 controller plugins and Simplex's "complex controller" that can be swapped without touching safety.
  - Setup-time plugin selection.
- **Add or sharpen:**
  - (1) A skill contract with pre/hold/post-conditions and uniform outcomes, plus a `modify` verb.
  - (2) Explicit preemption semantics: skills must handle "autopilot/reflex took over" (PX4 deactivate/reactivate; ArduPilot latching).
  - (3) An F3269-style input manager validating perception output before the approach law.
  - (4) Reflexes split into autopilot-owned (do not duplicate) and companion-owned (supervisor hold, vision-loss hold, separation), with a defined "most severe wins" order.
  - (5) A scoped, logged failsafe-deferral mechanism for payload phases, like PX4 `deferFailsafes`.
  - (6) Liveness monitoring of the companion command stream on both sides.
  - (7) Port versioning plus a per-port SITL conformance kit.
- **Open design choice:** if multiple Skills or concerns must be active concurrently (approach + peer separation), choose between (i) a Simplex-style override chain and (ii) MOOS-IvP-style weighted objective arbitration for soft concerns. The surveyed evidence supports override for hard safety and is mixed for soft blending.
- **Mission formalism:** an FSM/HSM is sufficient for linear module flows. A BT is preferable if skills need continuous re-checking and fallback. Empirically both are used with similar structure (Ghzouli 2023), so the outcome contract matters more than the formalism.

### Gaps
- I found no published UAV-swarm stack that combines all four layers with per-vehicle reflexes *and* swarm-level coordination arbitration. Aerostack2's multi-robot evidence covers 2 vehicles, and MOOS-IvP's multi-vehicle field use is marine.
- No source quantified the runtime overhead or latency of BT vs FSM mission executors on companion hardware.
