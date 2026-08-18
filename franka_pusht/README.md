# Franka PushT

ROS 2 application package for real FR3 PushT demonstrations and policy deployment.

See `NEED_TO_CONFIGURE.md` for the original calibration checklist. This README
documents the actual working procedure for running the real robot, established
and validated through hands-on testing.

## Running the real robot: full startup procedure

Four things need to be running, brought up **in this order**, each in its own
terminal. `robot_ip` and Motive/OptiTrack addresses below are specific to this
lab's network.

### 1. OptiTrack driver

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```

This only configures the node; it does not auto-activate. In a second
terminal (or after it prints "Configured!"):

```bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

Verify: `ros2 topic hz /rigid_bodies` should show ~120 Hz.

### 2. Franka + MoveIt + Servo

**Prerequisite**: on the Franka Desk web UI, the robot must be in **Execution
mode with FCI activated** (not Programming mode) — otherwise `ros2_control_node`
hangs forever waiting on `robot_description` and never actually connects to the
real hardware, silently falling back to a frozen default joint state.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=10.90.90.177 use_fake_hardware:=false load_gripper:=false \
  ee_id:=custom_pusher_ee
```

Verify controllers came up active:

```bash
ros2 control list_controllers
# fr3_arm_controller, joint_state_broadcaster, franka_robot_state_broadcaster
# must all read "active"
```

### 3. Switch Servo to twist mode

This is **per-launch state** — it does not persist across `moveit.launch.py`
restarts and must be redone every time:

```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
```

### 4. The `franka_pusht` pipeline

```bash
source install/setup.bash
ros2 launch franka_pusht pusht_collect.launch.py
```

Brings up `optitrack_bridge_node`, `observation_node`, `joy_node`,
`ps5_teleop_node`, `safety_node`, `data_collection_node`. Verify:

```bash
ros2 topic echo /pusht/observation_valid   # should be `true`
```

## Controller mapping (DualSense)

- **Deadman**: L1 (button index 4) — must be held for any motion.
- **Speed**: R2 trigger — partial squeeze scales speed, released = zero.
- **Direction**: left stick — up/down and left/right map to the two
  normalized action axes.
- **Home**: PS button (button index 10) — pauses Servo, drives to the fixed
  `HOME_JOINTS` via MoveGroup, resumes Servo. Fails silently to a log line
  (`"Home failed"`) if anything in that chain breaks (see Troubleshooting).
- **Start episode**: button 0 = **Cross (✕)** — homes first, then calls
  `/pusht/start_episode` only if homing succeeded, so every episode starts
  from the same fixed pose.
- **Stop episode**: button 1 = **Circle (○)** — calls `/pusht/finish_episode`,
  then homes again. (Both button indices verified empirically via `ros2 topic
  echo /joy`; the raw reading flickers on/off rapidly while held, which
  hasn't caused problems so far but is worth knowing about.)

## Collecting data

1. Bring up the full stack (steps 1-4 above) and confirm
   `/pusht/observation_valid` is `true`.
2. Place the T-block at its episode starting position on the table (a
   consistent starting condition across episodes, matching how the sim
   environment resets).
3. Press **Cross (✕)** (start episode). It homes the arm first, then calls
   `/pusht/start_episode` only if homing succeeded — wait for homing to
   finish before teleoperating.
4. Teleoperate: hold the deadman, squeeze R2, push the T-block toward the
   goal with the left stick.
5. Press **Circle (○)** (stop episode) — saves the episode, then homes again,
   ready for the next one.
6. Repeat from step 2 for each new episode.

Episodes land in `~/pusht_data/session_<timestamp>/episode_0000.npz`,
`episode_0001.npz`, etc., each with `states`, `actions`, `next_states`,
`absorbing`, `rewards` (zero-filled — unused by IRL training, since the
reward is learned, not supplied by the expert). Note: `data_collection_node`
creates a new `session_<timestamp>/` directory every time it starts, whether
or not any episodes get recorded in it — a pile of near-empty session
folders (just a `metadata.json`, no `episode_*.npz`) is expected from
restarts, not a bug.

To discard an in-progress episode without saving it:
`ros2 service call /pusht/abort_episode std_srvs/srv/Trigger "{}"` (not bound
to a controller button).

`goal_to_robot_quaternion_wxyz` being unvalidated only affects how intuitive
the joystick feels, not the correctness of recorded data — as long as you can
successfully push the block to the goal, the recorded transitions are valid
regardless of whether the mapping feels natural.

## Troubleshooting

These are real issues hit (and fixed) while getting this running, in
likely order of relevance:

- **Home button silently fails ("Home failed" in the log, arm never moves)**:
  Check `ros2 control list_controllers` — if `fr3_arm_controller` rejects
  goals (`move_group`'s log shows `"Goal was rejected by server"`), and
  `/fr3_arm_controller/joint_trajectory` shows no publisher via
  `ros2 topic hz`/`ros2 topic echo --once`, Servo and/or the controller stack
  is in a stuck state. **Fix: fully restart `moveit.launch.py`** (Ctrl+C, then
  relaunch) — this has reliably cleared it every time; partial fixes (manually
  toggling controller lifecycle states via `ros2 control set_controller_state`)
  have made it *worse* (cascaded other controllers to `inactive`/`unconfigured`).
  After restarting: redo step 3 (switch Servo to twist mode) and verify step 4
  is still up (it usually survives, but the OptiTrack driver from step 1
  sometimes does not — check `ros2 node list | grep mocap`).
- **Teleop does nothing, but nothing looks wrong**: check for a stray
  `RTPS_TRANSPORT_SHM Error: Failed init_port ...` on any fresh `ros2` CLI
  call — a sign of a stale/orphaned FastDDS shared-memory lock in `/dev/shm`
  (look for a `fastrtps_port<N>_el` file with no matching `fastrtps_port<N>`
  data segment or `sem.fastrtps_port<N>_mutex` — that mismatched triplet is
  the signature of a leftover from a forcefully-killed process, safe to `rm`).
  A full stack restart also clears this, since it recreates all segments.
  Note that ONLY `ros2 control list_controllers` and other `list`/`info`
  commands query the daemon's cached graph and can look fine even when actual
  message delivery to a fresh listener is broken — use `ros2 topic echo
  <topic> --once` to check for genuine data flow, not just topology.
  If teleop still won't move the arm, also confirm the deadman is really
  registering (`ros2 topic echo /pusht/deadman`) and that Servo hasn't halted
  on a joint-limit warning (`ros2 topic echo /servo_node/status` — code 6,
  "Close to a joint bound").
- **This build of `moveit_servo` has no `start_servo`/`stop_servo` Trigger
  services** (only `pause_servo`, a `std_srvs/SetBool`: `true` = pause,
  `false` = resume). `ps5_teleop_node.py` already uses `pause_servo` — if you
  see references to `start_servo`/`stop_servo` elsewhere (e.g. the separate,
  unrelated `ps5/joy_to_twist.py` script), those calls silently no-op against
  this Servo build.
- **A home target can be kinematically valid but still leave a joint sitting
  right at its safety-halt margin.** Planning to a joint-space goal can
  succeed even when a joint ends up within `joint_limit_margin` (0.10 rad in
  `fr3_servo_config.yaml`) of its limit — Servo will then halt
  ("close to a joint bound") on almost any further motion from there. When
  picking a new home pose, verify margin against the real per-joint limits
  (`franka_description/robots/fr3/joint_limits.yaml`), not just planning
  success. `/compute_ik` (plan-only, no motion) is a safe way to explore
  candidate poses/seeds before committing to one.
- **`optitrack_bridge` repeatedly logs `"Never seen rigid bodies named:
  ['REPLACE_WITH_MOTIVE_EE_MARKER_RIGID_BODY_NAME']"`**: expected and
  harmless. `ee_rigid_body_name` is unset in `pusht_robot.yaml` since EE
  tracking no longer uses a marker (see Calibration status below) — the node
  still declares a default placeholder for that parameter and warns that it's
  never seen, but nothing depends on it.
- **Restarting only `franka_pusht` (killing/relaunching `pusht_collect.launch.py`)
  while `moveit.launch.py`/`servo_node` keep running** appears to sometimes leave
  Servo's subscription to `/servo_node/delta_twist_cmds` stuck (input flows at
  20 Hz from a fresh `safety_node`, but `/fr3_arm_controller/joint_trajectory`
  never produces output) — likely stale DDS discovery after `safety_node`'s
  publisher identity changes out from under Servo's still-running subscription.
  A full `moveit.launch.py` restart alongside `franka_pusht` avoids it; restart
  both together when possible rather than just the `franka_pusht` pipeline.

## Calibration status (2026-08-18)

- **T-block tracking**: live via OptiTrack (`objectPushT` rigid body).
  `block_marker_to_object_translation/quaternion_wxyz` = identity, validated
  by placing the block at goal and confirming `/pusht/observation[0:7]`
  matches within ~1.5 cm / 2°.
- **EE tracking**: the real pusher tool carries no OptiTrack marker. EE pose
  comes from forward kinematics (`fr3_link0 -> fr3_pusher_tcp` via `/tf`)
  composed with `franka_pole_base`'s live tracked pose and a fixed, calibrated
  `pole_base_to_robot_base_translation/quaternion_wxyz` offset (see
  `ee_from_tf` in `pusht_robot.yaml` and `observation_node.py`). This
  self-corrects if the robot's stand is ever bumped/repositioned, unlike a
  hardcoded world-to-base constant.
- **Fixed goal pose**: set from a real physical T-block placement.
- **Home position**: solved via `/compute_ik` for the final link exactly
  perpendicular to the floor, near table height, with ~60° of joint-limit
  margin on every joint.
- **NOT yet validated**: `goal_to_robot_quaternion_wxyz` is still the identity
  placeholder. A candidate was solved via least-squares regression (paired
  finite-difference `/pusht/observation` EE velocity against concurrent
  `/pusht/executed_action` over a varied joystick sweep, while the rotation
  was still identity) — it passed its own internal orthogonality check
  (rows ~perpendicular, as a real rotation requires) but was tried live and
  reverted: reported no left/right motion and reversed up/down. Identity
  itself is *also* known-wrong (the very first direction check showed "up"
  moving opposite to +fr3_link0-X, not what identity implies) — data
  collection is proceeding anyway since this only affects teleop
  intuitiveness, not recorded-data correctness (see Collecting data above).
  Revisit with a cleaner regression capture (longer sweep, wider direction
  coverage) when there's appetite to fix the feel of teleop.
- **NOT yet done**: real workspace bounds (`workspace_x/y/z`) — currently a
  generous provisional box for testing, not the actual measured-safe
  envelope (move the tool to each true physical edge and read
  `ee_pos_rel_goal`, per the original checklist).

## Build

`optitrack_bridge/` (the `mocap4r2` OptiTrack driver stack) lives directly
under `src/`, alongside this package. It is not itself a ROS package, so
colcon recurses into it and discovers the packages nested inside
(`mocap4r2_optitrack_driver`, `mocap4r2_msgs`, `mocap4r2_control`, etc.)
automatically. No separate build step is needed — a normal workspace build
from the workspace root builds everything, in dependency order:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

To rebuild just this package and the OptiTrack driver while iterating:

```bash
colcon build --symlink-install --packages-up-to franka_pusht mocap4r2_optitrack_driver
```

## Interfaces

- OptiTrack `PoseStamped`: T-block marker only (no EE marker; see Calibration
  status above); the goal may be fixed or tracked.
- `/joint_states`: named FR3 positions and velocities.
- `/pusht/observation`: 24 floats in PPO-FB order.
- `/pusht/teleop_action`, `/pusht/policy_action`: normalized XY actions.
- `/pusht/executed_action`: accepted normalized action.
- `/servo_node/delta_twist_cmds`: planar TCP velocity in `fr3_link0`.
- `/pusht/start_episode`, `/pusht/finish_episode`, `/pusht/abort_episode`.

## Local smoke test

Launch fake FR3 hardware separately with `ee_id:=custom_pusher_ee`, then run:

```bash
ros2 launch franka_pusht pusht_mock.launch.py
ros2 topic hz /pusht/observation
ros2 topic echo /pusht/observation --once
```

The mock launch supplies fixed FR3 home joint states and a mock OptiTrack EE
marker (`ee_from_tf: false` in `pusht_mock.yaml` — the mock harness has no
`/tf`, unlike the real robot path). A `/joy` stream is needed only for
nonzero actions. These application-level mocks work on Humble and Jazzy; the
actual Franka/MoveIt stack must be validated on Jazzy.

## Scaling contract

The safety node converts normalized action to metres/second using
`max_linear_speed` (currently `0.08`, raised from the original `0.05` default
after dead-man/timeout/workspace/controller behavior was confirmed good; see
`NEED_TO_CONFIGURE.md` step 10 for the original conservative-start rationale).
Its output is therefore physical velocity. Configure MoveIt Servo with
`command_in_type: speed_units`; do not apply a second scale.

Policy actions and workspace limits are expressed in the goal frame. Configure
`goal_to_robot_quaternion_wxyz` to rotate those commands into `fr3_link0`
(**not yet validated** — see Calibration status above).

Real collection starts the standard ROS `joy_node`, followed by
`franka_pusht/ps5_teleop_node`. This PushT-specific adapter publishes normalized
actions only; the safety node remains the sole Servo command publisher. The
existing `ps5` package is not modified or launched by this package.
