# Franka PushT

ROS 2 application package for real FR3 PushT demonstrations and policy deployment.

See `NEED_TO_CONFIGURE.md` for the original calibration checklist. This README
documents the actual working procedure for running the real robot, established
and validated through hands-on testing.

See `SIM_ALIGNMENT.md` to configure `pusht_mjx` to match this rig — required if
demonstrations recorded here are to be valid training data over there. It lists
every measured rig property, the exact sim edits, and the known remaining
mismatches.

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

`ps5_teleop_node` now does this automatically at startup (see step 4) — it
retries against `/servo_node/switch_command_type` in a background thread as
soon as it comes up. You only need to do this by hand if you're testing Servo
directly without the `franka_pusht` pipeline running, or if the teleop node's
log shows `"Failed to switch Servo to Cartesian twist command mode"`:

```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
```

This is **per-launch state** — it does not persist across `moveit.launch.py`
restarts, which is exactly why it's now automated rather than a manual step to
remember.

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

`goal_to_robot_quaternion_wxyz` is now validated (see Calibration status
below) — teleop direction should feel consistent and controllable.

## T-block episode start pose

Place the block here at the start of **every** episode, so the real task has a
fixed reset like the sim's (`pusht_mjx` resets to a fixed block pose too, and
the policy was trained only from that one start state). Measured from the block's physical placement on 2026-08-20, after all frame and
origin corrections -- these supersede any earlier figures. Re-record (see the
echo below) whenever the block's start placement changes.

Canonical block pose (i.e. after `block_marker_to_object_*`), expressed in
`fr3_link0`, which is also the sim's world frame:

| | value |
| --- | --- |
| position x, y | `0.6729`, `-0.1391` m |
| z (block mid-thickness) | `0.0463` m |
| yaw | `+88.28°` = `1.5408` rad (tilt off vertical `1.21°`) |

As the observation reports it — the practical way to check a placement, since
it needs no transforms:

```text
block_pos_rel_goal  = [ 0.1230, -0.1309,  0.0012]     # 18.0 cm from the goal
block_quat_rel_goal = [ 0.6903, -0.0039,  0.0087, 0.7235]   # 92.7 deg rotated
```

```bash
ros2 topic echo /pusht/observation --once   # indices 0:3 and 3:7
```

Raw mocap of the `objectPushT` marker (`/rigid_bodies`, Y-up `map` frame), for
reference — note this is the *marker* pose, not the canonical block pose:

```text
position    = [1.954989, 0.092635, 1.217220]
quat (wxyz) = [-0.698347, -0.003770, -0.715702, 0.008312]
```

### Matching this in simulation

This placement is already very close to the sim's own default start, so
`pusht_mjx` needs only a small nudge to line up:

| | real | `pusht_mjx` default |
| --- | --- | --- |
| block position | `(0.673, -0.139)` | `BLOCK_POS = (0.6, -0.1)` |
| block yaw | `88.28°` | `BLOCK_ANGLE = pi/2` (90°) |
| goal position | `(0.560, 0.001)` | goal body `pos="0.5 0.0 0.04"` |
| block → goal distance | 18.0 cm | 14.1 cm |
| block → goal rotation | 92.7° | 90° |

To reproduce the real setup exactly, in `environment.py`:

```python
BLOCK_POS = (0.673, -0.139)   # was (0.6, -0.1)
BLOCK_ANGLE = 1.5408          # rad = 88.28 deg; was pi/2
```

and move the `goal` body in `scene_mjx_free.xml` / `scene_mjx_joint.xml` to
`pos="0.560 0.001 0.044"` (and raise `ground` to z=0.0125). **`SIM_ALIGNMENT.md`
is the authoritative list** — it covers `QHOME`, `GOAL_POS_EE` and `MAX_SPEED`
too, which this summary omits. Since block and goal are both offset by roughly
+0.08 m in x, the real setup is close to a pure translation of the sim's, and
the *goal-relative* geometry the policy actually observes differs only slightly.
Moving the goal matters more than it might look: `arm_qpos`/`arm_qvel` are 14 of
the 24 observation dimensions, and those depend on where the goal sits relative
to the robot, not just on the block-to-goal offset.

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
- **Teleop/policy produces zero motion, `/pusht/observation_valid` is `false`,
  but `/rigid_bodies` topology looks fine**: check
  `ros2 lifecycle get /mocap4r2_optitrack_driver_node` — a fresh
  `optitrack2.launch.py` relaunch always comes up `inactive` and stays that
  way until you re-run the `activate` command from step 1. It's easy to
  restart the OptiTrack driver (e.g. after Motive dropped) and forget this
  step, since `ros2 node list`/`ros2 topic info` both look completely normal
  while it's inactive — only `ros2 topic echo <topic> --once` (or checking
  `observation_valid`) reveals that nothing is actually flowing.
- **Teleop/policy produces zero motion, `observation_valid` is `true`, deadman
  and `/pusht/executed_action` both look correct, but
  `/fr3_arm_controller/joint_trajectory` never produces output even though
  `/servo_node/delta_twist_cmds` is flowing at 20 Hz**: Servo silently ignores
  Cartesian twist commands (no error, no warning on `/servo_node/status`)
  until `switch_command_type` has been called since its last restart — see
  step 3. `ps5_teleop_node` now does this automatically on startup, but if
  `servo_node` itself gets restarted independently later (without restarting
  `franka_pusht`), the mode reverts and needs the manual service call again.
- **The tool slowly drifts in height (eventually far enough to hit the table),
  even though only X/Y is ever commanded**: this was `moveit_servo`'s
  `apply_twist_commands_about_ee_frame` defaulting to `true`, which applied
  twists about `fr3_pusher_tcp` instead of `fr3_link0` -- silently
  contradicting `robot_link_command_frame: fr3_link0` in the same config, and
  ignoring the `header.frame_id` we set. Because the pusher points at the
  floor, the tool frame is flipped (its +Z points *down*) and yawed ~44 deg, so
  a nominally-horizontal command leaked 1-2 mm/s of vertical velocity that
  accumulated without bound, and any height correction ran inverted (its
  signature: the correction pegged at one limit, then flipped and pegged at the
  other). Now set to `false` explicitly in `fr3_servo_config.yaml`; measured
  height then held to +/-1 mm across a full teleop sweep, versus 5.7 cm of
  drift before. It is a launch-time parameter, so `moveit.launch.py` must be
  restarted after changing it.
  Note this also changes what the horizontal commands mean, so
  `goal_to_robot_quaternion_wxyz` is only correct-by-construction under
  `false` -- re-check direction feel after ever touching this.

## Calibration status (current as of 2026-08-20)

Every geometric transform below is measured and independently validated. The
values live in `config/pusht_robot.yaml`, each with its derivation in a comment.

| parameter | value | how it was validated |
| --- | --- | --- |
| `block_marker_to_object_quaternion_wxyz` | `[0, 0, 0.707107, 0.707107]` | 152.4 mm probe length measurement only comes out right if the stem axis is marker −X |
| `block_marker_to_object_translation` | `[0.03189, 0.0, 0.00126]` | sphere-probe touch-off, ±1.21 mm residuals |
| `fixed_goal_quaternion_wxyz` | `[0.016196, 0.000419, 0.715687, 0.698234]` | `block_quat_rel_goal` is exactly `[1,0,0,0]` with the block at the goal |
| `goal_to_robot_quaternion_wxyz` | `[0.999127, 0.008619, -0.014088, -0.038362]` | −5.6° mean error, measured non-circularly against `ee_pos_rel_goal` |
| `z_hold_target` | `0.050` | height held to ±1 mm; 5 mm above block mid-height for slab clearance near singularities |
| `pole_base_to_robot_base_*` | see config | EE estimate lands ~5 cm from the goal, physically consistent |

Block geometry, confirmed by hand and matching `pusht_mjx` exactly: **150 mm**
overall length, **120 mm** crossbar span, **50 mm** stem width / thickness /
crossbar depth. The probe independently measured the length as 152.4 mm, so
probe error is ~1.6%.

Known residual errors: the goal frame's z-axis is 1.89° off true vertical, and
`goal_to_robot` carries a 4.4° yaw — both consistent with the ~2-3° spread of
the hand-eye calibration, and both small enough not to matter at the 5 cm
success threshold.

- **Frame conventions** (fixed 2026-08-20 — read this before touching any
  transform). Three frames are in play and two of them are **Y-up**:
  - `mocap4r2` publishes `/rigid_bodies` **Y-up** with `frame_id: map`, but its
    TF frame `optitrack` is **Z-up**; `p_optitrack = Rx(90°) · p_map`. Verified
    live: `objectPushT` reads `[2.0796, 0.0910, 1.2820]` on `/rigid_bodies` and
    `[2.080, -1.282, 0.091]` via `tf2_echo optitrack objectPushT`. The
    `hand_eye_calibration` tool solves against the **TF** frame, so its result
    must not be applied to `/rigid_bodies` coordinates directly (doing so
    produces a bogus ~1.8 m error and an apparent 1.3 m height offset).
  - Motive's `objectPushT` rigid body is itself defined **Y-up**.
  - `pusht_mjx` is **Z-up**, with the canonical block frame's +X along the stem
    toward the crossbar.
  The observation is expressed entirely in the goal frame, so it is *invariant*
  to any global rotation of the world frame — converting the bridge output
  Y-up→Z-up would change nothing. What sets the observation's axis layout is
  the **goal frame's own orientation**, which is fixed via
  `block_marker_to_object_*` and `fixed_goal_quaternion_wxyz`.
- **T-block origin offset**: Motive's `objectPushT` pivot sits **~32 mm toward
  the crossbar** from the stem-bar centre that `pusht_mjx` uses as the block
  origin. Measured by sphere-probe touch-off, since `fr3_pusher_tcp` is the
  centre of the pusher's 15 mm sphere (`custom_pusher_ee.xacro`) — at contact
  the TCP therefore lies exactly 15 mm along the touched face's outward normal
  whatever the approach angle, like a CMM touch probe. Each touch gives one
  linear equation in the marker frame:
  `m^T offset = m^T v − (plane + 0.015)`, with `m = R_off·n_canonical` and
  `v = R_markerᵀ(p_tip_map − p_marker)`. Touch `stem_end`, `crossbar_end` and
  one crossbar side face, then least-squares. Because everything is computed in
  the marker frame, the block sliding *between* touches is harmless — only
  motion *during* a sample matters.
  This was `[0,0,0]` before, biasing `block_pos_rel_goal` by up to ~64 mm as the
  block rotated away from the goal orientation (the bias enters as
  `(R_marker − R_marker_goal)·offset`, so it vanishes at the goal orientation
  and grows to `2·|offset|` at 180°) — against a 5 cm success threshold.
  The vertical component is deliberately left at 0, not missing: with the block
  flat on the table the block-to-goal rotation is a pure yaw about the vertical,
  which leaves a vertical vector unchanged, so it cancels exactly.
  **Take probe samples only in Execution mode with FCI active.** In Programming
  mode the controllers go `inactive` and a fallback `joint_state_publisher`
  serves frozen defaults, so FK silently reports the tip at `z_hold_target`
  instead of its true position. Check `ros2 control list_controllers` first.
- **T-block tracking**: live via OptiTrack (`objectPushT` rigid body).
  `block_marker_to_object_quaternion_wxyz` maps Motive's Y-up marker frame onto
  the sim's Z-up canonical frame (see the derivation in `pusht_robot.yaml`).
  It was previously identity, which left the **vertical axis at index 1** of
  `block_pos_rel_goal`/`ee_pos_rel_goal` while `pusht_mjx` and the trained
  policy read **index 2** as vertical — the position triples were scrambled
  relative to sim. Identity had looked "validated" only because a block placed
  at the goal yields a near-identity relative pose under *any* frame
  convention, so that check proved nothing.
  Validated after the fix: with the block at a random spot on the table,
  `block_pos_rel_goal = [-0.031, -0.036, 0.0012]` — the vertical component is
  1.2 mm and now sits in index 2, and `block_quat_rel_goal` is exactly
  `[1,0,0,0]` when the block is at the goal.
- **DATA COLLECTED BEFORE 2026-08-20 IS NOT USABLE** for sim-matched training:
  its observations carry the scrambled axis layout above, and its actions are
  sign-flipped in X/Y because `goal_to_robot_quaternion_wxyz` was ~180° wrong.
  The tool-frame bug means the action mapping was not even constant within an
  episode (it varied with arm pose), so no fixed rotation can repair it.
- **EE tracking**: the real pusher tool carries no OptiTrack marker. EE pose
  comes from forward kinematics (`fr3_link0 -> fr3_pusher_tcp` via `/tf`)
  composed with `franka_pole_base`'s live tracked pose and a fixed, calibrated
  `pole_base_to_robot_base_translation/quaternion_wxyz` offset (see
  `ee_from_tf` in `pusht_robot.yaml` and `observation_node.py`). This
  self-corrects if the robot's stand is ever bumped/repositioned, unlike a
  hardcoded world-to-base constant.
- **Fixed goal pose**: set from a real physical T-block placement.
- **Home position**: tool tip `[0.40, -0.10, 0.050]` in `fr3_link0`, exactly
  vertical, 23.6° of joint-limit margin, 15 mm of pusher-sphere clearance above
  the 0.020 slab. Solved via `/compute_ik` and chosen to put `arm_qpos` near the
  distribution the policy was trained on (see `HOME_JOINTS` in
  `ps5_teleop_node.py`). Keep it equal to `z_hold_target`, and keep
  `servo_ik_node`'s `nullspace_target` equal to it too.
- **`goal_to_robot_quaternion_wxyz`: validated independently 2026-08-20.**
  Derived as `R_handeye · Rx(90°) · R_goal` — the `Rx(90°)` being the Y-up→Z-up
  step described above. The result is **≈identity** (4.4° yaw, 1.9° tilt), which
  independently reproduces `pusht_mjx`, where the goal body carries
  `quat="1 0 0 0"` and the goal frame is axis-aligned with the robot base. That
  agreement is the strongest single check that the whole chain is now
  sim-consistent.
  Validated non-circularly by comparing `d(ee_pos_rel_goal)/dt` (taken from the
  OptiTrack-anchored observation, which never touches this rotation) against the
  commanded action: **−5.6° mean error**, with vertical leakage of 2.9 mm/s
  against 61 mm/s horizontal (consistent with the 1.9° tilt).
  Earlier values were all wrong, and worth knowing about as traps:
  identity; a least-squares regression candidate (~169°, failed live); and
  ~175.5° — off by ~180° because it composed the Z-up hand-eye rotation with the
  Y-up goal quaternion, omitting the `Rx(90°)`.
  Beware of validating this by driving the robot and comparing FK motion to the
  commanded action: that is **circular**, because `safety_node` uses this same
  rotation to build the command, so it only ever confirms that Servo executes
  base-frame twists faithfully. Always validate against `ee_pos_rel_goal` from
  the observation instead. Direction "feel" is not evidence either — the old
  ~175.5° value felt fine for a whole session.
- **Pusher height hold**: active, validated 2026-08-19 (held to +/-1 mm across
  a teleop sweep). `safety_node` servos the tool to a fixed `z_hold_target` in
  `fr3_link0`, read from forward kinematics via `/tf` rather than from the
  observation's `ee_pos_rel_goal` (FK is exact; the observation's EE estimate
  inherits the OptiTrack `pole_base` calibration error, which a height hold
  must not chase). This mirrors the `pusht_mjx` sim, whose differential IK
  re-solves `Z_HOLD` every substep — the policy was trained with height held
  as a hard constraint, so it never learned to hold it itself. Keep
  `z_hold_target` equal to the home pose's TCP height.
- **Orientation hold**: implemented but disabled (`orientation_hold_enabled:
  false`), and its configured quaternion is a placeholder ~44 deg off this
  rig's real tool pose — read the real value with `tf2_echo` before enabling.
  Deferred because measured orientation was already stable to <0.01 deg across
  a sweep. See the comments in `pusht_common.yaml`.
- **NOT yet done**: real workspace bounds (`workspace_x/y/z`) — currently a
  generous provisional box for testing, not the actual measured-safe
  envelope (move the tool to each true physical edge and read
  `ee_pos_rel_goal`, per the original checklist). Note `workspace_z` is not
  enforced at all today: `safety_node` only clamps X/Y, so the height hold
  above is what actually keeps the tool off the table.
- **Known sim/real gaps** (relevant to policy deployment, not teleop):
  `max_linear_speed` is 0.08 m/s here versus `MAX_SPEED = 0.35` in
  `pusht_mjx`, so the policy has ~4x less authority per action than it was
  trained with; and Servo resolves the arm's redundancy differently from the
  sim's null-space pull toward a fixed `QHOME`, while `arm_qpos`/`arm_qvel`
  are half of the 24-dim observation.

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
