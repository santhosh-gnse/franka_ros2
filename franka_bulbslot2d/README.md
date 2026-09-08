# Franka BulbSlot2D

Sibling of `franka_bulbscrew`: the same slotting task (bulb starts held,
carried to the socket, dropped in and left), reduced from a 5-D teleop action
space to 3-D by observing that the bulb-in-hand home pose and the socket seat
pose are two fixed, already-calibrated points -- the meaningful motion between
them is "down and across" a single vertical plane, not full 3-D, and there is
no wrist rotation needed for slotting at all.

Nothing physical is re-measured here. Home position, socket/seat position,
bulb grasp offsets, and the hand-eye anchor are all copied verbatim from
`franka_bulbscrew/config/bulbscrew_robot.yaml`. This package only changes the
*action space* and the nodes that enforce it.

- No kinesthetic teaching in this version -- data collection is joystick
  teleop only (PS5 controller), same style as `franka_pusht`.
- No `policy_node` yet -- teleop data collection only. Deploying a trained
  policy through this action space is a later step.

## The plane

`fr3_link0` frame, current (2026-09-06) calibration:
- Home (bulb-in-hand start): `[0.7369, -0.1828, 0.2096]`
- Seat (socket target): `[0.6084, 0.0787, 0.0685]`

Horizontal direction from home to seat:
`yaw = atan2(dy, dx) = 2.0275369 rad (116.169 deg)` from `fr3_link0`'s +X axis.
That single yaw defines the whole plane:
```
Y_PRIME = [cos(yaw), sin(yaw), 0]   # ACTUATED -- toward/away from the socket
X_PRIME = [-sin(yaw), cos(yaw), 0]  # HELD -- perpendicular, out-of-plane
Z_hat   = [0, 0, 1]                 # ACTUATED -- vertical, unchanged from world Z
```
Verified numerically: `X_PRIME . home == X_PRIME . seat == -0.5807445` to full
float precision -- home and seat are *exactly* coplanar by construction, not
an approximation. `Y_PRIME . home = -0.48905`, `Y_PRIME . seat = -0.19769`
(seat sits further along +Y_PRIME). `z`: home `0.2096`, seat `0.0685`.

Action is `[a_yprime, a_z, grip]` (3-D). No commanded yaw at all -- full
orientation is held via the same `hold_quaternion_wxyz` mechanism
`franka_bulbscrew` uses, now applied on all 3 angular axes instead of just
roll/pitch, since there's no yaw action left to carve out an exception for.

**Do not trust `bulbscrew_robot.yaml`'s own `workspace_x/y/z` comment block**
if you ever cross-reference it: it quotes a stale home figure
(`[0.671, -0.032, 0.250]`, superseded weeks ago by the current
`[0.7369, -0.1828, 0.2096]`) and a seat figure that matches neither the old
nor the current calibration precisely. This package's `workspace_yprime`/
`workspace_z` are derived fresh from the verified current values above, not
from that block.

## What's reused vs changed from `franka_bulbscrew`

Copied verbatim (only topic namespace `/bulbscrew/` -> `/bulbslot2d/`):
`math_utils.py`, `observation_node.py` (still the full 25-D observation --
this package only changes the action space, not what's observed, so the bulb
is still tracked live via OptiTrack the same way), `gripper_node.py`,
`collision_behavior_node.py`, `optitrack_bridge_node.py`, `servo_ik_node.py`
(same `HOME_JOINTS` nullspace target -- the init pose is unchanged),
`data_collection_node.py`.

Adapted:
- `ps5_teleop_node.py` -- trimmed from 5-D to 3-D, and the left stick now
  commands both plane DOF at once: stick Y drives `a_z` (vertical), stick X
  drives `a_yprime` (along the home-seat line), pushed through one combined
  2-D deadzone rather than two independent per-axis ones -- the whole task is
  a single plane, so a single 2-D stick suffices, unlike `franka_bulbscrew`
  where the stick only had 2 of the task's 4 DOF and the rest needed
  triggers/right-stick. L2/R2 and the right stick are unused. Gripper is
  Cross/Triangle (close/open) and start/stop episode are Circle/Square --
  reassigned from `franka_bulbscrew`'s R1/Square and Cross/Circle
  respectively, to free Cross/Triangle for the gripper.
- `safety_node.py` -- the real geometry change. `franka_bulbscrew`'s
  `action[0:3]` maps directly onto raw `fr3_link0` x/y/z with no rotation;
  that doesn't generalize to a non-axis-aligned plane, so this version builds
  `linear = speed*a_yprime*Y_PRIME + [0,0,speed*a_z] + v_hold*X_PRIME`, where
  `v_hold` is a gain-and-clamp correction toward `x_prime_hold_target` (same
  shape as a `z_hold` elsewhere in this workspace, just projected onto
  `X_PRIME` instead of literal Z). The workspace clamp is likewise projected
  onto `Y_PRIME` instead of raw x/y. Orientation hold is the same mechanism as
  `franka_bulbscrew`, just no longer overwritten on the z (yaw) axis, since
  there's no yaw action to overwrite it with.

## Running

Four terminals, in this order. All need `source /opt/ros/jazzy/setup.bash &&
source install/setup.bash` from `/home/trirl-ip/Documents/new/franka_ros2_ws`
first.

**1. OptiTrack** (bulb tracking -- comes up inactive on every launch, needs an
explicit activate):
```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```
```bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```
Confirm `bulb-trirl` is actually publishing before moving on:
```bash
ros2 topic echo /rigid_bodies --once
```
If this driver has been sitting for hours since a previous session, don't
trust it blindly -- it's silently gone stale before (`ros2 node list` /
`ros2 lifecycle get /mocap4r2_optitrack_driver_node` returning nothing despite
the process being alive). Kill it by PID and relaunch fresh if so.

**2. Arm stack.** Desk in Execution mode, FCI active, user stop released.
**`ee_id:=franka_hand` and `load_gripper:=true` are both required together**
-- `ee_id` defaults to `none`, which attaches no hand model at all and
`fr3_hand_tcp` (everything in this package depends on that frame) will not
exist in TF. `ee_id:=none` looks like it should be harmless and is NOT.
```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=10.90.90.177 use_fake_hardware:=false load_gripper:=true ee_id:=franka_hand
```

**3. Gripper driver** (moveit.launch.py builds this include but never starts
it):
```bash
ros2 launch franka_gripper gripper.launch.py robot_ip:=10.90.90.177 namespace:=fr3
```

**4. This package:**
```bash
ros2 launch franka_bulbslot2d bulbslot2d_collect.launch.py
```

Gamepad:

| button | action |
| --- | --- |
| L1 (4, hold) | dead-man -- gates arm motion only |
| left stick (2-D) | Y-axis = up/down (Z), X-axis = along the home-seat line (Y') |
| Cross / Triangle | close / open gripper (latched) -- works WITHOUT L1 held |
| PS (10) | home |
| Circle (1) | start episode (homes first) |
| Square (3) | stop episode (saves, then homes) |

L2/R2 and the right stick are unused.

## Status as of 2026-09-08

Verified on the real robot, at a safe height well clear of the socket:
- Homing reaches `HOME_JOINTS` correctly (`fr3_hand_tcp` within ~6mm of the
  documented `[0.7369, -0.1828, 0.2096]`).
- The plane math is correct: driving the stick moves the tool along `Y_PRIME`
  while `x_prime_hold` keeps `X_PRIME` within a few mm of target, including
  through a large (~180mm) deliberate vertical excursion.
- Orientation hold is now correct and stable -- see the `hold_quaternion_wxyz`
  fix below. `workspace_configured: true` and `calibration_configured: true`
  are both live in `bulbslot2d_robot.yaml`, not just set ad-hoc.
- Gripper open/close confirmed working independent of the dead-man.

**Two real bugs found and fixed today, not just tuning:**
1. `hold_quaternion_wxyz` inherited from `franka_bulbscrew` was measured
   2026-08-28, before the 2026-09-06 recalibration and before `HOME_JOINTS`
   was re-solved for slotting. Measured fresh at the actual current
   `HOME_JOINTS`, it was **164 degrees off**, not a small yaw drift -- holding
   it actively rotated the gripper the moment the dead-man was pressed. Fixed
   in `bulbslot2d_common.yaml` with a value measured live at the current home
   pose (7345 samples, std ~1e-6 rad).
2. The gripper was originally gated behind the dead-man like the rest of the
   action, inherited unchanged from `franka_bulbscrew`. Changed in
   `safety_node.py` so gripper commands (`pipeline_ok`) are independent of
   arm-motion gating (`motion_safe`/dead-man) -- releasing L1 must not block
   an open/close command the way it intentionally blocks arm motion.

**Not yet done -- pick up here:**
- No full episode has actually been recorded and inspected end-to-end under
  this final configuration (stick mapping and orientation fix both landed
  mid-test). Do that first: Circle to start, a small motion, Square to stop,
  then confirm the saved `.npz` under `~/bulbslot2d_data/session_.../` has
  `states`/`actions`/`next_states` with `actions.shape == (T, 3)`, matching
  `metadata.json`'s `action_dim: 3`.
- Nothing has been driven anywhere near the actual seat point yet -- every
  test so far stayed at a safe height. Only approach the real socket after
  a clean recorded episode confirms the whole pipeline, and go slowly the
  first time.
- Double-check the stick feels right in both directions at least once more
  after a fresh relaunch (left/right and up/down signs were flipped
  interactively this session, based on feel -- `SIGN_YPRIME = -1.0`,
  `SIGN_Z = 1.0` in `ps5_teleop_node.py` are the values that tested correct).
