# Franka Pizza

ROS 2 application package for real FR3 pizza-sauce spreading: kinesthetic
demonstration collection. Same tool as `franka_pusht` (`fr3_pusher_tcp`), same
fixed-height/fixed-orientation constraint, but a 2-D free-space task with no
external object to track and no single goal pose -- spreading covers an area,
it doesn't reach a target.

- `NEED_TO_CONFIGURE.md` -- the calibration checklist. **Start here**; nothing
  moves until it is worked through.

## Status

**Scaffolded, not yet calibrated.** `home_position` and `home_joint_positions`
are placeholders, `calibration_configured` is `false`, and `observation_node`
outputs `observation_valid: false` while it is. The kinesthetic launch has no
`safety_node`, so nothing enforces this on the arm -- it is there so a bad
calibration announces itself rather than silently producing plausible-looking
garbage (see `franka_bulbscrew/FINDINGS.md` B1/B15 for what that failure mode
looks like elsewhere in this workspace).

## Why this package is smaller than `franka_pusht` or `franka_bulbscrew`

Both of those track an external object via OptiTrack at runtime -- the
T-block, the bulb -- and the observation is built relative to it. Sauce
spreading has no such object: the board doesn't move, and there is nothing
else to sense. So:

- **No `optitrack_bridge_node`, no rigid-body subscription at runtime.**
  OptiTrack is used exactly once, offline, to measure `home_position` (a
  fixed point in `fr3_link0`) -- after that it plays no further part. Nothing
  can go stale from a marker being removed or the driver dropping mid-session,
  because nothing at runtime depends on it.
- **No gripper node.** The tool has no actuation.
- **Observation is 17-D**, not 24 or 25:
  `ee_pos_rel_home[3] + arm_qpos[7] + arm_qvel[7]`. Height and orientation are
  not separate observation dims because the task holds both constant, not
  because they don't matter -- dim 2 (z) should stay near `z_hold_target`
  through a good demonstration, worth checking per episode.
- **Action is 2-D**: `[vx, vy] / max_linear_speed`, matching PushT's action
  space, not BulbScrew's 5-D one.

## Running

**Terminal 1 -- OptiTrack.** Only needed for the one-off `home_position`
reading during calibration (`NEED_TO_CONFIGURE.md` step 2); not required once
`calibration_configured: true`.

```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

> The driver comes up `inactive` on every launch and can drop back to
> `inactive` on its own. See `franka_bulbscrew/README.md`'s bring-up section
> for the traps this caused there -- they apply identically here whenever
> OptiTrack is in the loop.

**Terminal 2 -- arm.** Desk in Execution mode, FCI active, user stop released.

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=10.90.90.177 use_fake_hardware:=false load_gripper:=false ee_id:=custom_pusher_ee
```

`ee_id:=custom_pusher_ee`, not `none` or `franka_hand`: `fr3_pusher_tcp` only
exists in the TF tree under this specific end-effector variant, confirmed
repeatedly this session -- every node here depends on that frame. Check it
actually exists before relying on anything:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_pusher_tcp
```

### Mode A -- kinesthetic teaching (the only mode this package has so far)

```bash
ros2 launch franka_pizza pizza_kinesthetic.launch.py
ros2 run franka_pizza guiding_mode --ros-args -p enable:=true
```

The arm goes limp apart from gravity support. **Keep a hand on it the first
time.** Gamepad, since both hands are on the robot:

| button | action |
| --- | --- |
| Circle (1) | start episode |
| Square (3) | stop episode and save |
| PS (10) | home the arm, then hand it back to guiding |

Per episode: **PS** -> **Circle** -> spread -> **Square**. Homing is refused
while recording, since that motion would be captured as part of the
demonstration.

Return to normal control before anything that commands the arm:

```bash
ros2 run franka_pizza guiding_mode --ros-args -p enable:=false
```

Episodes land in `~/pizza_data/kinesthetic_<timestamp>/`. Actions are
*derived* from the motion produced -- see `kinesthetic_recorder_node.py`.

### Not built yet

**Teleop and policy deployment.** These need `safety_node` and
`servo_ik_node` doing the height/orientation hold that kinesthetic teaching
gets for free from the operator's own contact with the board.
`franka_pusht`'s versions already do exactly this (fixed height, fixed
orientation, 2-D horizontal action) and are the right starting point to copy
rather than write from scratch -- see `pizza_common.yaml`'s
`z_hold_*`/`orientation_hold_*` parameters, which exist already, unused, for
that purpose.

**A success/coverage criterion.** Unlike PushT and BulbScrew, spreading has
no single target pose to measure distance-to, so there is no
`extract_success_trajectories` equivalent here. If a coverage metric ever
becomes available (e.g. from a camera), it would need external sensing this
package does not currently have.
