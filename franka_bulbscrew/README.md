# Franka BulbScrew

ROS 2 application package for real FR3 light-bulb screwing: teleoperated
demonstrations, safety arbitration, and policy deployment. Mirrors the
architecture of `franka_pusht`, adapted for a 3-D task with a rotational
screwing degree of freedom and a gripper.

- `NEED_TO_CONFIGURE.md` — the calibration checklist. **Start here**; nothing
  moves until it is worked through.
- `SIM_ALIGNMENT.md` — how to bend `bulbscrew_mjx` to match the measured rig.

This task is being set up **real-first**, then the sim is matched to it. That is
deliberately the opposite of the PushT order, and it is the better one: most of
the pain there came from finding out, after collecting data, that the real
observation did not mean what the sim thought it meant.

## Status

**Calibrated and collecting** (2026-09-06). The full chain -- hand-eye anchor,
plank, bulb frame, socket seat -- has been re-measured after the robot base, the
socket and the bulb's markers all moved, and the seat height reproduces the
independent 2026-08-30 figure to 1.5 mm.

The task **starts with the bulb already held**: an episode is carry -> align ->
screw, with no reach-and-grasp phase.

Two things are still owed, neither blocking. `bulbscrew_mjx` has not been
updated to the measured rig (`SIM_ALIGNMENT.md` §3b lists every change), and
`max_yaw_rate` is set from the configured value rather than a measured one --
yaw saturates on 10-16% of steps, so it may be labelling turns slower than they
were performed.

## Interfaces

**Observation (25-D)** — `bulbscrew_mjx`'s layout with the two fingertip tactile
dims removed, since the real Franka Hand has no tactile sensing (the sim drops
them too; see `SIM_ALIGNMENT.md` §2.1):

| dims | quantity |
|---|---|
| 0:3 | bulb screw-tip position relative to the socket seat |
| 3:7 | bulb orientation quaternion (w, x, y, z) |
| 7:10 | end-effector position relative to the bulb neck (grasp point) |
| 10 | gripper opening width |
| 11:18 | arm joint positions |
| 18:25 | arm joint velocities |

**Action (5-D, in [-1, 1])** — note this is *not* 2-D like PushT:

| index | meaning | scale |
|---|---|---|
| 0:3 | end-effector velocity x, y, z | `max_linear_speed` |
| 3 | wrist yaw rate — **the screwing DoF** | `max_yaw_rate` |
| 4 | gripper, −1 closed .. +1 open | `grip_max` per finger |

## Pipeline

```
optitrack_bridge_node   /rigid_bodies -> per-body PoseStamped
observation_node        poses + FK + gripper width -> 25-D observation
ps5_teleop_node   \
policy_node        }->  5-D action
safety_node             gates, clamps, 5-D action -> 6-D twist + gripper width
servo_ik_node           twist -> joint velocities (JointJog) with null-space term
gripper_node            width -> franka_gripper Grasp/Move goals
data_collection_node    records episodes to ~/bulbscrew_data
```

`safety_node` never talks to Servo directly. It publishes a twist that
`servo_ik_node` resolves, so redundancy is resolved the way the sim does rather
than however Servo chooses — which on PushT was the difference between a policy
that worked and one that stalled a few steps in.

### The screwing DoF

`safety_node` mirrors `bulbscrew_mjx`'s `_differential_ik` exactly: roll and
pitch are *servoed* to keep the gripper vertical, while yaw is *commanded* and
**overwrites** the hold's z term rather than adding to it.

```
rot = KP_ROT * quat_error(Q_HOLD, ee_quat)
rot[2] = yaw_rate
```

A PushT-style full orientation hold would silently suppress the screwing motion
and make the task unsolvable while everything still looked healthy.

## Running

There are **two separate modes**. They use different launch files and must not
be run together -- the teleop stack commands the arm, and nothing may command it
while it is being moved by hand.

### Bring-up, one terminal each, in this order

**Terminal 1 — OptiTrack.**

```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

> **The single most common cause of "nothing works".** The driver comes up
> `inactive` on every launch, and it also **drops back to `inactive` on its own**
> -- twice in one hour on 2026-08-30, with no error anywhere. While inactive,
> `/bulbscrew/bulb_pose` is silent, `observation_valid` is false, and the
> recorder **silently refuses to start an episode**. The gripper buttons keep
> working, so it looks exactly like a dead gamepad.
>
> Check it before every session, and again if a Circle press seems ignored:
>
> ```bash
> ros2 lifecycle get /mocap4r2_optitrack_driver_node    # want: active [3]
> ```
>
> Re-activating is safe at any time. Allow **~5 s** afterwards before starting an
> episode -- the recorder needs the observation to settle, and a start attempted
> immediately still fails.
>
> A full **process restart** (not just deactivate/activate) is required after any
> edit to Motive's asset list. A lifecycle cycle does not re-read it, and the
> driver then pairs names to poses by index -- so every rigid body silently
> carries the wrong name. See FINDINGS.

**Terminal 2 — arm.** Desk in Execution mode, FCI active, Franka Hand enabled as
the end effector, user stop released.

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=10.90.90.177 use_fake_hardware:=false \
  load_gripper:=true ee_id:=franka_hand
```

Both arguments are needed: `ee_id` defaults to `none`, so `load_gripper` alone
silently loads no end effector and the frames stop at `fr3_link8`.

**Terminal 3 — gripper.** `moveit.launch.py` builds this include but never adds
it (commented out at line 340), and `namespace` has no default.

```bash
ros2 launch franka_gripper gripper.launch.py robot_ip:=10.90.90.177 namespace:=fr3
```

**After ANY robot power cycle, home the gripper once:**

```bash
ros2 action send_goal /fr3/franka_gripper/homing franka_msgs/action/Homing "{}"
```

> An un-homed Franka Hand **accepts goals and reports `succeeded`** -- in about
> 0.25 s, far too fast for real travel -- while the fingers do not move and the
> width reads a constant 0.0 mm. Every layer above it looks healthy, so it
> presents as a dead gamepad. Diagnose with
> `ros2 topic echo /fr3/franka_gripper/joint_states --once`: a width pinned at
> 0.0 while commands are flowing means it needs homing.

### Check before collecting

```bash
ros2 lifecycle get /mocap4r2_optitrack_driver_node   # active [3]
ros2 control list_controllers                        # all active
ros2 topic echo /bulbscrew/observation_valid --once   # data: true
ros2 node list | grep -c bulbscrew                   # 5, not 10
ros2 topic info /joy                                 # Publisher count: 1
```

The last two catch a duplicate launch. Two recorders fight over the controller
switch and two `joy_node`s scramble button edge detection -- which looks exactly
like a broken gamepad.

> **If `ros2` commands report an empty world, suspect the daemon before the
> stack.** It caches the graph, and after many processes die -- or after DDS
> shared memory is cleared -- it reports that nothing exists while everything is
> running fine.
>
> ```bash
> ros2 node list --no-daemon      # if THIS shows nodes, it is the daemon
> ros2 daemon stop && ros2 daemon start
> ```

If `observation_valid` is false with everything else healthy, it is almost
always the OptiTrack driver.

### Mode A — kinesthetic teaching (recommended for demonstrations)

Guide the arm by hand. Preferred for this task: it is contact-rich, and your
hands feel the threads engage in a way a gamepad cannot.

**Terminal 4 — collection stack:**

```bash
ros2 launch franka_bulbscrew bulbscrew_kinesthetic.launch.py
```

**Terminal 5 — hand guiding.** Let terminal 4 settle first; switching
controllers while something else is starting up has tripped
`communication_constraints_violation` and dropped the whole stack.

```bash
ros2 run franka_bulbscrew guiding_mode --ros-args -p enable:=true
```

The arm goes limp apart from gravity support. **Keep a hand on it the first
time.** Gamepad, since both hands are on the robot:

| button | action |
| --- | --- |
| Circle (1) | start episode |
| Square (3) | stop episode and save |
| Cross / X (0) | close gripper |
| Triangle (2) | open gripper |
| PS (10) | home the arm, then hand it back to guiding |

Per episode, **the task now starts with the bulb already held**:

1. **PS** — home. The jaws open automatically, before the arm moves
2. place the bulb in the jaws
3. **X** — close on it
4. **Circle** — start recording
5. carry, align, screw
6. **Square** — stop and save

Homing is refused while recording, since that motion would be captured as part
of the demonstration. Home before the bulb is in hand, never after — the arm
moves through a trajectory and nothing holds the bulb during it.

The home pose puts the tool where the bulb used to be picked up, lifted clear:
the screw tip hangs 71 mm above the plank and the socket is 0.325 m away, so an
episode is carry -> align -> slot, with no approach phase.

**The task is SLOTTING, not screwing** (changed 2026-09-06): drop the bulb into
the socket mouth and leave it there. No turns are needed, and any final rotation
counts. The bulb will rest at a lean -- 6 to 16 deg across two measured attempts
-- which is expected: only screwing pulls it perpendicular, so uprightness is
not a success condition.

Success is position alone: `d_seat < 20 mm`, held 1 s. The goal frame remains
the fully-screwed pose, which is repeatable to 0.02 mm, so a slotted bulb reads
as roughly 10-12 mm from it.

Return to normal control before teleop or anything that commands the arm:

```bash
ros2 run franka_bulbscrew guiding_mode --ros-args -p enable:=false
```

Episodes land in `~/bulbscrew_data/kinesthetic_<timestamp>/`. Actions are
*derived* from the motion produced -- see `kinesthetic_recorder_node`.

### If a button seems ignored

The recorder declines to start when the observation is invalid, and that refusal
currently goes only into the service response -- nothing surfaces it. Probe it
directly:

```bash
ros2 service call /bulbscrew/start_episode std_srvs/srv/Trigger
```

- `observation unavailable or invalid` -> the OptiTrack driver, almost always
- `episode already active` -> it *is* recording; press Square
- `success=True` -> recording started from this call

Note a session directory and its `metadata.json` are written when the recorder
starts, so an empty session directory does **not** mean an episode was lost --
it means none ever started.

### Recovering without restarting

If the controllers go inactive (Desk mode change, user stop, a reflex), the
hardware component drops to `unconfigured` and cannot be activated directly:

```bash
ros2 control set_hardware_component_state FrankaHardwareInterface active
ros2 control set_controller_state joint_state_broadcaster inactive   # configure
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state fr3_arm_controller active
ros2 control set_controller_state franka_robot_state_broadcaster active
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

The `inactive` step is not a typo -- an `unconfigured` controller cannot go
straight to `active`, and the error message does not say so.

**Then verify the joint states are real.** A fallback `joint_state_publisher`
serves URDF defaults on the same topic, so forward kinematics returns a
plausible, completely wrong pose:

```bash
ros2 topic echo /joint_states --once    # twice; the values must differ slightly
```

Byte-identical readings mean frozen data, not a stationary arm.

### Mode B — gamepad teleoperation

The arm is commanded through `safety_node` -> `servo_ik_node` -> Servo. Use this
to test the control path, validate directions, or deploy a policy.

```bash
ros2 launch franka_bulbscrew bulbscrew_collect.launch.py
```

| control | action |
| --- | --- |
| L1 (4) | dead-man -- nothing moves unless held |
| left stick | x / y translation |
| R2 / L2 triggers | up / down (analog, partial press = partial speed) |
| right stick X | wrist yaw -- the screwing DoF |
| R1 (5) / Square (3) | close / open gripper (latched) |
| PS (10) | home |
| Cross (0) / Circle (1) | start / stop episode |

Servo must have `apply_twist_commands_about_ee_frame: false` (already set). With
the default `true`, twists are applied about the tool frame -- whose +Z points
*down* when the gripper points down -- inverting vertical motion and rotating the
horizontal plane.

### Policy deployment

```bash
ros2 launch franka_bulbscrew bulbscrew_deploy.launch.py
```

Runs `policy_node` in place of teleop, with `command_source: policy`. There is
**no dead-man** in this mode.

## Collecting data

Same flow as PushT: **✕** to start (homes first), teleoperate, **○** to stop
(saves, then homes). Episodes land in
`~/bulbscrew_data/session_<timestamp>/episode_NNNN.npz`.

Recording stops *before* homing begins, so disturbing the scene during the
return-to-home cannot contaminate an episode.

To keep only the demonstrations that solved the task, truncated at success:

```bash
ros2 run franka_bulbscrew extract_success_trajectories
```

It scans both `session_*` (teleop) and `kinesthetic_*` directories, keeps the
episodes that reached **`d_seat < 3 mm held for 1 s`**, and truncates at that
point. Originals are never modified.

That criterion is deliberately **not** the sim's own
`d_seat + 0.1·upright_err < 0.02`. The sim's test is yaw-invariant — a bulb
being a body of revolution — so it cannot tell "resting in the socket mouth"
from "screwed tight", and fires roughly two turns early. `--sim-criterion`
reproduces the old behaviour for comparison; `--depth` and `--hold-s` adjust the
new one.

Episodes with more than 1% implausible bulb frames are skipped: mocap tracking
some other object produces data that looks structurally perfect. Override with
`--allow-mocap-dropouts` only if you know why.

## Reused from `franka_pusht`

`math_utils.py`, `servo_ik_node.py` and `data_collection_node.py` are carried
over, retargeted rather than rewritten. Two constants differ deliberately:

- `nullspace_gain` is **0.5** here (the sim's `KP_NULL`), against PushT's 10.0.
  A hard posture pull fights the vertical carry in a 3-D task.
- `tcp_frame` is `fr3_hand_tcp`, not `fr3_pusher_tcp`.
