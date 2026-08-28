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

**Scaffolded, not yet calibrated.** Every transform in `bulbscrew_robot.yaml` is
a placeholder, `calibration_configured` and `workspace_configured` are both
`false`, and `safety_node` outputs zero while they are. The pipeline runs
end-to-end but will not move the robot until the checklist is done.

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

Bring-up differs from PushT — the Franka Hand replaces the pusher:

```bash
# 1. OptiTrack (remember: a fresh relaunch always needs re-activating)
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate

# 2. Robot, with the gripper (BOTH arguments are needed: ee_id defaults to 'none')
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=10.90.90.177 use_fake_hardware:=false \
  load_gripper:=true ee_id:=franka_hand

# 3. Pipeline
ros2 launch franka_bulbscrew bulbscrew_collect.launch.py
```

Servo must have `apply_twist_commands_about_ee_frame: false` (already set in
`fr3_servo_config.yaml` for PushT). With the default `true`, twists are applied
about the tool frame — whose +Z points *down* when the gripper points down —
which inverts vertical motion and rotates the horizontal plane.

## Controller mapping (DualSense)

| control | action |
|---|---|
| L1 (4) | dead-man — nothing moves unless held |
| left stick | x / y translation |
| R1 (5) / L2 (2) | up / down (z) |
| right stick X | wrist yaw — screw in / out |
| R2 (7) / Square (3) | close / open gripper (latched) |
| PS (10) | home |
| Cross (0) | start episode (homes first) |
| Circle (1) | stop episode (saves, then homes) |

The gripper is latched rather than proportional: a bulb needs a settled grip, a
stick axis would jitter the width every tick, and latching means releasing the
dead-man does not drop the bulb.

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

This applies the sim's own criterion — `d_seat + 0.1·upright_err < 0.02` — and
truncates at the first step meeting it, so a trajectory ends exactly where a sim
episode would. Originals are never modified.

## Reused from `franka_pusht`

`math_utils.py`, `servo_ik_node.py` and `data_collection_node.py` are carried
over, retargeted rather than rewritten. Two constants differ deliberately:

- `nullspace_gain` is **0.5** here (the sim's `KP_NULL`), against PushT's 10.0.
  A hard posture pull fights the vertical carry in a 3-D task.
- `tcp_frame` is `fr3_hand_tcp`, not `fr3_pusher_tcp`.
