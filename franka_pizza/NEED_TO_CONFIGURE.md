# Calibration checklist

Work through these in order. `observation_node` outputs `observation_valid:
false` while `calibration_configured` is `false`, so nothing can silently run
on placeholder geometry -- but note the kinesthetic launch has no `safety_node`
to enforce anything downstream of that; this flag documents state, it does not
gate the arm.

## 1. Confirm (or redo) the hand-eye anchor

`franka_bulbscrew`'s `optitrack_to_robot_base_translation` /
`_quaternion_wxyz` is a property of the robot base and the mocap system, not
of any task -- if the base has not moved since that calibration, reuse it
directly rather than re-deriving it. If it *has* moved (check: has anyone
driven the stand, or has the FR3 been re-mounted?), redo the hand-eye
calibration the same way:

```bash
ros2 run optitrack_robot_calibration hand_eye_calibration --ros-args \
  -p config_path:=<path>/fr3_waypoints.yaml \
  -p trajectory_controller:=fr3_arm_controller \
  -p referenceA:=fr3_link0 -p referenceB:=optitrack \
  -p matchA:=fr3_link8 -p matchB:=fr3-calibration-ee
```

## 2. Measure the board centre

Place an OptiTrack marker (or a spare rigid body) exactly at the board's
physical centre, resting on the surface. Read its pose from `/rigid_bodies`,
push it through the hand-eye anchor from step 1 (Y-up `map` -> Z-up
`optitrack` -> `fr3_link0` -- see `franka_bulbscrew/FINDINGS.md` B2 for the
frame conventions, which apply identically here), and set
`config/pizza_robot.yaml`'s `home_position` to the result.

The marker is only needed for this one reading -- nothing in the running
pipeline subscribes to OptiTrack at all (see `observation_node.py`'s
docstring). It can be removed once this is done.

## 3. Measure the board height

Jog the tool down until it touches the board surface (gravity-compensated
guiding, or teleop once that exists) and read

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_pusher_tcp
```

Set `config/pizza_common.yaml`'s `z_hold_target` to that height. This is a
robot measurement with no mocap in it, same as `franka_bulbscrew`'s plank
touch -- prefer it over anything derived through OptiTrack, for the same
reason FINDINGS documents there (B3): a check anchored to physical contact
can fail; one that merely looks plausible cannot.

## 4. Solve the home joint configuration

With `home_position` and `z_hold_target` known, solve for the arm
configuration that places `fr3_pusher_tcp` there, tool pointing straight
down:

```bash
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "..."
```

(see `franka_bulbscrew/franka_bulbscrew/ps5_teleop_node.py`'s home-solving
comments for the damped-least-squares approach used there, if `/compute_ik`
does not converge cleanly). **Drive the real robot there and confirm it looks
right before trusting it** -- do not just take the solver's word.

Set both `config/pizza_robot.yaml`'s `home_joint_positions` and
`kinesthetic_recorder_node.py`'s `HOME_JOINTS` to the result. They must agree;
nothing currently derives one from the other automatically.

## 5. Set `calibration_configured: true`

Only once all of the above are real measurements, not the placeholders this
package ships with.

## 6. (Later, for deployment only) workspace bounds and orientation confirmation

Not needed for kinesthetic collection -- there is no `safety_node` in that
path to enforce a workspace box. Owed before any policy is deployed:
drive to each real edge of the reachable board area and read
`ee_pos_rel_home`; confirm `orientation_hold_quaternion_wxyz` (copied from
`franka_pusht`) still matches this tool's measured tool-down orientation to
within a degree or so.
