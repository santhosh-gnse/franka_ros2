# Real-robot configuration checklist — bulb screwing

The package ships fail-closed: `calibration_configured` and `workspace_configured`
are both `false`, and `safety_node` outputs zero while either is. Work through
this in order; every step ends with a check that can actually fail.

All values go in `config/bulbscrew_robot.yaml`. Once the rig is measured,
`SIM_ALIGNMENT.md` turns those measurements into the sim edits.

**The single most important lesson from PushT**: a calibration that "looks
right" is not validated. Several checks there passed while the underlying value
was wrong — most memorably placing the block at the goal and confirming the
relative pose was near-identity, which is true for *any* frame convention and
therefore proves nothing. Each step below says what would actually falsify it.

---

## 0. Hardware and bring-up

The Franka Hand replaces the PushT pusher, so the launch differs:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=10.90.90.177 use_fake_hardware:=false load_gripper:=true
```

Note `load_gripper:=true` and no `ee_id:=custom_pusher_ee`.

Check:

```bash
ros2 control list_controllers          # all active
ros2 topic echo /fr3_gripper/joint_states --once
ros2 action list | grep fr3_gripper    # grasp, move, homing
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

If `fr3_hand_tcp` does not exist, find the actual tool frame and set
`tcp_frame` in both config files. It must be the frame you want to servo.

**Robot must be in Execution mode with FCI active.** In Programming mode the
controllers go inactive and a fallback `joint_state_publisher` serves frozen
defaults — forward kinematics then silently reports a stale pose rather than
erroring. This ruined a calibration sample on PushT. Always confirm with
`ros2 control list_controllers` before trusting any FK-derived measurement.

---

## 1. OptiTrack

The bulb is tracked as `bulb-trirl`; the stand marker `franka_pole_base` is
unchanged from PushT.

```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate   # required every relaunch
ros2 topic echo /rigid_bodies --once
```

A freshly relaunched driver always comes up `inactive`, and nothing about the
node list or topic list reveals it — only the absence of data does. If
`observation_valid` is false, check this first.

Confirm `bulb-trirl` reports a non-zero pose. Motive publishes all-zero
position and `(0,0,0,-1)` orientation for a body it knows about but cannot
currently see, which looks like data but is not.

---

## 2. Robot base anchor

`pole_base_to_robot_base_*` maps the stand marker's Motive frame to `fr3_link0`.
The values shipped are inherited from `franka_pusht`. They are only valid if the
stand and its marker have not moved since.

Verify rather than assume: put the gripper somewhere unambiguous, then compare
the FK position against the mocap-derived one. If they disagree by more than a
centimetre or so, redo the hand-eye calibration
(`optitrack_robot_calibration/hand_eye_calibration`).

Beware when using that tool's result: it solves against the driver's **Z-up**
TF frame `optitrack`, while `/rigid_bodies` is **Y-up** `map`. Applying its
output directly to `/rigid_bodies` coordinates produces a confident, completely
wrong answer (on PushT, a 1.8 m error that briefly looked like a real
miscalibration). See `SIM_ALIGNMENT.md` §0.

---

## 3. Bulb frame — `bulb_marker_to_object_*`

Maps Motive's `bulb-trirl` frame onto the sim's canonical bulb body, whose
**−z axis** carries `bulb_tip_offset` (screw tip) and `bulb_neck_offset` (grasp
point).

Two things to establish:

**Orientation.** Which of the marker frame's axes points along the bulb's screw
axis, and which way. Stand the bulb upright in its holder and read
`/rigid_bodies`; the local axis that maps to world vertical is the screw axis.
Then determine its sign — tip down or tip up — from the geometry.

**Translation.** Where Motive put the origin relative to the bulb body. Use the
gripper fingertips or a known fixture as a touch probe: each contact with a face
of known geometry gives one linear equation

```
m^T offset = m^T v − d,    m = R_off · n_canonical,
v = R_marker^T (p_contact − p_marker)
```

Working in the marker frame means the bulb sliding *between* touches is
harmless; only motion during a sample invalidates it.

Verify against **known geometry**, not against the socket: measure the bulb's
real tip-to-neck distance and confirm the calibrated frame reproduces it. On
PushT the equivalent check (block length, 152.4 mm measured vs 150 mm true)
caught the frame convention independently of any goal placement.

Also confirm `bulb_tip_offset` and `bulb_neck_offset` against your real bulb.
The sim's head diameter (60 mm) already matches the real one, which is a good
sign the model is realistic, but the parts that matter for this transform are
further down: neck ⌀24 mm at z = −0.006, screw ⌀21 mm at z = −0.038, tip at
z = −0.053. If the real bulb is an E27 (27 mm base) rather than the sim's 21 mm,
the offsets and the sim's socket cavity both need revisiting.

---

## 4. Socket seat — `fixed_socket_*`

Bolt the socket down. Its pose is measured once, in the `map` frame.

The seat is `fixed_socket_position` composed with `socket_seat_offset` (18 mm up
the socket's own z). Easiest measurement: seat the bulb correctly by hand, read
`bulb-trirl`, and back out the socket pose using the calibrated bulb frame from
step 3 — the tip then sits exactly at the seat by construction.

**The seat frame's orientation sets the observation's axis layout.** Its z must
be true vertical. Check by moving the bulb around at a constant height and
confirming `block`-equivalent index 2 (`observation[2]`) stays near zero while
0 and 1 vary. If the *middle* component is the quiet one, the Y-up/Z-up
convention is wrong.

Set `calibration_configured: true` only after steps 2–4 are all verified.

---

## 5. Home pose

Solve with `/compute_ik` for a gripper-vertical pose that clears the socket and
holder, then put it in `HOME_JOINTS` (`ps5_teleop_node.py`) **and**
`nullspace_target` (`servo_ik_node`) — they must match.

Requirements, in order of how badly each bites:

1. **Joint-limit margin well beyond Servo's `joint_limit_margin` (0.10 rad).**
   A pose can plan fine and still sit inside the margin, after which Servo halts
   on almost any motion. Check against the real limits, not planning success.
2. **Gripper exactly vertical** — verify with `tf2_echo`, not by eye.
3. **`arm_qpos` near the policy's training distribution.** On PushT this moved
   the policy's action from 67° off-target to 3°. It is not cosmetic.

Then capture the real tool-down orientation and put it in
`hold_quaternion_wxyz` (`bulbscrew_common.yaml`):

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp   # convert xyzw -> wxyz
```

The shipped value is a placeholder. On PushT the equivalent placeholder was 44°
from the real tool pose.

---

## 6. Workspace bounds

Bounds are on the end effector relative to the seat, in the seat frame —
`observation[7:10]` shifted by the bulb neck. **This task is genuinely 3-D**, so
`workspace_z` is a real bound, not a formality as it was on PushT.

Teleoperate to each safe edge, read the observation, add inward margin. Keep the
bounds off the working region: when clamped, `executed_action` is zeroed while
the operator is still commanding, which is self-consistent but produces
unnatural demonstrations. On PushT a too-tight bound silently stalled the policy
mid-episode.

Set `workspace_configured: true` only after every bound is measured.

---

## 7. Speeds

`max_linear_speed` and `max_yaw_rate` are what the demonstrations are
**labelled** with. Whatever you record at must be mirrored in
`bulbscrew_mjx`, or an action of 1.0 means different velocities on each side.
The sim ships 0.35 m/s and 1.5 rad/s; this package starts at 0.10 and 0.75.

Raise both sides together, and measure the *achieved* speed rather than assuming
the commanded one — on PushT the achieved speed ran ~25% below the command.

---

## 8. Before the first real episode

```bash
ros2 topic echo /bulbscrew/observation_valid    # true
ros2 topic echo /bulbscrew/observation --once   # 25 values, all plausible
ros2 topic echo /bulbscrew/executed_action      # zero until the dead-man is held
ros2 topic echo /bulbscrew/desired_twist        # zero until the dead-man is held
```

Then, with a hand on the stop: hold the dead-man and check each DoF separately —
x, y, z, yaw, gripper — confirming each moves the way you expect *before*
combining them.

**Validate direction against the observation, never against the command.**
Comparing FK motion to the commanded action using the same transform that built
the command is circular: it only proves Servo executes faithfully. On PushT that
mistake produced a confident −2.0° "validation" of a transform that was wrong by
180°. Compare `d(observation[7:10])/dt` against the commanded action instead.
