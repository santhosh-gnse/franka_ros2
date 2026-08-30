# Findings and traps

Things that cost real time on this rig, written down so the next person does not
pay for them twice. Most share a shape: **the system looked healthy and produced
plausible numbers while being wrong.** Where a check is listed, it is one that
can actually fail — several "validations" here passed for the wrong reason.

Numbered so they can be cited in review. Split into the ones that bite during
bring-up and the ones that quietly corrupt data or a policy.

---

## A. Bring-up traps

### A1. `load_gripper:=true` alone loads no end effector

`load_gripper` and `ee_id` are **separate** launch arguments and `ee_id`
defaults to `none`. Passing only `load_gripper` gives no error — the frame chain
simply stops at `fr3_link8` and `fr3_hand_tcp` "does not exist".

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=... load_gripper:=true ee_id:=franka_hand
```

### A2. `moveit.launch.py` never starts `franka_gripper`

The include is built (line 319) but never added to the launch description
(commented out, line 340). The gripper needs its own terminal, and its
`namespace` argument has **no default**, so launch fails without one:

```bash
ros2 launch franka_gripper gripper.launch.py robot_ip:=... namespace:=fr3
```

With `namespace:=fr3` the node is `/fr3/franka_gripper`, so its actions are
`/fr3/franka_gripper/{grasp,move}` — **not** the `/fr3_gripper/...` that MoveIt's
`fr3_controllers.yaml` implies.

### A3. "Connection to FCI refused" from the gripper usually means Desk, not FCI

If the **arm connects** and only the **gripper** is refused, FCI is fine — the
two open separate connections, and libfranka prints the same generic string for
any refusal. The real cause is normally that the Franka Hand is not enabled as
the end effector in Desk. `ee_id:=franka_hand` only changes the URDF; it cannot
make a gripper exist.

**Check:** bring up the arm. If `ros2_control_node` logs "Successfully connected
to robot" while the gripper is refused, look at Desk.

### A4. The OptiTrack driver always comes up `inactive`

Every relaunch. Nothing about `ros2 node list` or `ros2 topic list` reveals it —
only the absence of data does.

```bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

If `observation_valid` is false, check this **first**.

### A5. A stalled launch and stale DDS shared memory

Symptom: a launch prints its banner, never starts its node, and survives Ctrl+C.
Cause seen here: 106 orphaned `fastrtps_*` segments in `/dev/shm` accumulated
over hours of restarts.

Kill the stalled process, then remove **only** segments no live process has
mapped (`/proc/*/maps`). Removing one that is in use will break a running node.
Do it with all ROS processes stopped.

### A6. Controllers must be loaded *and* configured, not just declared

Declaring a controller in `fr3_ros_controllers.yaml` only makes its type
spawnable. `controller_manager` still has to **load** and **configure** it before
it can be activated, otherwise a switch fails with an unhelpful "switch failed".
`guiding_mode` now does this itself.

### A7. A parameter name collision across nodes

The configs use a `/**:` wildcard, so **every parameter reaches every node**. A
gripper parameter named `move_action` silently overwrote the MoveGroup action
name in `kinesthetic_recorder_node`, which then waited forever for a MoveGroup
server at the gripper's address. Renamed to `gripper_move_action`.

**Lesson:** under a wildcard namespace, name parameters for their node
(`gripper_*`), not for their role.

### A8. Action goals from a background thread need a multithreaded executor

An action client used from a worker thread under the default single-threaded
`rclpy.spin()` may never see its server; `wait_for_server` just times out. Use
`MultiThreadedExecutor` + `ReentrantCallbackGroup`.

*(Suspected here and fixed pre-emptively, but the actual cause of that
particular failure turned out to be A7 — found only because the log line named
the failing stage. Vague failure messages cost more than they save.)*

---

## B. Things that silently corrupt data or a policy

### B1. Motive publishes a zero pose for bodies it cannot see

A rigid body that is defined but not currently visible is published as position
`(0,0,0)`, orientation `(0,0,0,-1)` — **structurally valid, at full rate**. Freshness
checks accept it happily, and the observation is built from a bogus pose while
`observation_valid` still reports `true`.

`observation_node` now rejects the exact-origin placeholder and logs
`"<body> is not visible to OptiTrack"`. **The same latent bug existed in
`franka_pusht`** and was fixed there too.

That guard is necessary and not sufficient — see B15.

### B2. Two Y-up frames and one Z-up frame

| frame | up axis |
| --- | --- |
| `/rigid_bodies`, `frame_id: map` | **Y** |
| TF frame `optitrack` (same driver) | **Z** |
| `bulbscrew_mjx` / `fr3_link0` | **Z** |

`p_optitrack = Rx(90°) · p_map`. `hand_eye_calibration` solves against the **TF**
frame; applying its result to `/rigid_bodies` coordinates produces a confident,
badly wrong answer — on PushT a 1.8 m error that looked like a real
miscalibration for a while. Motive's rigid-body frames are themselves Y-up.

### B3. "Place the object at the goal and check the relative pose is small" proves nothing

That is true for **any** frame convention, so it validates nothing about whether
your object frame matches the sim's. On PushT this exact check let an identity
`block_marker_to_object` pass while the observation's axes were scrambled.

**Validate against known geometry instead.** Here: standing the bulb on the
plank and confirming the computed tip stays at 18.0 mm (±1 mm) through a 357°
spin — a test anchored to physical fact, and one the socket cannot fake.

### B4. Direction tests that use the transform under test are circular

Comparing FK motion against the commanded action, when `safety_node` used that
same transform to build the command, only proves Servo executes faithfully. On
PushT it produced a confident **−2.0°** "validation" of a transform that was
wrong by **180°**.

**Compare against the observation instead** — `d(ee_pos_rel_*)/dt` versus the
commanded action, since the observation is anchored to mocap and never touches
the rotation being tested.

### B5. Action scale must match the sim exactly

Actions are normalised to [−1, 1] and multiplied by `max_linear_speed` /
`max_yaw_rate`. Recording at 0.10 m/s while the sim uses 0.35 makes every
demonstration read as **3.5× faster than performed**. Set the sim from the
*achieved* speed, not the commanded one — on PushT the achieved value ran ~25%
below the command.

### B6. Servo's twist frame default inverts vertical motion

`moveit_servo`'s `apply_twist_commands_about_ee_frame` defaults to **true**,
applying twists about the tool frame — whose +Z points *down* when the gripper
points down. It silently contradicted `robot_link_command_frame: fr3_link0` in
the same config and ignored `header.frame_id`.

Effect on PushT: 1–2 mm/s of vertical leakage accumulating without bound, and an
inverted height correction (its signature: the correction pegged at one limit,
flipped, and pegged at the other). Set to `false` explicitly.

### B7. Yaw must be *commanded*, not corrected

For screwing, the orientation hold servos roll and pitch but the yaw rate
**overwrites** the hold's z term rather than adding to it. A PushT-style full
orientation hold silently suppresses the screwing DoF and makes the task
unsolvable while everything looks healthy.

Verified safe: with the tool vertical, wrist yaw *is* rotation about world z, so
the right-invariant error stays purely in z. Roll/pitch error remains at machine
zero through 359° of accumulated screwing.

### B8. Workspace bounds must be against something static

An early version clamped `observation[7:10]`, which in this task is the tool
relative to the **bulb** — an object that moves, and is nearly constant once
grasped. That bounds nothing. (PushT's equivalent index was relative to a fixed
goal, which is why the pattern worked there.) Now bounded in `fr3_link0`.

### B9. `franka_gripper`'s Grasp epsilon is an acceptance window, not a tolerance

If the resulting width falls outside `[width − inner, width + outer]`, the grasp
is reported **failed** and the fingers **stop applying force** — they let go.
Commanding `width: 0` with `epsilon: 0.05` meant anything thicker than 50 mm was
a failure, and the bulb head is 60 mm, so every grasp released a moment after
closing.

Using Grasp as "close and hold with force" requires the window to span the whole
jaw range.

### B10. The collision reflex fires on the task working correctly

Franka's default thresholds are tuned for free-space motion. Squeezing glass,
lifting against the bulb's weight, and loading the wrist while turning all trip
it, dropping the robot into the red error state.

Thresholds are **runtime state** — `franka_hardware` restores its defaults on
every connect, so a manual service call is lost at the next restart.
`collision_behavior_node` re-applies them on startup.

Recover without restarting:
```bash
ros2 action send_goal /service_server/error_recovery franka_msgs/action/ErrorRecovery {}
```

**Tradeoff:** this is the reflex that stops the arm when it hits something it
should not. Raising it means the arm pushes harder before reacting. Lower
`scale` to tighten.

### B11. Hand-guided demonstrations can be unreproducible by the robot

The first kinesthetic demo reached a minimum joint-limit margin of **1.5°**,
inside Servo's `joint_limit_margin` of 5.7°. Guiding by hand has no such limit,
but a policy deployed through `servo_ik_node` **would halt there**.

Guide through comfortable arm configurations, and check the margin per episode.

### B12. Elbow configuration differs between demonstration and deployment

Guiding by hand means the operator picks the elbow; at deployment
`servo_ik_node` picks it via its null-space term. `arm_qpos`/`arm_qvel` are
**14 of the 25 observation dimensions**.

On PushT this was decisive: same block and tool state, only joints differing —

```
at the home pose         action 0.72,   6.7° off target
after 34.8° joint drift  action 0.006, 153° off target
```

Compare a recorded `arm_qpos` against what `servo_ik_node` produces for the same
tool path before trusting a large dataset. If they diverge, replay the tool paths
through the normal stack.

### B13. Demonstrations that continue past success

The first demo solved at step 1370 of 1667 — then carried on and ended
*un-solved*, 12.9 cm from the seat. 18% of it was post-success. Left in, that
teaches a policy to keep fiddling with a bulb that is already seated.

Measure `task_err` exactly as `bulbscrew_mjx` does — `d_seat + 0.1·orn` with
`orn = 2·arccos(|q_w|)` — not with a substitute uprightness measure. A
plausible-looking alternative (`1 − z_axis·ẑ`) put the first solve at step 467
here, off by 903 steps, and turned 18% post-success into 72%.

Stop recording the moment the task is done;
`extract_success_trajectories` also truncates at the first step meeting the
sim's own criterion.

### B14. Episode length versus the sim horizon

That demo ran **83 s (1667 steps)** against `bulbscrew_mjx`'s 400-step (20 s)
horizon. A sim episode would be truncated before a comparable trajectory could
finish, so the policy would never see the task completed.

### B15. Mocap can track the *wrong object* and call it your rigid body

Worse than B1, because the guard there does not catch it. In the first
kinesthetic demo, **683 of 1667 frames (41%)** put the bulb about **1.86 m**
from the socket seat — behind the robot, impossible on a 0.6 m reach.

It is not the all-zero placeholder: an unseen body would read 2.48 m here (the
origin-to-seat distance), and these readings jitter at the centimetre scale.
Motive was **live-tracking something real** and matching it to the `bulb-trirl`
marker set. Back-projected into the `map` frame the phantom sits near
`[0.18, 0.09, 1.44]` — about 1.86 m along `map` +x from the seat, roughly 9 cm
off the floor.

The 11 dropout segments cluster around grasping and carrying — exactly when the
gripper and the operator's hands occlude the markers. The longest is 12.1 s.

What makes it expensive: `observation_valid` stayed `true` throughout, and
`extract_success_trajectories` still accepts the episode, because the frame it
truncates at happens to be a good one. 41% wrong observations would have entered
the dataset looking fine.

**Check every episode before trusting it:**

```python
d_seat = np.linalg.norm(np.load(ep)["states"][:, 0:3], axis=1)
print((d_seat > 0.6).mean())     # must be 0.0
```

And gate it at source, next to the B1 check — reject a bulb pose farther than
~0.6 m from the seat rather than recording it.

### B16. A sim constant copied into the real config, then "validated" against itself

`bulb_tip_offset` (canonical bulb origin → screw tip) was taken as **−0.053 m**
straight from `bulbscrew_mjx`. It was never measured on the real bulb, and the
calibration that followed could not catch it: solving
`bulb_marker_to_object_translation` used the constraint *"the tip sits on the
18 mm plank"* — with that same −53 mm as an input. The check then confirmed the
tip sits at 18.0 mm. It always would have. This is B3's trap wearing different
clothes.

What exposed it was replaying the demonstration in the sim, which puts the arm
(from joint encoders) and the bulb (from mocap) in one picture. The bulb hangs
below the fingers, never between them. Quantified at the moment the bulb is
verifiably screwed home and the jaws are closed on it at 58 mm:

```
TCP from joint encoders   [0.6014, 0.0999, 0.1651]   fr3_link0
socket seat from mocap    [0.5996, 0.0970, 0.0195]
lateral disagreement       1.8 mm, 2.9 mm            <- fine
TCP height above the seat  145.6 mm
top of a seated 120 mm bulb 120.0 mm above the seat
=> the tool is 25.6 mm above the top of the glass while holding it
```

**Two hypotheses fit this equally well**, and the demonstration cannot separate
them:

- **(A) object geometry** — the canonical bulb origin sits ~52 mm too low
  relative to the marker frame, so the grasp point is computed too low.
- **(B) mocap-to-robot anchor** — everything mocap-derived sits ~52 mm too low
  in `fr3_link0`.

They separate only when the bulb is *tilted*, and it never is: while gripped the
tilt is 4.4° median, 17° at p90, and there are **zero** frames past 25°. Inside
that range "along the bulb axis" and "vertically" are the same direction.

```
tilt < 10 deg   n=283   axial +49.5 mm (sd 7.7)   vertical +49.5 mm (sd 8.1)
tilt 10-25 deg  n= 53   axial +52.1 mm (sd 6.8)   vertical +48.0 mm (sd 4.8)
tilt > 25 deg   n=  0
```

Do not read a whole-episode variance comparison as evidence either — it is
dominated by frames where the bulb reads upside down, which are themselves bad.

Circumstantial support for (A): −53 mm puts Motive's origin at the neck/head
junction, where no markers are; they are on the glass head, so the centroid
should sit far higher. Support for (B): the seat measures 1.5 mm above the plank
while the socket sits on a 65 mm stand. But the seat was *back-solved from the
bulb frame*, so that symptom is explained by either.

**The measurement that separates them takes a minute and involves no mocap.**
Jog the TCP down until it touches the plank and read its height from FK:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

- plank at ≈ 0.018 m → the anchor is sound, and the bulb frame is wrong (A)
- plank at ≈ 0.070 m → the anchor is off by ~52 mm (B)

Then re-derive whichever is at fault, plus `fixed_socket_position`, which
inherits from the bulb frame either way. `observation_node` now warns whenever
the jaws are closed and the tool is more than 30 mm from the grasp point — the
check that would have caught this on the first episode.

**The general rule:** a constant that comes from the sim is a hypothesis about
the real object, not a measurement. Mark it as such, and never let it be an
input to the calibration that is supposed to verify it.

---

## C. Quick diagnostic order

When nothing moves, in this order:

1. `ros2 lifecycle get /mocap4r2_optitrack_driver_node` — `active`? (A4)
2. `ros2 topic echo /bulbscrew/observation_valid` — `true`?
3. `ros2 control list_controllers` — all `active`? (Desk in Execution mode with FCI?)
4. `ros2 param get /bulbscrew_safety workspace_configured` — both `*_configured` true?
5. `ros2 topic echo /bulbscrew/executed_action` — non-zero while the dead-man is held?
6. `ros2 topic hz /fr3_arm_controller/joint_trajectory` — is Servo producing output?

Each step isolates one layer. `observation_valid` false with everything else
healthy is almost always A4 or B1.

And after every recording session, before the data is used at all, run the B15
plausibility check over each episode.
