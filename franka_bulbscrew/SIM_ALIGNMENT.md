# Aligning `bulbscrew_mjx` with the real FR3 rig

This task is being set up **real-first**: measure the rig, then bend the sim to
match. That is the opposite of how PushT went, and it is the better order — most
of the pain there came from discovering, after collecting data, that the real
observation did not mean what the sim thought it meant.

Fill in §1 as you work through `NEED_TO_CONFIGURE.md`; §2 then tells you exactly
what to change in `bulbscrew_mjx`.

Sim paths are relative to `trust_region_irl/environments/bulbscrew_mjx/`.

---

## 0. Read this first: the frame conventions

Three frames, and **two of them are Y-up**. This is the single easiest way to
produce data that looks fine and trains to nothing.

| frame | up axis | notes |
| --- | --- | --- |
| `/rigid_bodies`, `frame_id: map` (Motive via mocap4r2) | **Y** | what `observation_node` consumes |
| TF frame `optitrack` (same driver) | **Z** | `p_optitrack = Rx(90°) · p_map` |
| `bulbscrew_mjx` world / `fr3_link0` | **Z** | |

Two consequences that cost real time on PushT:

- `hand_eye_calibration` solves against the **Z-up TF** frame. Applying its
  result to **Y-up `/rigid_bodies`** coordinates gives a confident, badly wrong
  answer — there it produced a 1.8 m error that looked like a genuine
  miscalibration for a while.
- Motive's rigid-body frames are themselves Y-up, and their origins are wherever
  Motive put them. `bulb_marker_to_object_*` exists precisely to fix that.

The observation is expressed **relative to the socket seat, in the seat's own
frame**, so what actually sets its axis layout is
`fixed_socket_quaternion_wxyz`. Converting the mocap stream Y-up→Z-up would
change nothing. Get the seat orientation right and the vertical component lands
in index 2, matching sim.

---

## 1. Measured properties of the real rig

All measured and cross-validated 2026-08-28/29. Everything in §2 follows from
these. Positions are in `fr3_link0` (Z-up) unless stated.

### Geometry

| quantity | value | how obtained |
| --- | --- | --- |
| Robot base plane | `z = 0` | definition |
| Plank / work surface | `z = 0.018` | measured directly |
| Bulb head diameter | **60 mm** | measured; sim's 30 mm-radius sphere already matches |
| Bulb overall height | **120 mm** | measured; sim is 124 mm (head top +0.071 to tip −0.053), 3% over |
| Bulb neck diameter | *unverified* | sim: ⌀24 mm at body z = −0.006 |
| Bulb screw-base diameter | *unverified* | sim: ⌀21 mm at z = −0.038. A real E27 is 27 mm |
| **No bulb holder** | — | the bulb stands free on its screw base; delete the sim's `holder` body |
| Tapered mount 85→55 mm, 65 mm tall | measured | the **socket's** stand, not a bulb holder |

### Task points

| quantity | value | notes |
| --- | --- | --- |
| Socket seat (goal) | `[0.5996, 0.0970, 0.0195]` | 0.607 m reach; seat frame 0.52° from true vertical |
| Bulb start, free-standing | tip at `z = −0.0094`, neck `[0.742, −0.161, 0.038]` | 0.761 m reach; IK margin 39.7° |
| Bulb start **varies** | hand-placed each episode | randomise the sim spawn to match |
| Home tool tip | `[0.671, −0.032, 0.250]` | exactly vertical, joint margin 78.7° |
| Home joints (= sim `QHOME`) | `[0.201812, 0.461781, −0.293619, −1.651913, 0.149214, 2.091641, −0.853753]` | midpoint between bulb and socket |
| Tool-down orientation | `[0.0, 0.731027, 0.682348, 0.0]` wxyz | sim's `GOAL_QUAT_EE` matches to 0.50° |

### Calibrated transforms (real side only — not sim parameters)

| parameter | value |
| --- | --- |
| `fr3_link0 → optitrack` | xyz `2.57022, −1.40307, −0.00480`; xyzw `0.00145, 0.00165, 0.99967, 0.02555` |
| `pole_base_to_robot_base_translation` | `[−0.074, −0.00924, −0.03365]` |
| `pole_base_to_robot_base_quaternion_wxyz` | `[0.01351, −0.01733, −0.70912, −0.70475]` |
| `bulb_marker_to_object_translation` | `[0.00223, −0.05689, −0.00088]` |
| `bulb_marker_to_object_quaternion_wxyz` | `[0.707107, −0.707107, 0.0, 0.0]` |
| `fixed_socket_position` | `[2.044912, 0.005265, 1.397291]` (map frame) |
| `fixed_socket_quaternion_wxyz` | `[0.707852, −0.705547, −0.026774, −0.020802]` |

### Control

| quantity | real value | sim ships |
| --- | --- | --- |
| `max_linear_speed` | **0.10** m/s | `MAX_SPEED = 0.35` |
| `max_yaw_rate` | **0.75** rad/s | `MAX_YAW_RATE = 1.5` |
| `grip_max` | 0.04 m/finger | `GRIP_MAX = 0.04` ✓ |
| Gripper total opening | 0.08 m (measured 0.0403/finger open) | ✓ |
| Grasp point | **head centre, +0.041** | `NECK_OFF = −0.006` ✗ |
| Control rate | 20 Hz | `CONTROL_FREQ = 20` ✓ |
| Orientation hold gain | 3.0 | `KP_ROT = 3.0` ✓ |
| Null-space gain | 0.5 | `KP_NULL = 0.5` ✓ |
| Workspace bounds (tool, `fr3_link0`) | x `[0.35, 0.85]`, y `[−0.35, 0.30]`, z `[0.02, 0.45]` | sim has none |
| Collision thresholds | raised to Franka "high" | sim has no reflex |

### Observed from the first demonstration

| quantity | value | implication for sim |
| --- | --- | --- |
| Episode length | **83 s (1667 steps)** | sim `horizon = 400` (20 s) is **4× too short** |
| Yaw saturation | 9.2% of steps at \|action\|=1 | `max_yaw_rate` may need raising on both sides |
| Linear saturation | 1–2% | fine |
| Minimum joint margin | **1.5°** | inside Servo's 5.7° halt margin — see §3.7 |

---

## 2. Changes to make in the sim

### 2.1 Drop the tactile dimensions — do this first

The real Franka Hand has no fingertip tactile sensors, so the real observation
is **25-D**, not 27-D. In `environment.py`:

- remove the `touch` term from `get_observation`
- drop `TOUCH_CLIP`, `_touch()` and the `touch_adr` lookup
- change the observation space from 27 to 25
- remove `touch` from the `base` / `base_rbf` feature functions (or replace it
  with a gripper-width term)

The resulting layout, which both sides must agree on exactly:

| dims | quantity |
|---|---|
| 0:3 | bulb screw-tip position relative to the socket seat |
| 3:7 | bulb orientation quaternion (w, x, y, z) |
| 7:10 | end-effector position relative to the bulb neck |
| 10 | gripper opening width |
| 11:18 | arm joint positions |
| 18:25 | arm joint velocities |

Note dropping dims 11–12 **shifts `arm_qpos` from 13:20 to 11:18** and
`arm_qvel` from 20:27 to 18:25. A silent off-by-two here would be exactly the
class of bug that scrambled PushT's axes.

### 2.2 Scene geometry — `data/scene_mjx_bulb.xml`

Once §1 is measured:

```xml
<body name="socket" pos="X Y Z">      <!-- from fixed_socket_position, in fr3_link0 -->
<body name="holder" pos="X Y Z">      <!-- the bulb's start placement -->
<body name="bulb"   pos="X Y Z">      <!-- start pose, standing in the holder -->
```

Check the work-surface height too. On PushT the sim's ground body sat 5 mm below
the real slab because its geom half-height was not accounted for — every
relative height was off until that was found.

If the real bulb's screw base is an E27 (27 mm) rather than the sim's 21 mm,
widen `bulb_screw` and the socket cavity together, or the insertion clearance
will not match.

**Delete the `holder` body entirely.** There is no bulb holder on the real rig —
the bulb stands free on its own screw base. Remove the four `hold_*` box geoms
and the body that carries them, and set the bulb's start `pos` from the measured
free-standing pose instead of a holder rim.

Two things to watch when you do:

- **The sim bulb may not stand up on its own.** Its only ground contact is
  `bulb_tip_flat`, a 16 × 16 mm box, supporting a 120 mm tall body — a marginal
  footprint. The real bulb is stable on its base, so if the sim one topples,
  widen that geom to the real screw base's diameter (measure it; a standard E27
  is 27 mm against the sim's 21 mm) rather than reinstating a holder. Check this
  before generating demonstrations: a bulb that falls over at reset silently
  ruins every episode.
- The tapered 85 → 55 mm, 65 mm tall piece measured on the rig is the
  **socket's** mount, not a bulb holder. Model it under the socket if it is
  bulky enough to matter for collisions — the gripper approaches the socket
  closely during insertion.

**The start pose varies between episodes**, because the bulb is placed by hand.
`bulbscrew_mjx` currently spawns it at one fixed pose, so it should randomise
the start over whatever spread the real placement actually produces. Training on
a fixed spawn and deploying against a varying one is exactly the sort of
mismatch that surfaces late and looks like a policy failure. Record the spread
from the first batch of episodes and match it.

### 2.3 Grasp point: the head, not the neck

The real bulb is held by its glass **body**, not the neck: the jaws close at
59.7 mm, across the 60 mm head at its widest point, putting the TCP at the head
centre. The neck is small and fragile and is not a practical grip.

So in `environment.py`:

```python
NECK_OFF = 0.041      # was -0.006; the head centre, not the neck
```

and widen whatever the FSM expert uses as its grasp target to match. The name
is now misleading -- it is the grasp point, not the neck.

This matters because observation dims 7:10 are the tool relative to this point.
If the sim grasps the neck while the real robot grasps the head, those three
dims describe a grasp that never happens, and a policy trained on them will
reach for the wrong place.

Note the head is a 60 mm sphere in an 80 mm jaw, so the grip has only 10 mm of
clearance per side and the bulb will pendulum about it -- the body hangs 120 mm
below. Check the sim reproduces that, since it affects insertion.

### 2.4 `environment.py` constants

```python
QHOME = np.array([0.201812, 0.461781, -0.293619,
                  -1.651913, 0.149214, 2.091641, -0.853753])

MAX_SPEED    = 0.10        # was 0.35   <-- see warning
MAX_YAW_RATE = 0.75        # was 1.5    <-- see warning
NECK_OFF     = 0.041       # was -0.006; the head centre, see 2.3
horizon      = 1700        # was 400; the first real demo took 1667 steps
```

`GOAL_POS_EE` should be the home tool tip, `[0.671, -0.032, 0.250]`, and the
scene's socket and bulb bodies placed per §2.2.

**`MAX_SPEED` and `MAX_YAW_RATE` are what silently ruin training.** The recorded
action is normalised to [-1, 1] and multiplied by these. Leave the sim at 0.35
while recording at 0.10 and every demonstration reads as 3.5x faster than it was
performed. Same for yaw. Set them from the *achieved* speed, not the commanded
one.

**The horizon matters more than it looks.** The first hand-guided demonstration
ran 83 s -- 1667 steps -- against the sim's 400-step (20 s) horizon. A sim
episode would be truncated before a comparable trajectory could finish, so the
policy would never see the task completed. Either raise the horizon or expect
faster demonstrations; do not leave them mismatched.

### 2.5 Already matching — do not change

| | value | |
| --- | --- | --- |
| `CONTROL_FREQ` | 20 | |
| `GRIP_MAX` | 0.04 | Franka Hand is 0.04 per finger |
| bulb head | ⌀60 mm | confirmed against the real bulb |
| bulb height | ~124 mm | real is 120 mm; 3% over, close enough to leave |
| `KP_ROT` | 3.0 | mirrored by `orientation_hold_gain` |
| `KP_NULL` | 0.5 | mirrored by `servo_ik_node`'s `nullspace_gain` |
| success test | `d_seat + 0.1·upright_err < 0.02` | used by `extract_success_trajectories` |

---

## 3. Known mismatches to keep in view

1. **Yaw is commanded, not corrected.** Both sides substitute the commanded yaw
   rate into the orientation-hold's z term rather than adding to it. If the real
   side ever "holds" full orientation instead, the screwing DoF is silently
   suppressed and the task becomes unsolvable while looking fine.
2. **Redundancy resolution.** `servo_ik_node` ports the sim's null-space term,
   but with a 6-DOF task a 7-DOF arm has only **one** free DOF, so it pins that
   single dimension; the other six follow from where the tool is. Matching
   `QHOME` *and* the socket position is what makes the joint distributions
   agree. This mattered enormously on PushT: 35° of joint drift moved the
   policy's action from 7° off-target to 153°.
3. **Joint velocity limits.** `servo_ik_node` clips to 30% of the actuator
   limits as a hardware precaution; the sim uses 100%.
4. **No thread mechanics** in the sim (MJX has no SDF collision), so "screwed
   in" is a pose criterion, not thread engagement. The real bulb will actually
   thread, so real demonstrations may include rotation the sim cannot reproduce.
   Worth watching once demonstrations exist.
5. **Gripper dynamics.** The sim uses position-actuated fingers; the real hand
   is driven by `Grasp`/`Move` goals with a force setting. Contact behaviour
   during insertion will differ.
6. **Workspace bounds.** Real `safety_node` clamps; the sim has no such clamp.
   Keep the bounds off the working region.
7. **Hand-guided demonstrations can visit poses the robot cannot reproduce.**
   The first demo reached a minimum joint-limit margin of **1.5 deg**, well
   inside Servo's `joint_limit_margin` of 0.10 rad (5.7 deg). Guiding by hand
   has no such limit, but a policy deployed through `servo_ik_node` would halt
   there. Two consequences: demonstrations should be guided through
   comfortable arm configurations, and the sim should arguably enforce the same
   margin so it does not learn trajectories the hardware refuses to execute.
8. **Redundancy resolution differs between demonstration and deployment.**
   Guiding by hand means the operator picks the elbow; at deployment
   `servo_ik_node` picks it via the null-space term. `arm_qpos`/`arm_qvel` are
   14 of the 25 observation dims, so those distributions can diverge -- the same
   mismatch that made the PushT policy stall. Compare a recorded `arm_qpos`
   against what `servo_ik_node` produces for the same tool path before trusting
   a large dataset.

---

## 3b. Task change: the episode starts with the bulb already held

Adopted 2026-08-30. The reach-and-grasp phase is removed; an episode is
carry -> align -> screw. What this requires in `bulbscrew_mjx`:

**Reset must spawn the bulb in the jaws.** Fingers closed to the measured
**55.9 +- 10.3 mm**, arm at the new `QHOME`, bulb placed at the grasp point
derived from the hand pose. Measured in-hand spread over 29 real
demonstrations, as the randomisation to match:

```
x  +2.9 +- 2.1 mm     y  +4.0 +- 3.5 mm     z  +0.8 +- 3.5 mm
|offset| mean 6.8 mm, max 16.5 mm      -> randomise over about +-4 mm
```

**Holding it is the hard part.** MJX contact-based grasping of a smooth sphere
is unreliable. A `weld` equality between `bulb` and `hand` is the robust option
now that the task never releases. Two costs, both worth stating rather than
discovering: dropping the bulb stops being a possible failure mode, and
`obs[7:10]` becomes exactly constant in sim while it varies by ~7 mm on the real
robot. If that gap matters, use a compliant equality rather than a rigid weld.

**`QHOME`** — set from the real home, `fr3_hand_tcp` at
`[0.744, -0.162, 0.200]`, tool vertical:

```python
QHOME = [-0.216654, 0.825380, 0.003529, -1.152593, -0.002824, 1.977969, -1.260341]
```

**Horizon** can come down. Real episodes with the approach cut average 559 steps
(27.9 s); ~700 leaves headroom.

**The bulb no longer starts on the plank**, so its spawn position and the
`holder` (already slated for deletion) stop mattering. The board still does —
the arm passes over it.

**`action[4]` becomes degenerate.** The gripper never opens, so that dimension
is constant. Keeping it preserves the 5-D action space on both sides; just know
the policy carries a dead input.


## 3c. Task change: slotting, not screwing

Adopted 2026-09-06, superseding the screwing variant above. The bulb is dropped
into the socket mouth and left; nothing is threaded.

**Success is position only.** Measured over two consecutive slotting attempts:

```
             tip in fr3_link0            d_seat    tilt
attempt 0    [0.6063, 0.0786, 0.0780]    9.7 mm    6.08 deg
attempt 1    [0.6084, 0.0785, 0.0807]   12.2 mm   16.09 deg
```

Position repeats to **2.7 mm**; tilt varies by **10 deg**. A slotted bulb rests
against the socket rim at whatever angle it settles -- only screwing pulls it
perpendicular. So the criterion must be yaw-invariant AND must not test
uprightness:

```
solved  <=>  d_seat < 0.020 m, held for 1.0 s
```

**The goal frame stays the FULLY-SCREWED pose**, not a slotted one. That pose is
set by a hard stop, repeats to 0.02 mm and is vertical to 0.40 deg; a slotted
pose would have encoded one attempt's accidental 6 deg lean as the task
definition, and every later episode would have been measured against it.

**Consequences for the sim.** The screwing DoF stops mattering: `MAX_YAW_RATE`
is no longer critical, the horizon can come down further (no 2-4 turns), and the
`success_threshold` must move to the depth rule above. `QHOME`'s joint 7 goes
back to the margin-maximising value, since the 346 deg wrist range is no longer
the binding constraint.

**Regrasping is no longer forced.** Under the screwing variant it was, by
kinematics -- joint 7's full range is 346 deg against 2-4 turns needed. Slotting
should be a single clean insertion, so `action[4]` really may be near-constant
now, unlike under screwing.


## 4. Verifying the alignment

Worth doing before trusting a training run:

1. **Bulb seated ⇒ near-zero.** Seat the bulb by hand; `observation[0:3]` should
   be ≈0 and the upright error small. If not, `fixed_socket_*` and
   `bulb_marker_to_object_*` disagree.
2. **Vertical is index 2.** Move the bulb around at constant height;
   `observation[2]` should stay quiet while 0 and 1 vary. If the *middle*
   component is the quiet one, the Y-up/Z-up convention is wrong.
3. **Action means the same thing.** Command a known action on both sides and
   compare achieved tool speed and yaw rate. This is what catches a `MAX_SPEED`
   or `MAX_YAW_RATE` mismatch.
4. **Direction validated against the observation, never the command.** Comparing
   FK motion to the commanded action through the same transform that built the
   command is circular — it only proves Servo executes faithfully. On PushT that
   produced a confident −2.0° "validation" of a transform wrong by 180°.
