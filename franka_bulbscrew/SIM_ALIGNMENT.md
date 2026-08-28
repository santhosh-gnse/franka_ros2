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

Fill these in as they are measured. Everything in §2 follows from them.

| quantity | value | how obtained |
| --- | --- | --- |
| Table / work surface height in `fr3_link0` | *TBD* | measure directly |
| Bulb head diameter | **60 mm** | measured; matches the sim's 30 mm-radius sphere exactly |
| Bulb overall height | **120 mm** | measured; sim is 124 mm (head top +0.071 to tip −0.053) — 3% over |
| Holder, base square | **85 mm** | measured |
| Holder, top square | **55 mm** | measured |
| Holder height | **65 mm** | measured; sim's holder is a 24 mm square, 30 mm tall — **does not match**, see §2.2 |
| Bulb neck diameter (grasp point) | *verify* | sim: ⌀24 mm at body z = −0.006 |
| Bulb screw-base diameter | *verify* | sim: ⌀21 mm at z = −0.038. A real E27 is 27 mm |
| Bulb tip offset along body −z | *verify* | sim: −0.053 |
| Socket seat, in `map` | *TBD* | `fixed_socket_position` ∘ `socket_seat_offset` |
| Seat frame orientation vs vertical | *TBD* | must be true vertical, see §0 |
| Bulb start pose (in its holder) | *TBD* | record like PushT's episode start pose |
| Home tool tip + joints | *TBD* | `/compute_ik`, gripper vertical, good limit margin |
| Tool-down orientation | *TBD* | `tf2_echo fr3_link0 fr3_hand_tcp`, xyzw → wxyz |
| Action scale — linear | `0.10` m/s | `max_linear_speed` (sim ships 0.35) |
| Action scale — yaw | `0.75` rad/s | `max_yaw_rate` (sim ships 1.5) |
| Gripper max per finger | `0.04` m | `grip_max`; Franka Hand total opening 80 mm |
| Control rate | 20 Hz | matches sim |

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

**The holder needs remodelling — it is the one clear geometry mismatch found so
far.** The real holder is a tapered cup, 85 mm square at the base narrowing to
55 mm at the top, 65 mm tall. The sim's is four thin walls forming a 24 mm
square opening only 30 mm tall. Two consequences:

- With a 60 mm head over a 55 mm opening, the real bulb **rests on the rim**,
  screw end hanging inside — it is not gripped by the walls the way the sim's
  is. That sets the start pose height, and therefore where the gripper must go
  to reach the neck.
- The real cup is far bulkier, so it is a genuine obstacle for the gripper on
  approach and lift. The sim will under-represent collisions there.

Replace the four `hold_*` box geoms with a tapered cup of the real dimensions,
and set the bulb's start `pos` from where it actually rests on the rim rather
than from the sim's current spawn height.

### 2.3 `environment.py` constants

```python
QHOME       = <real HOME_JOINTS>       # from ps5_teleop_node
MAX_SPEED   = 0.10                     # was 0.35  <-- see warning
MAX_YAW_RATE = 0.75                    # was 1.5   <-- see warning
```

**`MAX_SPEED` and `MAX_YAW_RATE` are what silently ruin training.** The recorded
action is normalised to [−1, 1] and multiplied by these. Leave the sim at 0.35
while recording at 0.10 and every demonstration reads as 3.5× faster than it was
performed. Same for yaw. Raise both sides together, and set them from the
*achieved* speed, not the commanded one — on PushT the achieved speed ran ~25%
below the command.

### 2.4 Already matching — do not change

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

---

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
