# Aligning `pusht_mjx` with the real FR3 rig

Everything needed to build a simulation that matches this robot setup, so that
demonstrations recorded here are valid training data for a policy trained there.

Every number below was measured and cross-validated on 2026-08-20. Where a value
has a non-obvious derivation it is given, so you can re-derive rather than trust
it. Provenance for each lives in `config/pusht_robot.yaml` and `README.md`.

Sim paths are relative to
`trust_region_irl/environments/pusht_mjx/` in the `trust-region-irl` repo.

---

## 0. Read this first: the frame conventions

Three frames are in play and **two of them are Y-up**. Getting this wrong is the
single easiest way to produce data that looks fine and trains to nothing.

| frame | up axis | notes |
| --- | --- | --- |
| `/rigid_bodies`, `frame_id: map` (Motive via mocap4r2) | **Y** | what `observation_node` consumes |
| TF frame `optitrack` (same driver) | **Z** | `p_optitrack = Rx(90°) · p_map` |
| `pusht_mjx` world / `fr3_link0` | **Z** | |

Also: Motive's `objectPushT` rigid body is itself defined **Y-up**, and its
origin is **not** the block's centre — see §3.

The observation is expressed entirely in the goal frame, which makes it
*invariant* to any global rotation of the world frame. So converting the mocap
stream Y-up→Z-up changes nothing. What actually sets the observation's axis
layout is **the goal frame's own orientation**, which on the real side is fixed
by `block_marker_to_object_*` and `fixed_goal_quaternion_wxyz`.

On the real rig this is already correct: vertical lands in index 2 of
`block_pos_rel_goal` and `ee_pos_rel_goal`, matching sim.

---

## 1. Measured properties of the real rig

The source of truth. Everything in §2 follows from these.

| quantity | value | how it was obtained |
| --- | --- | --- |
| Table (slab) surface | `z = 0.020` in `fr3_link0` | measured directly |
| T-block | 150 × 120 × 50 mm | measured by hand; sphere probe independently read the length as 152.4 mm (1.6% error) |
| Block mid-height | `z = 0.045` | table + half thickness |
| Goal, canonical, in `fr3_link0` | `[0.5603, 0.0009, 0.0442]` | mocap + hand-eye; its height agrees with the geometric 0.045 to **0.8 mm** by an independent route |
| Goal orientation vs robot base | 4.4° yaw, 1.9° tilt | `goal_to_robot_quaternion_wxyz`, validated at −5.6° against measured motion |
| Block episode start, canonical, in `fr3_link0` | `(0.6729, −0.1391)`, yaw `+88.28°` | see §5 — **re-record if the block is repositioned** |
| Home tool tip | `[0.40, −0.10, 0.050]`, exactly vertical | `/compute_ik`, 23.6° joint-limit margin |
| Home joints | `[0.358647, 0.222581, -0.524795, -2.664501, 0.378454, 2.837589, -2.088795]` | same |
| Pusher height during motion | `0.050` (`z_hold_target`) | held to ±1 mm; 5 mm above block mid-height, see §2 |
| Action scale | `0.10` m/s at \|action\| = 1 | `max_linear_speed` |
| Control rate | 20 Hz | |
| Pusher tool | box 0.02 × 0.02 × 0.125 m, 15 mm sphere tip; TCP at the sphere **centre** | `custom_pusher_ee.xacro` |

---

## 2. Changes to make in the sim

### `data/scene_mjx_free.xml` **and** `data/scene_mjx_joint.xml`

```xml
<!-- goal: was pos="0.5 0.0 0.04" -->
<body name="goal" mocap="true" pos="0.560 0.001 0.044" quat="1 0 0 0">

<!-- ground: was pos="0.5 0.0 0.0075" -->
<body name="ground" pos="0.5 0.0 0.0125" quat="1 0 0 -1">
```

The ground change is easy to miss and matters. Its geom is a box of half-height
0.0075, so the *surface* sits at `0.015`, while the real slab is at `0.020`.
Left alone, the sim block settles 5 mm lower than the real one and every
`*_rel_goal` height is off by that much.

### `environment.py`

```python
QHOME = np.array([0.358647, 0.222581, -0.524795,
                  -2.664501, 0.378454, 2.837589, -2.088795])

BLOCK_POS   = (0.673, -0.139)        # was (0.6, -0.1)
BLOCK_ANGLE = 1.5408                 # rad = 88.28 deg; was np.pi / 2
GOAL_POS_EE = np.array([0.40, -0.10, 0.050])   # was [0.45, 0.1, 0.035]
Z_HOLD      = 0.050                  # was 0.045  <-- see note below
MAX_SPEED   = 0.10                   # was 0.35  <-- see the warning below
```

**`MAX_SPEED` is the one that silently ruins training.** The recorded action is
normalised to [-1, 1] and multiplied by this. At the sim default an action of
1.0 means 0.35 m/s, while the same recorded action on the robot meant 0.10 m/s —
so demonstrations would be interpreted as 3.5× faster than they were performed.

**`Z_HOLD` was 0.045 and briefly matched sim exactly** — the block's mid-height,
given a slab at 0.020 and a 50 mm block. It was raised to **0.050** on
2026-08-20 because the tool touched the slab near singular configurations, where
Servo's velocity scaling degrades the height hold. The pusher now rides 5 mm
above block mid-height, still well inside the block's 0.020–0.070 span, with
15 mm of sphere clearance instead of 10 mm.

### Already matching — do not change

| | value | |
| --- | --- | --- |
| `GOAL_QUAT_EE` | `[0.0, 0.7071, 0.7071, 0.0]` | 0.50° from the measured real tool-down orientation |
| `CONTROL_FREQ` | `20` | |
| block geometry | stem ±0.05 x, ±0.025 y; crossbar at x=0.075, ±0.06 y; half-thickness 0.025 | = 150 × 120 × 50 mm, confirmed |
| block initial `qpos[2]` | `0.045` | correct once the ground is raised |
| pusher model | box 0.02 × 0.02 × 0.125, 0.015 sphere | the URDF was built to mirror this |

---

## 3. Why `block_marker_to_object_*` exists

Not a sim change, but you need it to interpret recorded data.

Motive's `objectPushT` frame is **not** the sim's block frame:

- It is **Y-up**; sim's canonical block frame is **Z-up** with **+X along the
  stem toward the crossbar**.
- Its origin sits **~32 mm toward the crossbar** from the stem-bar centre that
  sim uses as the block origin, and **33.8 mm above** mid-height.

`observation_node` applies the correction, so the recorded observation is
already in sim's convention. Just don't feed raw mocap into anything expecting
sim coordinates.

A trap worth knowing: this transform was once set to identity and appeared
"validated" because placing the block at the goal gave a near-identity relative
pose. That is true for *any* frame convention, so it proves nothing.

---

## 4. Known remaining mismatches

Be honest about these when interpreting results.

1. **Joint velocity limits.** `servo_ik_node` clips to **30%** of the actuator
   limits as a hardware precaution; sim uses 100%. Either raise
   `joint_velocity_scale` or clip sim to match, or joint dynamics differ.
2. **Redundancy resolution.** Sim resolves it with a null-space pull toward
   `QHOME`; the real side uses `servo_ik_node`, which ports the same term — but
   with a 6-DOF task a 7-DOF arm has only **one** free DOF, so this only pins
   that single dimension. The other six follow from where the tool is. Matching
   `QHOME` and the goal position (§2) is what makes them agree.
3. **Goal orientation.** Sim uses identity; the real goal carries a 4.4° yaw and
   1.9° tilt. Small, but you can encode the yaw in the goal body's `quat`.
4. **Workspace bounds.** The real `safety_node` clamps to `workspace_x/y/z`;
   sim has no such clamp (only a 1 m block-divergence guard). If a demonstration
   hits a bound, `executed_action` is zeroed while the operator is still
   pushing — self-consistent, but unnatural. Keep the bounds off the working
   region. They are also **not yet the measured-safe envelope**
   (`NEED_TO_CONFIGURE.md` step 6 is still owed).
5. **Servo velocity scaling.** Measured through the older twist path, the
   achieved speed was ~25% below the commanded value. Re-measure under
   joint-jog control and set `MAX_SPEED` to what is actually achieved rather
   than what is requested.

---

## 5. Re-recording the block start pose

If the block's start placement changes, update `BLOCK_POS` / `BLOCK_ANGLE`.
The quickest check needs no transforms — with the pipeline running:

```bash
ros2 topic echo /pusht/observation --once     # indices 0:3 and 3:7
```

At this placement the block sits 18.0 cm from the goal, rotated 92.7°, with
`block_pos_rel_goal = [0.1230, -0.1309, 0.0012]` — the near-zero third
component being the check that the Z-up convention is right (§6.2).
The reference start pose is recorded in `README.md` ("T-block episode start
pose"), both as `block_pos_rel_goal` / `block_quat_rel_goal` and as the
canonical pose in `fr3_link0` with the sim equivalents.

---

## 6. Verifying the alignment

Worth doing before trusting a training run:

1. **Block at the goal ⇒ identity.** Place the block at the goal;
   `block_quat_rel_goal` must be `[1, 0, 0, 0]` and `block_pos_rel_goal` ≈ 0.
   If not, `fixed_goal_quaternion_wxyz` and `block_marker_to_object_*` have
   drifted out of sync — they must satisfy
   `fixed_goal_quaternion = q_marker_at_goal ⊗ q_block_marker_to_object`.
2. **Vertical is index 2.** With the block anywhere flat on the table,
   `block_pos_rel_goal[2]` should be ≈ 0 (measured: 1.2 mm). If the *middle*
   component is the small one instead, the Y-up/Z-up convention is wrong.
3. **Heights agree.** Sim and real should both put the block's centre at
   `z = 0.045` and the pusher at `0.045`.
4. **Action means the same thing.** Command a known action on both and compare
   the achieved tool speed. This is what catches a `MAX_SPEED` mismatch.
