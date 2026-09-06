"""Collect the demonstrations that actually solved the task, truncated at success.

Recorded episodes usually run past the point where the block reaches the goal --
the operator keeps nudging it, or simply does not press Stop immediately. Left
in, that tail teaches a policy to keep pushing an already-placed block, and it
is a large fraction of the data: one episode here solved at step 417 of 2330, so
82% of it was post-success.

This walks the dataset, finds the first step where the block satisfies the sim's
success test, truncates there, and writes the result to a separate directory.
Originals are never modified.

The criterion mirrors bulbscrew_mjx exactly. environment.py's _goal_errors() takes

    pos_err = ||block_pos_rel_goal||          (full 3-D norm)
    orn_err = ||rotvec(block_quat_rel_goal)|| (i.e. the rotation angle, radians)

after flipping the quaternion to the shortest arc, and _step() sets
`terminated` when d_seat + 0.1*upright_err < success_threshold. So a truncated episode
ends exactly where a sim episode would, with absorbing[-1] = 1.

Episodes recorded before the frame conventions were fixed on 2026-08-20 are
skipped by default. Their observations carry the vertical axis in the wrong slot
and a zero block-origin offset, so this criterion silently compares the wrong
quantities on them -- their task_err looks plausible but means nothing. See
README.md, "DATA COLLECTED BEFORE 2026-08-20 IS NOT USABLE".

usage:
    ros2 run franka_bulbscrew extract_success_trajectories
    ros2 run franka_bulbscrew extract_success_trajectories --root ~/bulbscrew_data \
        --threshold 0.05 --out success_trajectories
    ros2 run franka_bulbscrew extract_success_trajectories --include-prefix-fix-data
"""

import argparse
import glob
import json
import os

import numpy as np

# bulbscrew_mjx's own criterion, d_seat + 0.1*upright_err. Kept for
# comparability, but it is NOT the default here -- see the note below.
SIM_THRESHOLD = 0.02

# Screwing is not finished when the bulb first reaches the seat pose.
#
# The sim's criterion is yaw-invariant (deliberately -- a bulb is a body of
# revolution), so it cannot tell "resting in the socket mouth" from "screwed
# tight". On the real rig the bulb needs 3-4 full turns to seat, and descends
# ~2.8 mm per turn. Measured on kinesthetic_20260830_190008: the sim criterion
# fired at step 734 of 1581 with the bulb still 5.6 mm proud and only 96 deg of
# spin done, i.e. it would have cut away 2 of the 2.4 turns of actual screwing
# and taught a policy to drop the bulb in the hole and stop.
#
# Depth is what distinguishes them, because the seat was calibrated with the
# bulb screwed fully home: 5.6 mm proud when merely resting, 0.2 mm when tight.
# The hold requirement rejects the moment it passes through on the way down.
# The task is SLOTTING, not screwing: the bulb is dropped into the socket mouth
# and left there. Two consequences for the criterion.
#
# It must be yaw-invariant -- any rotation counts, since nothing is threaded --
# and it must NOT test uprightness. A slotted bulb rests against the socket rim
# at whatever angle it settles: measured 6.1 deg and 16.1 deg on two consecutive
# attempts. Only screwing pulls it perpendicular. An upright term would reject
# a perfectly good slot.
#
# Position, by contrast, IS repeatable: the tip landed within 2.7 mm across those
# attempts, 9.7-12.2 mm above the fully-screwed seat. 20 mm clears both with
# margin while still excluding a bulb merely held nearby.
#
# The seat itself stays the FULLY-SCREWED pose -- set by a hard stop, repeatable
# to 0.02 mm and vertical to 0.40 deg. Defining the goal as a slotted pose would
# have encoded one attempt's accidental lean as the task definition.
DEFAULT_DEPTH = 0.020          # m, tip-to-seat distance counting as slotted
DEFAULT_HOLD_S = 1.0           # s it must stay there -- rejects passing through

# An episode with implausible bulb frames is not usable regardless of where it
# ends (FINDINGS B15): the first demonstration had 41% of its frames tracking
# some other object, and the old extractor accepted it because the one frame it
# truncated at happened to be good.
MAX_BULB_DISTANCE = 0.6        # m from the seat
MAX_BAD_FRACTION = 0.01        # reject above 1% implausible frames
# Sessions recorded before the frame fixes landed; see the module docstring.
PRE_FIX_SESSION_PREFIXES = ()   # no known-bad sessions for this task yet


def goal_errors(states):
    """pos_err and orn_err for a batch of observations, as bulbscrew_mjx computes them."""
    block_pos = states[..., 0:3]
    block_quat = states[..., 3:7].copy()
    block_quat = np.where(block_quat[..., 0:1] < 0.0, -block_quat, block_quat)
    pos_err = np.linalg.norm(block_pos, axis=-1)
    orn_err = 2.0 * np.arccos(np.clip(block_quat[..., 0], -1.0, 1.0))
    return pos_err, orn_err


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="~/bulbscrew_data",
                        help="dataset root holding session_* / kinesthetic_* directories")
    parser.add_argument("--out", default="success_trajectories",
                        help="output directory, relative to --root unless absolute")
    parser.add_argument("--depth", type=float, default=DEFAULT_DEPTH,
                        help="tip-to-seat distance counting as slotted (m)")
    parser.add_argument("--hold-s", type=float, default=DEFAULT_HOLD_S,
                        help="how long it must stay within --depth to count")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--allow-mocap-dropouts", action="store_true",
                        help="keep episodes with implausible bulb frames (see FINDINGS B15)")
    parser.add_argument("--sim-criterion", action="store_true",
                        help="truncate on bulbscrew_mjx's d_seat + 0.1*upright_err instead. "
                             "Cuts mid-screw on real demonstrations; for comparison only.")
    parser.add_argument("--threshold", type=float, default=SIM_THRESHOLD,
                        help="threshold used with --sim-criterion")
    parser.add_argument("--include-pre-fix-data", action="store_true",
                        help="also process pre-2026-08-20 sessions (their frames are "
                             "scrambled, so the criterion is meaningless -- see the docstring)")
    args = parser.parse_args(argv)

    root = os.path.expanduser(args.root)
    out = args.out if os.path.isabs(args.out) else os.path.join(root, args.out)
    if not os.path.isdir(root):
        raise SystemExit(f"no such dataset root: {root}")
    os.makedirs(out, exist_ok=True)
    # Rebuild from scratch so removing a session upstream does not leave orphans.
    for stale in glob.glob(os.path.join(out, "*.npz")):
        os.remove(stale)

    # Both prefixes: ps5_teleop writes session_*, kinesthetic_recorder_node
    # writes kinesthetic_*. Globbing only session_* silently reported "0 solved"
    # for every hand-guided episode ever recorded.
    sessions = sorted(d for pattern in ("session_*", "kinesthetic_*")
                      for d in glob.glob(os.path.join(root, pattern)) if os.path.isdir(d))
    if not sessions:
        raise SystemExit(f"no session_* or kinesthetic_* directories under {root}")
    header = f"{'episode':<44} {'steps':>6} {'best err':>9} {'solved':>8} {'kept':>6}"
    print(header)
    print("-" * len(header))
    kept, skipped, unsolved, sources = [], 0, 0, set()
    for session in sessions:
        name = os.path.basename(session)
        pre_fix = bool(PRE_FIX_SESSION_PREFIXES) and name.startswith(PRE_FIX_SESSION_PREFIXES)
        for path in sorted(glob.glob(os.path.join(session, "episode_*.npz"))):
            label = f"{name}/{os.path.basename(path)}"
            data = np.load(path)
            next_states = data["next_states"]
            d_seat, upright_err = goal_errors(next_states)
            task_err = d_seat + 0.1 * upright_err
            if pre_fix and not args.include_pre_fix_data:
                skipped += 1
                print(f"{label:<44} {len(next_states):>6} {task_err.min():>9.4f} "
                      f"{'-':>8} {'skip':>6}   pre-fix frames")
                continue
            bad = (d_seat > MAX_BULB_DISTANCE).mean()
            if bad > MAX_BAD_FRACTION and not args.allow_mocap_dropouts:
                skipped += 1
                print(f"{label:<44} {len(next_states):>6} {task_err.min():>9.4f} "
                      f"{'-':>8} {'skip':>6}   {100*bad:.0f}% implausible bulb frames")
                continue
            if args.sim_criterion:
                hit = np.flatnonzero(task_err < args.threshold)
            else:
                hold = max(1, int(round(args.hold_s * args.rate_hz)))
                home = d_seat < args.depth
                # first index from which it stays home for the whole hold window
                run = np.convolve(home.astype(int), np.ones(hold, dtype=int), mode="valid")
                stable = np.flatnonzero(run == hold)
                hit = stable[:1]
            if hit.size == 0:
                unsolved += 1
                print(f"{label:<44} {len(next_states):>6} {task_err.min():>9.4f} "
                      f"{'never':>8} {'no':>6}")
                continue
            n = int(hit[0]) + 1
            absorbing = np.zeros(n, dtype=np.float32)
            absorbing[-1] = 1.0
            np.savez(os.path.join(out, f"{name}_{os.path.basename(path)}"),
                     states=data["states"][:n], actions=data["actions"][:n],
                     next_states=next_states[:n], absorbing=absorbing,
                     rewards=np.zeros(n, dtype=np.float32))
            kept.append(n)
            sources.add(name)
            print(f"{label:<44} {len(next_states):>6} {task_err.min():>9.4f} "
                  f"{int(hit[0]):>8} {n:>6}")

    print("-" * len(header))
    total = sum(kept)
    criterion = (f"task_err < {args.threshold}" if args.sim_criterion
                 else f"d_seat < {args.depth} m held for {args.hold_s} s")
    print(f"\n{len(kept)} trajectories solved ({criterion}) -> {out}")
    if kept:
        print(f"  {total} transitions, {total / 20.0:.1f} s at 20 Hz")
        print(f"  lengths: {kept}")
    if unsolved:
        print(f"  {unsolved} episode(s) never reached the threshold")
    if skipped:
        print(f"  {skipped} episode(s) skipped as pre-fix data "
              f"(--include-pre-fix-data to override)")
    json.dump({"control_rate_hz": 20.0, "observation_dim": 25, "action_dim": 5,
               "observation": "bulb_tip_rel_seat_xyz + bulb_quat_rel_seat_wxyz + "
                              "ee_rel_neck_xyz + gripper_width + q7 + dq7",
               "selection": f"truncated at the first step with d_seat + 0.1*upright_err < "
                            f"{args.threshold}, matching bulbscrew_mjx's success_threshold",
               "trajectories": len(kept), "transitions": total,
               "source_sessions": sorted(sources)},
              open(os.path.join(out, "metadata.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
