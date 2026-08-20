"""Collect the demonstrations that actually solved the task, truncated at success.

Recorded episodes usually run past the point where the block reaches the goal --
the operator keeps nudging it, or simply does not press Stop immediately. Left
in, that tail teaches a policy to keep pushing an already-placed block, and it
is a large fraction of the data: one episode here solved at step 417 of 2330, so
82% of it was post-success.

This walks the dataset, finds the first step where the block satisfies the sim's
success test, truncates there, and writes the result to a separate directory.
Originals are never modified.

The criterion mirrors pusht_mjx exactly. environment.py's _goal_errors() takes

    pos_err = ||block_pos_rel_goal||          (full 3-D norm)
    orn_err = ||rotvec(block_quat_rel_goal)|| (i.e. the rotation angle, radians)

after flipping the quaternion to the shortest arc, and _step() sets
`terminated` when pos_err + orn_err < success_threshold. So a truncated episode
ends exactly where a sim episode would, with absorbing[-1] = 1.

Episodes recorded before the frame conventions were fixed on 2026-08-20 are
skipped by default. Their observations carry the vertical axis in the wrong slot
and a zero block-origin offset, so this criterion silently compares the wrong
quantities on them -- their task_err looks plausible but means nothing. See
README.md, "DATA COLLECTED BEFORE 2026-08-20 IS NOT USABLE".

usage:
    ros2 run franka_pusht extract_success_trajectories
    ros2 run franka_pusht extract_success_trajectories --root ~/pusht_data \
        --threshold 0.05 --out success_trajectories
    ros2 run franka_pusht extract_success_trajectories --include-prefix-fix-data
"""

import argparse
import glob
import json
import os

import numpy as np

# pusht_mjx PushT.success_threshold
DEFAULT_THRESHOLD = 0.05
# Sessions recorded before the frame fixes landed; see the module docstring.
PRE_FIX_SESSION_PREFIXES = ("session_20260818", "session_20260819")


def goal_errors(states):
    """pos_err and orn_err for a batch of observations, as pusht_mjx computes them."""
    block_pos = states[..., 0:3]
    block_quat = states[..., 3:7].copy()
    block_quat = np.where(block_quat[..., 0:1] < 0.0, -block_quat, block_quat)
    pos_err = np.linalg.norm(block_pos, axis=-1)
    orn_err = 2.0 * np.arccos(np.clip(block_quat[..., 0], -1.0, 1.0))
    return pos_err, orn_err


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="~/pusht_data",
                        help="dataset root holding session_* directories")
    parser.add_argument("--out", default="success_trajectories",
                        help="output directory, relative to --root unless absolute")
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD,
                        help="success threshold on pos_err + orn_err (sim default 0.05)")
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

    sessions = sorted(d for d in glob.glob(os.path.join(root, "session_*")) if os.path.isdir(d))
    header = f"{'episode':<44} {'steps':>6} {'best err':>9} {'solved':>8} {'kept':>6}"
    print(header)
    print("-" * len(header))
    kept, skipped, unsolved, sources = [], 0, 0, set()
    for session in sessions:
        name = os.path.basename(session)
        pre_fix = name.startswith(PRE_FIX_SESSION_PREFIXES)
        for path in sorted(glob.glob(os.path.join(session, "episode_*.npz"))):
            label = f"{name}/{os.path.basename(path)}"
            data = np.load(path)
            next_states = data["next_states"]
            pos_err, orn_err = goal_errors(next_states)
            task_err = pos_err + orn_err
            if pre_fix and not args.include_pre_fix_data:
                skipped += 1
                print(f"{label:<44} {len(next_states):>6} {task_err.min():>9.4f} "
                      f"{'-':>8} {'skip':>6}   pre-fix frames")
                continue
            hit = np.flatnonzero(task_err < args.threshold)
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
    print(f"\n{len(kept)} trajectories solved (task_err < {args.threshold}) -> {out}")
    if kept:
        print(f"  {total} transitions, {total / 20.0:.1f} s at 20 Hz")
        print(f"  lengths: {kept}")
    if unsolved:
        print(f"  {unsolved} episode(s) never reached the threshold")
    if skipped:
        print(f"  {skipped} episode(s) skipped as pre-fix data "
              f"(--include-pre-fix-data to override)")
    json.dump({"control_rate_hz": 20.0, "observation_dim": 24, "action_dim": 2,
               "observation": "block_rel_goal_xyz + block_rel_goal_wxyz + "
                              "tcp_rel_goal_xyz + q7 + dq7",
               "selection": f"truncated at the first step with pos_err + orn_err < "
                            f"{args.threshold}, matching pusht_mjx's success_threshold",
               "trajectories": len(kept), "transitions": total,
               "source_sessions": sorted(sources)},
              open(os.path.join(out, "metadata.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
