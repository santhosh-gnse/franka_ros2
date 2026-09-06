"""Replay a real franka_bulbscrew episode in the MuJoCo scene.

Real episodes carry no qpos, only the 25-D observation, so the pose is
reconstructed from it:

    arm joints  <- obs[11:18]                   (joint encoders)
    fingers     <- obs[10] / 2 each
    bulb        <- tip = seat + obs[0:3], quat = obs[3:7];
                   body origin = tip - axis_z * TIP_OFF

That makes it an alignment check as well as a video: the arm comes from the
robot and the bulb from mocap, so if the calibration is right the bulb sits
between the fingers whenever the demonstration has it gripped.

Two departures from the shipped visualiser, both because this renders against
the STALE sim_old scene while the real rig has moved on:

  * the socket body is repositioned to the measured seat, otherwise the bulb
    (placed relative to the real seat) floats away from the sim's socket;
  * the holder is hidden -- the real bulb stands free on its screw base.

NOT a ROS entry point -- it needs mujoco, which the package does not depend on.
Run it from the bulbscrew conda environment, outside the ROS workspace:

    source ~/miniconda3/etc/profile.d/conda.sh && conda activate bulbscrew
    MUJOCO_GL=egl python render_episode.py --data <episode.npz> --out out.mp4

MODEL points at a local copy of bulbscrew_mjx's scene; edit it for your layout.
"""
import argparse
from pathlib import Path

import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image, ImageDraw

REPO = Path(__file__).resolve().parents[1]
MODEL = REPO / "sim_old" / "bulbscrew_mjx" / "data" / "scene_mjx_bulb.xml"

TIP_OFF = -0.053                                  # canonical origin -> screw tip
SEAT = np.array([0.6084, 0.0787, 0.0685])         # re-measured 2026-09-06, fr3_link0

# obs[0:3] and obs[3:7] are expressed in the SEAT frame, which on this rig is
# rotated -77.7 deg about z relative to fr3_link0. They are NOT fr3_link0
# coordinates: using them as such rotates the bulb about the socket by that
# angle and puts it roughly 0.4 m from where it really is. This matrix and
# quaternion take the seat frame into fr3_link0.
# Derived as conj(base_q) * fixed_socket_quaternion_wxyz, where base_q is
# fr3_link0's orientation in the mocap map frame from the hand-eye anchor.
Q_L0_SEAT = np.array([0.778551, -0.000876, -0.003338, -0.627571])   # wxyz
R_L0_SEAT = np.array([[ 0.212286,  0.977199, -0.004098],
                      [-0.977187,  0.212307,  0.005554],
                      [ 0.006297,  0.002826,  0.999976]])
SEAT_OFF = np.array([0.0, 0.0, 0.018])            # socket body origin -> seat site
PLANK_TOP = 0.0387                                # measured with the closed gripper
# Current workspace bounds; flagged in the overlay so a demonstration that
# deployment could not reproduce is visible rather than inferred.
WS_LO = np.array([0.35, -0.35, 0.02])
WS_HI = np.array([0.85,  0.30, 0.45])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def axis_z_of(q):
    w, x, y, z = q
    return np.array([2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=REPO / "videos" / "episode_replay.mp4")
    ap.add_argument("--fps", type=int, default=40)          # 2x real time
    ap.add_argument("--stride", type=int, default=1)
    args = ap.parse_args()

    S = np.load(args.data)["states"]
    m = mujoco.MjModel.from_xml_path(MODEL.as_posix())
    m.vis.global_.offwidth, m.vis.global_.offheight = 960, 720
    d = mujoco.MjData(m)

    m.body_pos[m.body("socket").id] = SEAT - SEAT_OFF
    m.body_pos[m.body("holder").id] = [0.0, 0.0, -1.0]      # out of frame
    # The sim board sits at z=0.02 while the real plank top is 0.0389, so the
    # bulb floats ~19 mm above it. Recentre over the real task span -- bulb at
    # x 0.744, seat at 0.599, y from -0.162 to 0.097 -- and set the true height.
    # The EXTENT is a render convenience, not a measurement: the real plank's
    # footprint has not been surveyed.
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "board")
    m.geom_pos[bid] = [0.67, -0.05, PLANK_TOP - m.geom_size[bid][2]]

    jadr = [m.joint(f"fr3_joint{i+1}").qposadr[0] for i in range(7)]
    f1 = m.joint("finger_joint1").qposadr[0]
    f2 = m.joint("finger_joint2").qposadr[0]
    bq = m.joint("bulb_joint").qposadr[0]

    d_seat = np.linalg.norm(S[:, 0:3], axis=1)
    quat = np.where(S[:, 3:4] < 0, -S[:, 3:7], S[:, 3:7])
    task_err = d_seat + 0.1 * 2*np.arccos(np.clip(quat[:, 0], -1.0, 1.0))
    solved = d_seat < 0.020    # slotting: position only, no uprightness
    grasp_err = np.linalg.norm(S[:, 7:10], axis=1)
    first = int(np.argmax(solved)) if solved.any() else None

    r = mujoco.Renderer(m, height=720, width=960)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.64, 0.03, 0.10]
    cam.distance, cam.elevation, cam.azimuth = 0.95, -22, 150

    frames = []
    for k in range(0, len(S), args.stride):
        o = S[k]
        d.qpos[jadr] = o[11:18]
        d.qpos[f1] = d.qpos[f2] = o[10] / 2.0
        q = quat_mul(Q_L0_SEAT, o[3:7] / (np.linalg.norm(o[3:7]) + 1e-9))
        tip = SEAT + R_L0_SEAT @ o[0:3]
        d.qpos[bq:bq+3] = tip - axis_z_of(q) * TIP_OFF
        d.qpos[bq+3:bq+7] = q
        mujoco.mj_forward(m, d)
        r.update_scene(d, camera=cam)
        img = Image.fromarray(r.render())
        dr = ImageDraw.Draw(img)
        dr.text((10, 8), f"step {k:4d}/{len(S)}   t {k/20:5.1f}s", fill=(255, 255, 80))
        dr.text((10, 24), f"task_err {task_err[k]:.4f}   d_seat {d_seat[k]:.4f}", fill=(255, 255, 80))
        dr.text((10, 40), f"gripper {o[10]*1000:5.1f} mm   tool-grasp {grasp_err[k]*1000:5.1f} mm",
                fill=(255, 255, 80))
        tcp = d.xpos[m.body("ee_frame").id]
        if np.any(tcp < WS_LO) or np.any(tcp > WS_HI):
            dr.text((700, 8), "TOOL OUTSIDE WORKSPACE", fill=(255, 120, 120))
        if solved[k]:
            dr.text((10, 56), "SOLVED", fill=(120, 255, 120))
        if first is not None and k == first:
            dr.text((760, 8), "FIRST SOLVE", fill=(120, 255, 120))
        frames.append(np.asarray(img))
    r.close()

    args.out.parent.mkdir(exist_ok=True)
    imageio.mimsave(args.out.as_posix(), frames, fps=args.fps)
    print(f"first solve at step {first}; task_err min {task_err.min():.4f}")
    print(f"tool-grasp residual while gripped: median "
          f"{1000*np.median(grasp_err[S[:,10] < 0.065]):.1f} mm")
    print(f"wrote {args.out} ({len(frames)} frames @ {args.fps} fps)")


if __name__ == "__main__":
    main()
