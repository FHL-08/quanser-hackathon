"""
Scripted pick-and-place in plain Python.

What it shows:
- A small dictionary of poses you can edit without hunting through code.
- A simple sequence that opens/closes the gripper between waypoints.
- Prints every move so you see which joint set is being sent.

Run it (opens Panda3D viewer by default):
    python -m demos.pick_and_place
Tweak `POSES` and `SEQUENCE` below to match where your hoop and stand sit.
"""

from __future__ import annotations

import time
import numpy as np
from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase

MODE = "sim"
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
STEP_DELAY_S = 1.0

# End-Effector targets in `(x, y, z)`.
TARGETS: dict[str, tuple[float, float, float]] = {
    "home": (0.3, -0.15, 0.25),
    "greenB": (0.32, 0.07, 0.0),
    "greenC": (0.2, 0.11, 0.0),
    "greenD" : (0.2, -0.14, 0.0),
    "greenF" : (0.2, -0.26, 0.0),
    "greenI" : (0.28, -0.371, 0.0),
    "greenH" : (-0.06, -0.230, 0.0),
    "greenN" : (0.04, -0.470, 0.0),
    "greenP" : (0.08, -0.570, 0.0),
    "lift_greenB": (0.32, 0.07, 0.24),
    "lift_greenC": (0.2, 0.11, 0.24),
    "lift_greenD" : (0.2, -0.14, 0.24),
    "lift_greenF" : (0.2, -0.26, 0.24),
    "lift_greenI" : (0.28, -0.371, 0.24),
    "lift_greenH" : (-0.06, -0.230, 0.24),
    "lift_greenN" : (0.04, -0.470, 0.24),
    "lift_greenP" : (0.08, -0.570, 0.24),
    "purple_drop": (0.3, -0.09, 0.24),
    "green_drop": (-0.09, -0.570, 0.24),
}

SEQUENCE: list[tuple[str, str]] = [
    ("Return home", "home"),
    ("Open gripper", "open"),
    ("Pick Green", "greenB"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenB"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_greenC"),
    ("Pick Green", "greenC"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenC"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_greenD"),
    ("Pick Green", "greenD"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenD"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_greenF"),
    ("Pick Green", "greenF"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenF"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_greenI"),
    ("Pick Green", "greenI"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenI"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_greenH"),
    ("Pick Green", "greenH"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenH"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_greenN"),
    ("Pick Green", "greenN"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenN"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_greenP"),
    ("Pick Green", "greenP"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_greenP"),
    ("Drop Hoop", "green_drop"),
    ("Open gripper", "open"),

    ("Return home", "home"),
]

GRIPPER_OPEN_ANGLE = 0.0
GRIPPER_CLOSED_ANGLE = 0.85

LINK1 = 0.14
LINK2 = 0.35
LINK3 = 0.05
LAMBDA2 = np.sqrt(LINK2**2 + LINK3**2)
LINK4 = 0.25
LINK5 = 0.15
LAMBDA3 = LINK4 + LINK5
BETA = 0.1419

def inverse_kinematics(target_name: str) -> None:
    target = TARGETS[target_name]
    p_EE = np.array([target[0], target[1], target[2]])
    x_pos = target[0]
    y_pos = target[1]

    q1 = np.arctan2(y_pos, x_pos)
    joint1_pos = np.array([0, 0, LINK1])

    p_2_to_EE = p_EE - joint1_pos
    lambda_4 = np.linalg.norm(p_2_to_EE)
    print((LAMBDA2**2 + LAMBDA3**2 - lambda_4**2)/(2*LAMBDA2*LAMBDA3))
    delta_now = np.arccos((LAMBDA2**2 + LAMBDA3**2 - lambda_4**2)/(2*LAMBDA2*LAMBDA3))
    q3 = np.pi/2 + (BETA - delta_now)

    d = np.linalg.norm(p_2_to_EE[0:2])
    r = np.linalg.norm(p_2_to_EE[0:3])
    psi = np.arctan2(p_2_to_EE[2], d)
    phi = np.arcsin(LAMBDA3*np.sin(delta_now)/r)
    q2 = (np.pi/2 - BETA) - (phi + psi)

    q4 = -np.pi/2
    return (q1, q2, q3, q4)


def go_to(arm: QArmBase, pose_name: str, wait: float) -> None:
    pose = inverse_kinematics(pose_name)
    print(f"[PickPlace] {pose_name}: {pose}")
    arm.set_joint_positions(pose)
    time.sleep(wait)


def run_sequence(arm: QArmBase, *, repeats: int | None = None) -> None:
    print("[PickPlace] Joint order:", ", ".join(DEFAULT_JOINT_ORDER))
    count = 0
    while repeats is None or count < repeats:
        for action, target in SEQUENCE:
            if target == "open":
                print("[PickPlace] Opening gripper")
                arm.set_gripper_position(GRIPPER_OPEN_ANGLE)
            elif target == "close":
                print("[PickPlace] Closing gripper")
                arm.set_gripper_position(GRIPPER_CLOSED_ANGLE)
            else:
                go_to(arm, target, STEP_DELAY_S)
            time.sleep(STEP_DELAY_S)
        count += 1


def main() -> None:
    auto_step = not USE_PANDA_VIEWER
    mode = MODE.lower()
    mirror_mode = mode == "mirror"
    effective_mode = "hardware" if mirror_mode else mode
    arm = make_qarm(
        mode=effective_mode,
        gui=USE_PYBULLET_GUI,
        real_time=False,
        auto_step=auto_step,
        mirror_sim=mirror_mode,
    )
    mirror_status = "on" if mirror_mode else "off"
    print(f"[PickPlace] Connected (mode={mode}, mirror={mirror_status}); moving to home.")
    arm.home()
    time.sleep(STEP_DELAY_S)

    try:
        if USE_PANDA_VIEWER:
            import threading
            from sim.panda_viewer import PandaArmViewer, PhysicsBridge

            def launch_viewer() -> None:
                env = getattr(arm, "env", None)
                if env is None:
                    print("[PickPlace] Viewer unavailable (no sim env).")
                    return
                args = SimpleNamespace(
                    time_step=env.time_step,
                    hide_base=False,
                    hide_accents=False,
                    probe_base_collision=False,
                    show_sliders=False,
                    reload_meshes=False,
                )
                bridge = PhysicsBridge(time_step=env.time_step, env=env, reset=False)
                PandaArmViewer(bridge, args).run()

            motion = threading.Thread(target=run_sequence, args=(arm,), kwargs={"repeats": 1}, daemon=True)
            motion.start()
            launch_viewer()  # blocks until window closes
        else:
            run_sequence(arm, repeats=None)
    except KeyboardInterrupt:
        print("\n[PickPlace] Sequence interrupted.")
    finally:
        try:
            print("[PickPlace] Returning to home...")
            arm.home()
        except Exception:
            pass


if __name__ == "__main__":
    main()
