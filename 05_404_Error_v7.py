"""
Friendly starter template for the QArm simulator.

Use this file as your sandbox: edit one section at a time, read the comments,
and try things out as you learn.

Run it:
    python -m demos.blank_sim
"""

from __future__ import annotations

import time
import numpy as np
from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase
from demos._shared import run_with_viewer

# ---------------------------------------------------------------------------
# Quick knobs to tweak the experience
# ---------------------------------------------------------------------------
MODE = "mirror"  # keep "sim" until you have hardware; use "mirror" for hardware+sim
USE_PANDA_VIEWER = True  # Panda3D window that shows the arm; set False for console only
USE_PYBULLET_GUI = False  # Bullet's debug sliders (rarely needed)
TIME_STEP = 1.0 / 240.0
HEADLESS_SECONDS = 10.0  # how long to run when no viewer is open
SHOW_VIEWER_SLIDERS = False
RELOAD_MESHES = False
STEP_DELAY_S = 0.92

# End-Effector targets in `(x, y, z)`.
TARGETS: dict[str, tuple[float, float, float]] = {
    "home": (0.3, -0.15, 0.25),

    "greenB": (0.32, 0.07, 0.01),
    "greenC": (0.2, 0.11, 0.01),
    "greenD" : (0.2, -0.14, 0.01),
    "greenF" : (0.2, -0.26, 0.01),
    "greenI" : (0.28, -0.371, 0.01),
    "greenH" : (-0.06, -0.230, 0.01),
    "greenN" : (0.04, -0.470, 0.01),
    "greenP" : (0.08, -0.570, 0.01),

    "lift_greenB": (0.32, 0.07, 0.245),
    "lift_greenC": (0.2, 0.11, 0.245),
    "lift_greenD" : (0.2, -0.14, 0.245),
    "lift_greenF" : (0.2, -0.26, 0.245),
    "lift_greenI" : (0.28, -0.371, 0.245),
    "lift_greenH" : (-0.06, -0.230, 0.245),
    "lift_greenN" : (0.04, -0.470, 0.245),
    "lift_greenP" : (0.08, -0.570, 0.245),

    "purpleA": (0.20, -0.020, 0.01),
    "purpleE" : (0.32, -0.21, 0.01),
    "purpleG" : (0.09, -0.23, 0.01),
    "purpleM" : (0.180, -0.49, 0.01),
    "purpleJ" : (0.125, -0.37, 0.01),
    "purpleK" : ( 0.0 , -0.34, 0.01),
    "purpleL" : (-0.120, -0.340, 0.01),
    "purpleO" : (-0.08, -0.45, 0.01),

    "lift_purpleA": (0.20, -0.020, 0.245),
    "lift_purpleE" : (0.32, -0.21, 0.245),
    "lift_purpleG" : (0.09, -0.23, 0.245),
    "lift_purpleM" : (0.180, -0.49, 0.245),
    "lift_purpleJ" : (0.125, -0.37, 0.245),
    "lift_purpleK" : ( 0.0 , -0.34, 0.245),
    "lift_purpleL" : (-0.120, -0.340, 0.245),
    "lift_purpleO" : (-0.08, -0.45, 0.245),

    "purple_drop": (0.3, -0.09, 0.245),
    "green_drop": (-0.0925, -0.5735, 0.245),
}

SEQUENCE: list[tuple[str, str]] = [
    ("Return home", "home"),
    ("Open gripper", "open"),
    ("Lift hoop", "lift_greenB"),
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

    ("Lift hoop", "lift_purpleA"),
    ("Pick Green", "purpleA"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleA"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_purpleE"),
    ("Pick Green", "purpleE"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleE"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_purpleG"),
    ("Pick Green", "purpleG"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleG"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_purpleM"),
    ("Pick Green", "purpleM"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleM"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_purpleJ"),
    ("Pick Green", "purpleJ"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleJ"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_purpleK"),
    ("Pick Green", "purpleK"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleK"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_purpleL"),
    ("Pick Green", "purpleL"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleL"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Lift hoop", "lift_purpleO"),
    ("Pick Green", "purpleO"),
    ("Close gripper", "close"),
    ("Lift hoop", "lift_purpleO"),
    ("Drop Hoop", "purple_drop"),
    ("Open gripper", "open"),

    ("Return home", "home"),
]

GRIPPER_OPEN_ANGLE = 0.2
GRIPPER_CLOSED_ANGLE = 0.6

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

    q4 = 0.0
    return (q1, q2, q3, q4)


def go_to(arm: QArmBase, pose_name: str, wait: float) -> None:
    pose = inverse_kinematics(pose_name)
    arm.set_joint_positions(pose)
    time.sleep(wait)


def _launch_viewer(arm: QArmBase) -> None:
    """Open the Panda3D viewer. Beginners: just leave this alone."""
    env = getattr(arm, "env", None)
    if env is None:
        print("[Blank] No simulator attached; viewer cannot start.")
        return

    from sim.panda_viewer import PandaArmViewer, PhysicsBridge

    viewer_args = SimpleNamespace(
        time_step=env.time_step,
        hide_base=False,
        hide_accents=False,
        probe_base_collision=False,
        show_sliders=SHOW_VIEWER_SLIDERS,
        reload_meshes=RELOAD_MESHES,
    )
    bridge = PhysicsBridge(time_step=env.time_step, env=env, reset=False)
    PandaArmViewer(bridge, viewer_args).run()


def _headless_spin(env: object) -> None:
    """Keep physics running without graphics (useful for scripts/tests)."""
    if not hasattr(env, "step") or not hasattr(env, "time_step"):
        print("[Blank] No environment to step. Enable the simulator first.")
        return
    steps = int(HEADLESS_SECONDS / env.time_step)
    print(f"[Blank] Stepping headless for {HEADLESS_SECONDS:.1f}s ({steps} steps)...")
    for _ in range(steps):
        env.step()
        time.sleep(env.time_step)
    print("[Blank] Done stepping; press Ctrl+C to exit or start scripting moves.")


def main() -> None:
    """Entry point. Follow the comments below to add your own code."""
    mode = MODE.lower()
    mirror_mode = mode == "mirror"
    effective_mode = "hardware" if mirror_mode else mode
    auto_step = not USE_PANDA_VIEWER  # viewer updates the sim when it's open
    print(f"[Blank] Connecting to QArm in {mode} mode with joint order {DEFAULT_JOINT_ORDER}")
    arm = make_qarm(
        mode=effective_mode,
        gui=USE_PYBULLET_GUI,
        real_time=False,
        time_step=TIME_STEP,
        auto_step=auto_step,
        mirror_sim=mirror_mode,
    )

    env = getattr(arm, "env", None)
    if env is not None:
        env.reset()  # start from a zeroed pose

    try:
        if USE_PANDA_VIEWER:
            print("[Blank] Viewer opening. Running your script while the window is visible.")
            # Use a worker thread so `student_script` runs at the same time as the viewer.
            # Threads let one part of your program keep working (running commands) while
            # another part (the Panda window) stays responsive on the main thread.
            import threading

            worker = threading.Thread(target=student_script, args=(arm,), daemon=True)
            worker.start()
            _launch_viewer(arm)
            worker.join()
        elif env is not None:
            student_script(arm)
            _headless_spin(env)
        else:
            print("[Blank] No simulator to run; check MODE.")
    except KeyboardInterrupt:
        print("\n[Blank] Stopping minimal sim.")
    finally:
        try:
            arm.home()
        except Exception:
            pass
        disconnect = getattr(arm, "disconnect", None)
        if callable(disconnect):
            try:
                disconnect()
            except Exception:
                pass


def student_script(arm: QArmBase) -> None:
    """
    BEGINNER PLAYGROUND: put your experiments here.

    This function runs after the viewer is on-screen (when enabled), so you can see everything.
    Uncomment one idea at a time or replace them with your own.
    """

    time.sleep(1.0)  # wait a moment for things to settle

    for action, target in SEQUENCE:
        if target == "open":
            arm.set_gripper_position(GRIPPER_OPEN_ANGLE)
        elif target == "close":
            arm.set_gripper_position(GRIPPER_CLOSED_ANGLE)
        else:
            go_to(arm, target, STEP_DELAY_S)
        time.sleep(STEP_DELAY_S)


if __name__ == "__main__":
    main()
