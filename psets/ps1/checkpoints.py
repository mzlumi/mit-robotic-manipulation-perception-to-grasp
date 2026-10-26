"""Compute every numeric Gradescope checkpoint of PS1 and save the plots.

Run from this directory:

    HEADLESS=1 python checkpoints.py

Numbers go to stdout (and results/ps1_checkpoints.txt), figures to figures/.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.append(str(Path(__file__).resolve().parents[1]))

import numpy as np

import cartesian_2d_robot as crb
import iiwa_1
from dtsystems import (
    Body1D,
    Constant,
    PController,
    feedback_composition,
    get_positions,
    get_state,
    series_composition,
    simulate,
)
from utils.plotting import plt

HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)


def find_log(diagram, simulator, name):
    for s in diagram.GetSystems():
        if s.get_name() == name:
            return s.FindLog(simulator.get_context())
    raise KeyError(name)


def settle_time(t, q, v, target, pos_tol, vel_tol):
    """First time after which |q - target| < pos_tol and |v| < vel_tol for good."""
    ok = (np.max(np.abs(q - target[:, None]), axis=0) < pos_tol) & (
        np.max(np.abs(v), axis=0) < vel_tol
    )
    bad = np.nonzero(~ok)[0]
    if len(bad) == 0:
        return t[0]
    if bad[-1] == len(t) - 1:
        return np.inf
    return t[bad[-1] + 1]


def p7():
    print("Problem 7")
    body = Body1D(mass=2, dt=0.1)
    s1 = body.next_state_fun(np.array([0.0, 0.5]), np.array([0.3]))
    print(f"  7.1 one step: position {s1[0]:.4f}, velocity {s1[1]:.4f}")

    diagram = series_composition(Constant([0.1]), Body1D(mass=2, dt=0.1))
    body = diagram.GetSystems()[1]
    sim = simulate(diagram, [[0.0, 0.0]], T=0.5)
    s = get_state(body, sim)
    print(f"  7.2 after 5 steps of 0.1 N: position {s[0]:.4f}, velocity {s[1]:.4f}")

    ctrl = PController(1, [1.0], gain=10)
    print(f"  7.3 force at x = 0.2: {ctrl.output_fun(None, np.array([0.2]))[0]:.4f}")

    diagram = feedback_composition(PController(1, [1.0], gain=10), Body1D(mass=2, dt=0.1))
    sim = simulate(diagram, [[0.2, 0.5]], T=10)
    log = find_log(diagram, sim, "log_Body1D")
    x = log.data()[0]
    first, last = np.abs(x[:20] - 1).max(), np.abs(x[-20:] - 1).max()
    print(f"  7.4 |x - 1| first 2 s max {first:.3f}, last 2 s max {last:.3f}")
    plt.figure()
    plt.plot(log.sample_times(), x, ".-")
    plt.axhline(1.0, color="0.5", ls="--")
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.title("7.4 P controller on a 1D body (k = 10, m = 2, dt = 0.1)")
    plt.savefig(FIG / "p7_4_feedback.png", dpi=120)
    plt.close()


def p8():
    print("Problem 8")
    diagram, plant = crb.create_2d_robot_diagram(force=np.array([4.0, 1.0]))
    sim = simulate(diagram, crb.finger_s0(crb.FINGER_START), 5.0)
    v = find_log(diagram, sim, "log_plant").data()[2:, -1]
    print(f"  8.1 final velocity {v} (force / damping = {np.array([4.0, 1.0]) / crb.JOINT_DAMPING})")

    target = np.array([0.9, 0.6])
    diagram, plant = crb.create_2d_robot_diagram_with_controller(100, target)
    sim = simulate(diagram, crb.finger_s0(crb.FINGER_START), 5.0)
    log = find_log(diagram, sim, "log_plant")
    d = log.data()
    for tol in (1e-2, 1e-3):
        ts = settle_time(log.sample_times(), d[:2], d[2:], target, tol, tol)
        print(f"  8.2 at rest within {tol * 1000:.0f} mm and {tol * 1000:.0f} mm/s after {ts:.3f} s")
    plt.figure()
    for i, lab in enumerate(["x", "y", "xdot", "ydot"]):
        plt.plot(log.sample_times(), d[i], label=lab)
    plt.xlabel("time (s)")
    plt.legend()
    plt.title("8.2 finger driven to (0.9, 0.6), gain 100")
    plt.savefig(FIG / "p8_2_setpoint.png", dpi=120)
    plt.close()

    # 8.3 asks for JOINT_DAMPING = 20; it is baked into the SDF text at import.
    crb.finger_sdf = crb.finger_sdf.replace(
        f"<damping>{crb.JOINT_DAMPING}</damping>", "<damping>20.0</damping>"
    )
    assert crb.finger_sdf.count("<damping>20.0</damping>") == 2
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    for ax, eps in zip(axes, (0.1, 0.01)):
        diagram, plant = crb.create_2d_robot_diagram_with_waypoints(100, crb.SQUARE, eps)
        sim = simulate(diagram, crb.finger_s0(crb.SQUARE[0]), 5.0)
        log = find_log(diagram, sim, "log_plant")
        xy = log.data()[:2]
        length = np.sum(np.linalg.norm(np.diff(xy, axis=1), axis=0))
        print(f"  8.3 damping 20, gain 100, epsilon {eps}: path length {length:.4f} m")
        ax.scatter(xy[0], xy[1], c=log.sample_times(), s=3)
        ax.set_title(f"epsilon = {eps}: {length:.3f} m")
        ax.set_aspect("equal")
    fig.suptitle("8.3 finger around SQUARE (damping 20, gain 100, 5 s)")
    fig.savefig(FIG / "p8_3_square.png", dpi=120)
    plt.close(fig)


def p9():
    print("Problem 9")
    for tau in (0.0, 20.0):
        diagram, plant = iiwa_1.create_IIWA14_diagram(torques=np.full(7, tau))
        sim = simulate(diagram, iiwa_1.iiwa_s0(iiwa_1.Q_START), 5.0)
        d = find_log(diagram, sim, "log_plant").data()
        print(f"  9.{1 if tau == 0 else 2} torque {tau}: final q {np.round(d[:7, -1], 2)}, "
              f"final |v| {np.linalg.norm(d[7:, -1]):.2f}, q range {np.round(np.ptp(d[:7], axis=1), 2)}")

    diagram, plant = iiwa_1.create_IIWA14_diagram_with_pcontroller(100, iiwa_1.Q_START)
    sim = simulate(diagram, iiwa_1.iiwa_s0(iiwa_1.Q_START), 10.0)
    log = find_log(diagram, sim, "log_plant")
    d = log.data()
    err = np.abs(d[:7] - iiwa_1.Q_START[:, None]).max(axis=0)
    print(f"  9.3 P gain 100: max |q - q0| in first 2 s {err[log.sample_times() < 2].max():.3f}, "
          f"last 2 s {err[log.sample_times() > 8].max():.3f}; final q {np.round(d[:7, -1], 3)}")
    plt.figure()
    for i in range(7):
        plt.plot(log.sample_times(), d[i] - iiwa_1.Q_START[i], label=f"q{i + 1}")
    plt.xlabel("time (s)")
    plt.ylabel("q - Q_START (rad)")
    plt.legend()
    plt.title("9.3 iiwa under a P controller, gain 100")
    plt.savefig(FIG / "p9_3_pcontrol.png", dpi=120)
    plt.close()

    target = iiwa_1.Q_START + iiwa_1.Q_STEP
    diagram, plant = iiwa_1.create_IIWA14_diagram_with_pd_controller(
        100, 30, target, dt=iiwa_1.PD_DT
    )
    sim = simulate(diagram, iiwa_1.iiwa_s0(iiwa_1.Q_START), 10.0)
    log = find_log(diagram, sim, "log_plant")
    d = log.data()
    t = log.sample_times()
    q_end = d[:7, -1]
    print(f"  9.4 PD (100, 30): final q - target {np.round(q_end - target, 3)}")
    for tol in (1e-2, 1e-3):
        ts = settle_time(t, d[:7], d[7:], q_end, tol, tol)
        print(f"  9.4 at rest within {tol} rad and {tol} rad/s of its final pose after {ts:.3f} s")
    plt.figure()
    for i in range(7):
        plt.plot(t, d[i], label=f"q{i + 1}")
    plt.xlabel("time (s)")
    plt.legend()
    plt.title("9.4 PD step response (kp 100, kd 30)")
    plt.savefig(FIG / "p9_4_pd_step.png", dpi=120)
    plt.close()


if __name__ == "__main__":
    p7()
    p8()
    p9()
