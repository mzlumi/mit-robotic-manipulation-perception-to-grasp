"""PS2 problems for 6.4212: 17 (trajectory speed), 18 (smooth operation) and
19 (the robot painter).

    python extras_64212.py

All three drive the same Jacobian pseudo-inverse controller as test3/test4,
with the blocks left on the table untouched.  Figures go to figures/.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.append(str(Path(__file__).resolve().parents[1]))

import numpy as np
from pydrake.all import RigidTransform, RotationMatrix, Simulator

import pick_and_place as pp
from utils.plotting import plt

FIG = Path(__file__).resolve().parent / "figures"
FIG.mkdir(exist_ok=True)

# Shortest leg we allow, so that a waypoint that only changes the fingers
# still gets time for them to move.
MIN_LEG = 1.0


def speed_waypoint_times(stops, speed: float) -> list[float]:
    """Problem 17: time each leg so the gripper moves at `speed` throughout.

    Speed is measured with pose_distance: translation in meters plus rotation
    scaled by GRIPPER_RADIUS, the distance a point on the gripper's rim moves
    for a radian of turn.  A leg that does not move the gripper (a finger
    change) gets MIN_LEG.
    """
    times = [0.0]
    for a, b in zip(stops[:-1], stops[1:]):
        d = pp.pose_distance(pp.pose_delta(a.X_WG, b.X_WG))
        times.append(times[-1] + max(d / speed, MIN_LEG if d < 1e-9 else 0.0))
    return times


def simulate(waypoints, speed, sample_dt=0.01):
    """Run the controller and log the commanded and actual gripper poses."""
    system, duration = pp.build_system_tracking_waypoints(waypoints, speed)
    station = system.GetSubsystemByName("station")
    plant = station.GetSubsystemByName("plant")
    G = plant.GetBodyByName("body", plant.GetModelInstanceByName("wsg")).body_frame()
    sim = Simulator(system)
    pp.start_the_integrator(system, sim.get_mutable_context())
    ctx = plant.GetMyContextFromRoot(sim.get_context())
    t_log, p_log, R_log = [], [], []
    for t in np.arange(0, duration + 1.0, sample_dt):
        sim.AdvanceTo(t)
        X = G.CalcPoseInWorld(ctx)
        t_log.append(t)
        p_log.append(X.translation())
        R_log.append(X.rotation())
    return np.array(t_log), np.array(p_log), R_log, duration


def gripper_start():
    plant = pp.MakeMultibodyPlant(pp.LoadScenario(data=pp.ROBOT))
    G = plant.GetBodyByName("body", plant.GetModelInstanceByName("wsg")).body_frame()
    return G.CalcPoseInWorld(plant.CreateDefaultContext())


def p17():
    """Constant speed through a move that translates and turns."""
    X0 = gripper_start()
    waypoints = [
        pp.Waypoint(RigidTransform(X0.rotation(), X0.translation() + [0.0, 0.2, -0.1])),
        pp.Waypoint(
            RigidTransform(
                RotationMatrix.MakeZRotation(np.pi / 2) @ X0.rotation(),
                X0.translation() + [0.0, 0.2, -0.1],
            )
        ),
        pp.Waypoint(RigidTransform(X0.rotation(), X0.translation() + [0.1, -0.1, 0.0])),
    ]
    speed = 0.05
    pp.waypoint_times = speed_waypoint_times
    try:
        t, p, R, duration = simulate(waypoints, speed)
    finally:
        pp.waypoint_times = lambda stops, speed: [pp.KEYFRAME_SECONDS * i for i in range(len(stops))]
    # The same speed measure as the timing, from the gripper's actual motion.
    rates = []
    for i in range(1, len(t)):
        d = pp.pose_distance(
            pp.pose_delta(RigidTransform(R[i - 1], p[i - 1]), RigidTransform(R[i], p[i]))
        )
        rates.append(d / (t[i] - t[i - 1]))
    rates = np.array(rates)
    moving = rates > 0.2 * speed
    print(f"17 speed {speed} m/s: plan {duration:.1f} s; while moving, measured speed "
          f"median {np.median(rates[moving]):.4f}, 10-90% [{np.percentile(rates[moving], 10):.4f}, "
          f"{np.percentile(rates[moving], 90):.4f}] m/s")
    plt.figure(figsize=(7, 3.5))
    plt.plot(t[1:], rates)
    plt.axhline(speed, color="0.5", ls="--", label="requested")
    plt.xlabel("time (s)")
    plt.ylabel("gripper speed (m/s, rotation x 0.1 m)")
    plt.title("17: constant-speed timing (translate, turn 90 deg in place, translate)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(FIG / "p17_constant_speed.png", dpi=120)
    plt.close()


def square_waypoints(side=0.2):
    X0 = gripper_start()
    c = X0.translation() + [0.05, 0.0, -0.05]
    h = side / 2
    corners = [c + [0, -h, -h], c + [0, h, -h], c + [0, h, h], c + [0, -h, h], c + [0, -h, -h]]
    return [pp.Waypoint(RigidTransform(X0.rotation(), q)) for q in corners], corners


def deviation_from_polyline(p, corners):
    best = np.full(len(p), np.inf)
    for a, b in zip(corners[:-1], corners[1:]):
        u = np.clip(((p - a) @ (b - a)) / ((b - a) @ (b - a)), 0, 1)
        best = np.minimum(best, np.linalg.norm(p - (a + u[:, None] * (b - a)), axis=1))
    return best


def p18():
    """Velocity profile at waypoints, and the speed-accuracy trade-off."""
    waypoints, corners = square_waypoints()
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    print("18 square of side 0.2 m: seconds per leg, max / RMS deviation from the square (mm)")
    for leg in [4.0, 1.0, 0.5, 0.25]:
        pp.KEYFRAME_SECONDS = leg
        try:
            t, p, _, duration = simulate(waypoints, 0.05, sample_dt=min(0.01, leg / 40))
        finally:
            pp.KEYFRAME_SECONDS = 4.0
        dev = deviation_from_polyline(p, corners)
        sel = (t > leg) & (t < duration)
        print(f"   {leg:4.2f}  {dev[sel].max() * 1000:6.1f} / {np.sqrt(np.mean(dev[sel] ** 2)) * 1000:5.1f}")
        speed = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.diff(t)
        axes[0].plot(t[1:] / leg, speed * leg, label=f"{leg} s per leg")
        axes[1].plot(p[:, 1], p[:, 2], label=f"{leg} s per leg")
    axes[0].set_xlabel("time / leg duration")
    axes[0].set_ylabel("speed x leg duration (m)")
    axes[0].set_title("velocity magnitude: a rectangle per leg")
    axes[0].legend(fontsize=8)
    sq = np.array(corners)
    axes[1].plot(sq[:, 1], sq[:, 2], "k--", lw=1)
    axes[1].set_aspect("equal")
    axes[1].set_xlabel("y (m)")
    axes[1].set_ylabel("z (m)")
    axes[1].set_title("gripper path in the square's plane")
    fig.tight_layout()
    fig.savefig(FIG / "p18_square_speed_accuracy.png", dpi=120)
    plt.close(fig)


def p19(radius=0.1, n=36, seconds_per_leg=0.5):
    """The painter: drive the gripper around a horizontal circle."""
    X0 = gripper_start()
    center = X0.translation() + [0.1, 0.0, -0.1]
    ring = [center + radius * np.array([np.cos(a), np.sin(a), 0.0]) for a in np.linspace(0, 2 * np.pi, n + 1)]
    waypoints = [pp.Waypoint(RigidTransform(X0.rotation(), ring[0]))] + [
        pp.Waypoint(RigidTransform(X0.rotation(), q)) for q in ring
    ]
    pp.KEYFRAME_SECONDS = seconds_per_leg
    try:
        t, p, _, duration = simulate(waypoints, 0.05)
    finally:
        pp.KEYFRAME_SECONDS = 4.0
    start = 2 * seconds_per_leg
    sel = (t >= start) & (t <= duration)
    r = np.linalg.norm(p[sel, :2] - center[:2], axis=1)
    print(f"19 circle r = {radius} m, {n} chords, {seconds_per_leg} s each: radius error max "
          f"{np.abs(r - radius).max() * 1000:.1f} mm (chord sag alone {radius * (1 - np.cos(np.pi / n)) * 1000:.1f} mm), "
          f"height error max {np.abs(p[sel, 2] - center[2]).max() * 1000:.1f} mm")
    plt.figure(figsize=(4.5, 4.5))
    plt.plot(p[sel, 0], p[sel, 1], label="gripper")
    th = np.linspace(0, 2 * np.pi, 200)
    plt.plot(center[0] + radius * np.cos(th), center[1] + radius * np.sin(th), "k--", lw=1, label="circle")
    plt.gca().set_aspect("equal")
    plt.legend()
    plt.title("19: robot painter")
    plt.tight_layout()
    plt.savefig(FIG / "p19_painter.png", dpi=120)
    plt.close()


if __name__ == "__main__":
    which = sys.argv[1:] or ["p17", "p18", "p19"]
    if "p17" in which:
        p17()
    if "p18" in which:
        p18()
    if "p19" in which:
        p19()
