"""Compute the numeric Gradescope checkpoints of PS2 and check the stacking.

    python checkpoints.py            # all of it
    python checkpoints.py stack      # just the stack, with a Meshcat replay

Numbers go to stdout; the stacking replay goes to results/ps2_stack.html.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.append(str(Path(__file__).resolve().parents[1]))

import numpy as np
from pydrake.all import Meshcat, Simulator

import forward_kin as fk
import pick_and_place as pp
import utils.viz

RESULTS = Path(__file__).resolve().parents[2] / "results"
RESULTS.mkdir(exist_ok=True)


def run(system, duration):
    simulator = Simulator(system)
    pp.start_the_integrator(system, simulator.get_mutable_context())
    simulator.AdvanceTo(duration)
    return simulator


def block_poses(system, simulator):
    station = system.GetSubsystemByName("station")
    plant = station.GetSubsystemByName("plant")
    ctx = plant.GetMyContextFromRoot(simulator.get_context())
    out = {}
    for name, *_ in pp.BLOCK_LAYOUT:
        body = plant.GetBodyByName(pp.BOX_BODY, plant.GetModelInstanceByName(name))
        out[name] = plant.EvalBodyPoseInWorld(ctx, body)
    return out


def measured_q(system, simulator):
    station = system.GetSubsystemByName("station")
    return station.GetOutputPort("iiwa.position_measured").Eval(
        station.GetMyContextFromRoot(simulator.get_context())
    )


def planar():
    q = np.array([0.3, 0.2])
    print(f"11.1 FK(0.3, 0.2) = {np.round(fk.forward_kinematics(q), 4)}")
    print(f"12.1 IK(-1, -0.5) = {[np.round(s, 2) for s in fk.inverse_kinematics(-1, -0.5)]}")
    print(f"13.1 J(0.3, 0.2) = {np.round(fk.Jacobian(q), 4).tolist()}")


def grasps():
    X_OG = pp.make_overhead_box_grasp()
    print(f"14.1 p_OG = {np.round(X_OG.translation(), 4)}")
    X_WG = pp.block_pose("blue") @ X_OG
    print(f"14.2 blue p_WG = {np.round(X_WG.translation(), 4)}")
    for name, *_ in pp.BLOCK_LAYOUT:
        print(f"     IK for {name}: {np.round(pp.target_conf_to_pick(name), 3)}")


def drive(name):
    X_WG_target = pp.block_pose(name) @ pp.make_overhead_box_grasp()
    system, duration = pp.build_system_tracking_waypoints([pp.Waypoint(X_WG_target)])
    simulator = run(system, duration + 2.0)
    q = measured_q(system, simulator)
    print(f"15.{4 if name == 'blue' else 6} test3({name!r}): trajectory ends at {duration:.1f} s; "
          f"final q = {np.round(q, 2).tolist()}")
    # Where did the gripper actually get to, against where it was sent?
    station = system.GetSubsystemByName("station")
    plant = station.GetSubsystemByName("plant")
    ctx = plant.GetMyContextFromRoot(simulator.get_context())
    G = plant.GetBodyByName("body", plant.GetModelInstanceByName("wsg")).body_frame()
    X = G.CalcPoseInWorld(ctx)
    err = np.linalg.norm(X.translation() - X_WG_target.translation())
    print(f"     gripper position error {err * 1000:.1f} mm")


def stack(a="red", b="blue", c="yellow", record=False):
    if record:
        utils.viz._meshcat = Meshcat()
        pp.get_meshcat = lambda: utils.viz._meshcat
    waypoints = pp.plan(a, b, c)
    system, duration = pp.build_system_tracking_waypoints(waypoints)
    sim = Simulator(system)
    pp.start_the_integrator(system, sim.get_mutable_context())
    if record:
        utils.viz._meshcat.StartRecording()
    sim.AdvanceTo(duration + 2.0)
    if record:
        utils.viz._meshcat.StopRecording()
        utils.viz._meshcat.PublishRecording()
        (RESULTS / f"ps2_stack_{a}_{b}_{c}.html").write_text(utils.viz._meshcat.StaticHtml())
    poses = block_poses(system, sim)
    Xa, Xb, Xc = poses[a], poses[b], poses[c]
    X_ab = Xa.inverse() @ Xb
    rpy_ab = X_ab.rotation().ToRollPitchYaw().vector()
    up_c = Xc.rotation().matrix()[:, 0]  # c's long axis
    centre_c = Xc @ np.array([0, 0, pp.BLOCK_SIZE[2] / 2])
    top_b = Xb @ np.array([0, 0, pp.BLOCK_SIZE[2]])
    print(f"16.1 plan({a!r}, {b!r}, {c!r}) has {len(waypoints)} waypoints, {duration:.0f} s")
    print(f"     b in a's frame: p = {np.round(X_ab.translation(), 4)}, rpy (deg) = {np.round(np.degrees(rpy_ab), 1)}")
    print(f"     c's long axis = {np.round(up_c, 3)}, c's center - b's top = {np.round(centre_c - top_b, 4)}")
    ok = (
        np.linalg.norm(X_ab.translation() - [0, 0, pp.BLOCK_SIZE[2]]) < 0.01
        and np.abs(rpy_ab).max() < np.radians(5)
        and up_c[2] > 0.99
        and np.linalg.norm((centre_c - top_b)[:2]) < 0.01
        and abs((centre_c - top_b)[2] - pp.BLOCK_SIZE[0] / 2) < 0.005
    )
    print(f"     stack {'OK' if ok else 'FAILED'}")
    return ok


def fast():
    """16.2: the same plan, 0.5 s per leg."""
    pp.KEYFRAME_SECONDS = 0.5
    try:
        ok = stack()
    except Exception as e:  # noqa: BLE001 -- report whatever the failure is
        print(f"16.2 raised {type(e).__name__}: {e}")
        ok = False
    finally:
        pp.KEYFRAME_SECONDS = 4.0
    print(f"16.2 with 0.5 s legs the stack {'still works' if ok else 'fails'}")


if __name__ == "__main__":
    which = sys.argv[1:] or ["planar", "grasps", "drive", "stack", "orders", "fast"]
    if "planar" in which:
        planar()
    if "grasps" in which:
        grasps()
    if "drive" in which:
        drive("blue")
        drive("red")
    if "stack" in which:
        stack(record=True)
    if "orders" in which:
        import itertools

        results = {abc: stack(*abc) for abc in itertools.permutations(["red", "blue", "yellow"])}
        print(f"all orders: {sum(results.values())} / {len(results)} stacks OK")
    if "fast" in which:
        fast()
