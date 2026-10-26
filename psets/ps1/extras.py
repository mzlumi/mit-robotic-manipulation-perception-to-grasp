"""PS1 9.5 (iiwa draws a square) and the 6.4212 problems 10 (step sizes) and
11 (PD damping).

    python extras.py            # everything
    python extras.py square     # just 9.5, also saves a Meshcat replay

Figures go to figures/, the replay to results/ps1_square_iiwa.html.
"""

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.append(str(Path(__file__).resolve().parents[1]))

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LogVectorOutput,
    Meshcat,
    MeshcatVisualizer,
    Parser,
    Simulator,
)

import dtsystems
import iiwa_1
from dtsystems import Observer, PDController, PDController2, SimpleTrajectoryFollower
from utils.plotting import plt

HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
RESULTS = HERE.parents[1] / "results"
FIG.mkdir(exist_ok=True)
RESULTS.mkdir(exist_ok=True)


def iiwa_loop(plant_dt, controller, follower=None, meshcat=None):
    """iiwa with a controller block (and optionally a waypoint follower) closed
    around it.  Returns diagram, plant, and the end-effector frame."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=plant_dt)
    Parser(plant, scene_graph).AddModelsFromUrl(iiwa_1.IIWA14_URL)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))
    plant.Finalize()
    if meshcat is not None:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    logger.set_name("log_plant")
    obs = builder.AddSystem(Observer(14, range(7)))
    ctrl = builder.AddSystem(controller)
    builder.Connect(plant.get_state_output_port(), obs.get_input_port())
    if follower is None:
        builder.Connect(obs.get_output_port(), ctrl.get_input_port())
    else:
        fol = builder.AddSystem(follower)
        builder.Connect(obs.get_output_port(), fol.get_input_port())
        builder.Connect(fol.get_output_port(), ctrl.GetInputPort("target"))
        builder.Connect(obs.get_output_port(), ctrl.GetInputPort("actual"))
        flog = LogVectorOutput(fol.get_output_port(), builder)
        flog.set_name("log_target")
    builder.Connect(ctrl.get_output_port(), plant.get_actuation_input_port())
    return builder.Build(), plant


def run(diagram, plant, q0, T, record=False):
    sim = Simulator(diagram)
    ctx = sim.get_mutable_context()
    plant.SetPositions(plant.GetMyMutableContextFromRoot(ctx), q0)
    # The PD controllers' state is the previous measurement: start it at q0 so
    # the first derivative estimate is zero rather than -q0 / dt.
    for s in diagram.GetSystems():
        if s.get_name() in ("PDController", "PDController2"):
            s.GetMyMutableContextFromRoot(ctx).SetDiscreteState(q0)
    vis = [s for s in diagram.GetSystems() if isinstance(s, MeshcatVisualizer)]
    if record and vis:
        vis[0].StartRecording()
    t0 = time.perf_counter()
    sim.AdvanceTo(T)
    wall = time.perf_counter() - t0
    if record and vis:
        vis[0].PublishRecording()
    logs = {s.get_name(): s.FindLog(sim.get_context()) for s in diagram.GetSystems()
            if s.get_name().startswith("log_")}
    return logs, wall


def ee_path(plant, q_traj):
    ctx = plant.CreateDefaultContext()
    ee = plant.GetFrameByName("iiwa_link_7")
    out = []
    for q in q_traj.T:
        plant.SetPositions(ctx, q)
        out.append(ee.CalcPoseInWorld(ctx).translation())
    return np.array(out).T


def square(kd=3000, kp=10000, eps=0.01, T=20.0, record=False, ctrl_dt=iiwa_1.PD_DT):
    meshcat = Meshcat() if record else None
    diagram, plant = iiwa_loop(
        1e-4,
        PDController2(7, kp, kd, dt=ctrl_dt),
        SimpleTrajectoryFollower(iiwa_1.SQUARE_IIWA, eps, dt=ctrl_dt),
        meshcat,
    )
    logs, wall = run(diagram, plant, iiwa_1.SQUARE_IIWA[0], T, record)
    if record:
        (RESULTS / "ps1_square_iiwa.html").write_text(meshcat.StaticHtml())
    t = logs["log_plant"].sample_times()
    q = logs["log_plant"].data()[:7]
    target = logs["log_target"].data()
    # done = first time the follower has its last waypoint and the arm is there
    last = iiwa_1.SQUARE_IIWA[-1]
    # The square is closed, so the last waypoint is also the first: only count
    # it once the follower has moved on from the start.
    on_last = np.all(np.isclose(target, last[:, None]), axis=0)
    left = np.nonzero(~on_last)[0]
    started = np.arange(len(t)) > (left[0] if len(left) else len(t))
    near = np.linalg.norm(q - last[:, None], axis=0) < eps
    idx = np.nonzero(on_last & near & started)[0]
    t_done = t[idx[0]] if len(idx) else np.inf
    # tracking error: distance of the end effector from the ideal square edges
    p = ee_path(plant, q)
    corners = ee_path(plant, np.array(iiwa_1.SQUARE_IIWA).T)
    dists = []
    for pt in p.T:
        d = []
        for a, b in zip(corners.T[:-1], corners.T[1:]):
            u = np.clip(np.dot(pt - a, b - a) / np.dot(b - a, b - a), 0, 1)
            d.append(np.linalg.norm(pt - (a + u * (b - a))))
        dists.append(min(d))
    dists = np.array(dists)
    sel = t <= t_done
    return dict(t=t, q=q, p=p, corners=corners, t_done=t_done,
                err_max=dists[sel].max(), err_rms=np.sqrt(np.mean(dists[sel] ** 2)), wall=wall)


def p9_5():
    r = square(record=True)
    print(f"9.5 square: done at {r['t_done']:.2f} s, max deviation from the square "
          f"{r['err_max'] * 1000:.1f} mm, RMS {r['err_rms'] * 1000:.1f} mm")
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.plot(*r["p"], lw=1)
    ax.plot(*r["corners"], "k--o", ms=4)
    ax.set_title("9.5 end-effector path (kp 10000, kd 3000, eps 0.01)")
    fig.savefig(FIG / "p9_5_square_path.png", dpi=120)
    plt.close(fig)


def p10():
    """Plant step and controller period for the 7.4-style PD step on the iiwa."""
    target = iiwa_1.Q_START + iiwa_1.Q_STEP
    T = 3.0
    # Ground truth: the finest plant step at a fixed controller period.
    ctrl_dt = 1e-3
    plant_dts = [1e-5, 3e-5, 1e-4, 3e-4, 1e-3]
    rows = []
    ref = None
    for h in plant_dts:
        diagram, plant = iiwa_loop(h, PDController(7, target, 100, 30, dt=ctrl_dt))
        logs, wall = run(diagram, plant, iiwa_1.Q_START, T)
        lg = logs["log_plant"]
        tt, qq = lg.sample_times(), lg.data()[:7]
        grid = np.arange(0, T, 1e-2)
        qg = np.array([np.interp(grid, tt, row) for row in qq])
        if ref is None:
            ref = qg
        rows.append((h, np.abs(qg - ref).max(), wall))
    print("10 plant step (controller period 1 ms): h, max |q - q_ref| (rad), wall (s)")
    for h, e, w in rows:
        print(f"   {h:.0e}  {e:.2e}  {w:.2f}")

    # Controller period at a fixed plant step: the derivative term divides by
    # the controller's dt, so a slow controller acts on a stale, noisy velocity.
    ctrl_dts = [1e-4, 1e-3, 1e-2, 3e-2, 5e-2, 1e-1]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    crows = []
    for cdt in ctrl_dts:
        diagram, plant = iiwa_loop(1e-4, PDController(7, target, 100, 30, dt=cdt))
        logs, wall = run(diagram, plant, iiwa_1.Q_START, T)
        lg = logs["log_plant"]
        q2 = lg.data()[1]
        axes[0].plot(lg.sample_times(), q2, label=f"ctrl dt {cdt:g}")
        crows.append((cdt, np.abs(lg.data()[:7, -1]).max() < 10 and np.isfinite(q2).all(), q2[-1], wall))
    print("10 controller period (plant step 1e-4): dt, finite, final q2, wall (s)")
    for cdt, ok, q2f, w in crows:
        print(f"   {cdt:g}  {ok}  {q2f:.3f}  {w:.2f}")
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("q2 (rad)")
    axes[0].set_ylim(-3, 3)
    axes[0].legend(fontsize=7)
    axes[0].set_title("PD step, plant 1e-4 s, varying controller period")
    hs, es, ws = zip(*rows)
    axes[1].loglog(hs[1:], es[1:], "o-", label="error vs h = 1e-5")
    ax2 = axes[1].twinx()
    ax2.loglog(hs, ws, "s--", color="C1", label="wall time")
    axes[1].set_xlabel("plant time step (s)")
    axes[1].set_ylabel("max joint error (rad)")
    ax2.set_ylabel("wall-clock time (s)")
    axes[1].set_title("plant step: accuracy and cost")
    fig.tight_layout()
    fig.savefig(FIG / "p10_step_size.png", dpi=120)
    plt.close(fig)


def p11():
    """Sweep the PD damping gain on a single-joint step, and on the square."""
    q_step = np.zeros(7)
    q_step[3] = 0.4
    target = iiwa_1.Q_START + q_step
    kds = [0, 5, 10, 20, 30, 60, 120]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    print("11 joint-4 step of 0.4 rad, kp 100: kd, overshoot (rad), 2% settling (s)")
    for kd in kds:
        diagram, plant = iiwa_loop(1e-4, PDController(7, target, 100, kd, dt=1e-3))
        logs, _ = run(diagram, plant, iiwa_1.Q_START, 6.0)
        lg = logs["log_plant"]
        t, q4 = lg.sample_times(), lg.data()[3]
        final = q4[-1]
        span = final - q4[0]
        over = max(0.0, (q4 - final).max() * np.sign(span)) if span > 0 else max(0.0, (final - q4).max())
        band = np.abs(q4 - final) > 0.02 * abs(span)
        ts = t[np.nonzero(band)[0][-1] + 1] if band.any() and band.nonzero()[0][-1] + 1 < len(t) else t[0]
        print(f"   {kd:4d}  {over:.3f}  {ts:.2f}")
        axes[0].plot(t, q4, label=f"kd {kd}")
    axes[0].axhline(target[3], color="0.5", ls="--")
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("q4 (rad)")
    axes[0].legend(fontsize=7)
    axes[0].set_title("joint 4 step, kp 100")

    print("11 square, kp 10000: kd, time to finish (s), max / RMS deviation (mm)")
    kds_sq = [300, 1000, 3000, 6000, 10000]
    res = []
    for kd in kds_sq:
        r = square(kd=kd, T=30.0)
        res.append((kd, r["t_done"], r["err_max"] * 1000, r["err_rms"] * 1000))
        print(f"   {kd:5d}  {r['t_done']:.2f}  {r['err_max'] * 1000:.1f} / {r['err_rms'] * 1000:.1f}")
    kd_, td, em, er = zip(*res)
    axes[1].semilogx(kd_, td, "o-", label="time to finish (s)")
    ax2 = axes[1].twinx()
    ax2.semilogx(kd_, er, "s--", color="C1", label="RMS deviation (mm)")
    axes[1].set_xlabel("kd")
    axes[1].set_ylabel("time to finish (s)")
    ax2.set_ylabel("RMS deviation from square (mm)")
    axes[1].set_title("square: damping trades speed for straightness")
    fig.tight_layout()
    fig.savefig(FIG / "p11_damping.png", dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    which = sys.argv[1:] or ["square", "p10", "p11"]
    if "square" in which:
        p9_5()
    if "p10" in which:
        p10()
    if "p11" in which:
        p11()
