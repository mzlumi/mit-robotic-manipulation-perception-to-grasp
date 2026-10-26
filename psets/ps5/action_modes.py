"""PS5 Problem 13: the pretrained Push-T diffusion policy's action distribution
at the handout's initial conditions, drawn and measured.

    python action_modes.py

Loads the helpers bundled in diffusion_policy_pusht_state.ipynb (the first run
downloads the authors' checkpoint into assets/), samples action sequences for
every query, and writes

    figures/ps5_action_samples_T0.png     T at its initial pose
    figures/ps5_action_samples_T100.png   T displaced 100 px in +x
    results/ps5_action_modes.md           a table of the mode analysis

Modality is judged by eye in the figures and backed by a number: each sampled
sequence is summarized by where it ends relative to the pusher, the endpoints
are split into two clusters with k-means, and the split counts as two modes
when the gap between the cluster centers is large compared with the spread
inside them (a separation ratio above SEPARATION and at least MIN_SHARE of
the samples in the smaller cluster).
"""

import json
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("HEADLESS", "1")

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)
(ROOT / "results").mkdir(exist_ok=True)

NUM_SAMPLES = 64
SEPARATION = 4.0
MIN_SHARE = 0.1


def load_helpers():
    """Run the notebook's own helper cell, which registers its modules."""
    os.chdir(HERE)  # the helpers download into ./assets
    nb = json.loads((HERE / "diffusion_policy_pusht_state.ipynb").read_text())
    cell = next(
        "".join(c["source"])
        for c in nb["cells"]
        if c["cell_type"] == "code" and "Load notebook helpers" in "".join(c["source"])
    )
    exec(cell, {"__name__": "helpers"})
    import pusht
    import pusht_diffusion

    return pusht, pusht_diffusion


def samples_for(pd, pusht, p_TPusher, X_WT, v_WPusher=(0.0, 0.0), seed=0):
    """Sampled action sequences, (num_samples, PRED_HORIZON, 2), in pixels,
    for the pusher at p_TPusher in the T frame -- the same history that
    show_action_samples builds."""
    policy = pd.default_policy()
    p_WPusher = np.asarray(pusht.apply_transform(X_WT, p_TPusher), dtype=float)
    v = np.asarray(v_WPusher, dtype=float)
    obs_history = np.stack(
        [
            pusht.PushTEnv(
                np.concatenate([p_WPusher - k * v / pd.CONTROL_HZ, X_WT])
            ).observation()
            for k in reversed(range(pd.OBS_HORIZON))
        ]
    )
    generator = torch.Generator(device=policy.device).manual_seed(seed)
    actions = pd.sample_action_sequences(
        policy, np.stack([obs_history] * NUM_SAMPLES), generator
    )
    return p_WPusher, np.asarray(actions)


def two_means(points, iters=50, seed=0):
    """k-means with k = 2; returns labels and the two centers."""
    rng = np.random.default_rng(seed)
    # Farthest-pair initialization is deterministic and finds a real split
    # when there is one.
    i = rng.integers(len(points))
    a = points[np.argmax(np.linalg.norm(points - points[i], axis=1))]
    b = points[np.argmax(np.linalg.norm(points - a, axis=1))]
    centers = np.stack([a, b])
    for _ in range(iters):
        labels = np.argmin(
            np.linalg.norm(points[:, None] - centers[None], axis=2), axis=1
        )
        new = np.stack(
            [points[labels == k].mean(axis=0) if np.any(labels == k) else centers[k] for k in (0, 1)]
        )
        if np.allclose(new, centers):
            break
        centers = new
    return labels, centers


def modality(p_WPusher, actions):
    """Mode analysis of the sequences' endpoints relative to the pusher."""
    ends = actions[:, -1, :] - p_WPusher
    labels, centers = two_means(ends)
    share = min(np.mean(labels == 0), np.mean(labels == 1))
    within = np.sqrt(
        np.mean([np.sum((ends[labels == k] - centers[k]) ** 2, axis=1).mean() for k in (0, 1) if np.any(labels == k)])
    )
    gap = np.linalg.norm(centers[0] - centers[1])
    ratio = gap / max(within, 1e-9)
    bimodal = ratio > SEPARATION and share >= MIN_SHARE
    # Direction of each cluster's mean end, in degrees, image coordinates.
    headings = [float(np.degrees(np.arctan2(c[1], c[0]))) for c in centers]
    return dict(
        bimodal=bool(bimodal),
        ratio=float(ratio),
        share=float(share),
        reach=float(np.linalg.norm(ends, axis=1).mean()),
        headings=headings,
    )


QUERIES = [
    ("(0, -100)", (0, -100), (0, 0)),
    ("(100, -100)", (100, -100), (0, 0)),
    ("(100, 50)", (100, 50), (0, 0)),
    ("(50, 50)", (50, 50), (0, 0)),
    ("(0, -300)", (0, -300), (0, 0)),
    ("(0, -100), v = (100, 0)", (0, -100), (100, 0)),
    ("(0, -100), v = (-100, 0)", (0, -100), (-100, 0)),
]


def run(pd, pusht, X_WT, tag, rows):
    plt = pd.plt
    fig, axes = plt.subplots(2, 4, figsize=(16, 8.5))
    for ax, (label, p_T, v) in zip(axes.flat, QUERIES):
        p_W, actions = samples_for(pd, pusht, p_T, np.asarray(X_WT, dtype=float), v)
        m = modality(p_W, actions)
        rows.append((tag, label, m))
        pusht.setup_axes(ax)
        pusht.draw_state(ax, np.concatenate([p_W, X_WT]))
        pd.draw_paths(ax, actions)
        ax.set_title(
            f"{label}\n{'bi' if m['bimodal'] else 'uni'}-modal (sep {m['ratio']:.1f}, minor {m['share']:.0%})",
            fontsize=9,
        )
    axes.flat[-1].axis("off")
    fig.suptitle(f"Sampled action sequences, {NUM_SAMPLES} per query, {tag}")
    fig.tight_layout()
    fig.savefig(FIG / f"ps5_action_samples_{'T0' if 'initial' in tag else 'T100'}.png", dpi=110)
    plt.close(fig)


def main():
    pusht, pd = load_helpers()
    X_WT0 = np.asarray(pd.LOGO_X_WT, dtype=float)
    X_WT1 = X_WT0 + np.array([100.0, 0.0, 0.0])
    rows = []
    run(pd, pusht, X_WT0, "T at its initial pose", rows)
    run(pd, pusht, X_WT1, "T displaced 100 px in +x", rows)

    lines = [
        "# PS5: modes of the Push-T diffusion policy",
        "",
        f"{NUM_SAMPLES} samples per query from the authors' pretrained checkpoint, seed 0. "
        "Pusher positions are in the T frame (pixels); velocities in pixels per second. "
        "Separation is the distance between the two k-means centers of the sequence endpoints "
        "divided by the RMS spread within the clusters; a query counts as bimodal when it is above "
        f"{SEPARATION} with at least {MIN_SHARE:.0%} of the samples in the smaller cluster. "
        "Headings are the directions of the two cluster centers from the pusher, in image "
        "coordinates (y down).",
        "",
        "| T pose | pusher (T frame) | modes | separation | minor share | mean reach (px) | cluster headings (deg) |",
        "|---|---|---|---|---|---|---|",
    ]
    for tag, label, m in rows:
        lines.append(
            f"| {tag} | {label} | {'2' if m['bimodal'] else '1'} | {m['ratio']:.1f} | "
            f"{m['share']:.0%} | {m['reach']:.0f} | {m['headings'][0]:.0f}, {m['headings'][1]:.0f} |"
        )
    text = "\n".join(lines) + "\n"
    (ROOT / "results" / "ps5_action_modes.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
