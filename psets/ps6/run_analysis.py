"""PS6 numerical answers that need no long training: the supplied 30-pair
cart-pole study, the BC fit, the one-batch PPO update, and the supplied
residual-PPO runs.  The same calls as the notebooks, without Jupyter.

    python run_analysis.py

Tables print to stdout; plots go to results/ under this directory.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("OMP_NUM_THREADS", "1")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
os.chdir(HERE)

import pandas as pd
import torch

torch.set_num_threads(1)
pd.set_option("display.width", 200)
pd.set_option("display.max_columns", 30)


def cartpole_study():
    from homework import DATA, RESULTS
    from learning_curves import compare_runs, plot_learning_curves

    plot_learning_curves(DATA / "cartpole", band="bootstrap", output_dir=RESULTS / "cartpole_reference")
    inference = compare_runs(DATA / "cartpole", show=False, output_dir=RESULTS / "cartpole_reference")
    print("== 6.2 supplied 30-pair cart-pole study")
    for k in ["n_pairs", "mean_difference", "paired_bootstrap_ci95", "paired_t_statistic", "paired_t_p"]:
        print(f"   {k}: {inference[k]}")


def push():
    from residual_push import clone_cautious, update_batch, update_from_batch
    from residual_analysis import (
        aggregate_table,
        learning,
        paired_goal_interval,
        reference,
        score_table,
        training_comparison,
    )

    print("== 7.3 BC fit")
    cloning = clone_cautious()
    print(cloning["validation"])
    print("   fit seconds:", cloning["fit_seconds"])

    print("== 7.6 one PPO update")
    batch, metadata = update_batch()
    print("  ", {k: metadata[k] for k in ["transitions", "episodes", "theta", "collection_seconds"]})
    print(update_from_batch())

    print("== 7.8 exploratory runs")
    exploratory = reference("exploratory")
    print(score_table(exploratory, "final"))
    print(aggregate_table(exploratory))
    learning(exploratory)
    print(paired_goal_interval(exploratory, variant="final", metric="execution_seconds", compare="midpoint"))
    print(paired_goal_interval(exploratory, variant="sampled", metric="reward"))

    print("== 7.8 confirmatory runs")
    confirmation = reference("confirmatory")
    print(score_table(confirmation, "final"))
    print(aggregate_table(confirmation))
    print(training_comparison(confirmation))
    learning(confirmation)


if __name__ == "__main__":
    cartpole_study()
    push()
