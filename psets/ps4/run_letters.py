"""PS4 11.5: pick the first initial, turn it to match the second, put it down.

    python run_letters.py B B
    python run_letters.py P M --record    # also saves results/ps4_letters_P_M.html

Prints the final orientation error of the letter.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.append(str(Path(__file__).resolve().parents[1]))

from pydrake.all import Meshcat

import sampling_antipodal_grasps as sag
import utils.viz

RESULTS = Path(__file__).resolve().parents[2] / "results"


def main(first: str, second: str, record: bool, seed: int = 0) -> None:
    sag.INITIALS = [first, second]
    sag.write_letter_assets()
    if record:
        utils.viz._meshcat = Meshcat()
        sag.get_meshcat = lambda: utils.viz._meshcat
    sag.test5(seed=seed)
    if record:
        RESULTS.mkdir(exist_ok=True)
        (RESULTS / f"ps4_letters_{first}_{second}.html").write_text(
            utils.viz._meshcat.StaticHtml()
        )


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    main(args[0], args[1], "--record" in sys.argv)
