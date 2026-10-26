"""Put every problem set's directory on the path, so tests import the pset
modules by name the way the scripts do when run from their own directory.

Module names are unique across psets/ps1 to psets/ps6, so one path serves all.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")

PSETS = Path(__file__).resolve().parents[1] / "psets"
sys.path.insert(0, str(PSETS))
for ps in sorted(PSETS.glob("ps[0-9]")):
    sys.path.insert(0, str(ps))
