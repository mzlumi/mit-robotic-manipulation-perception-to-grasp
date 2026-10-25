# mit-robotic-manipulation-perception-to-grasp

Self-study archive for **MIT 6.4210 / 6.4212 Robotic Manipulation** (Russ Tedrake, Tomas Lozano-Perez and staff), based on the latest offering, **Fall 2026**. It collects the official, unmodified course materials: problem set handouts with starter code and tests, official solutions, lecture slides and notes, the textbook (chapters and notebooks), and the final project guidelines.

> **Keep this repo private.** The upstream 6.4210 handouts README asks students not to fork the handouts repository publicly, because public forks can never be made private and would publish homework solutions. This repo vendors that handouts snapshot and also contains official solutions PDFs, so it must stay in a private repository. If you add your own solutions, keep them private too.

Everything here was retrieved on 2026-10-22 from official sources only (the course site and the GitHub repos of the course staff). See [SOURCES.md](SOURCES.md) for the original URL of every file and the upstream commit SHAs. Handout text and code are exact copies; nothing was summarized or edited. No downloaded code or notebook was executed while building this repo.

## What is inside

```
README.md, SOURCES.md
problem-sets-fall2026/    Whole snapshot of the Fall 2026 handouts repo: ps1..ps6 (PDF + code + notebooks + data), utils/
solutions/fall2026/       Official PS1-PS4 solutions, Quiz 1 solutions, midterm practice problems (+ solutions)
lectures/                 fall2026/ (slides PDFs, behavior cloning and RL course notes), fall2025/ (21 lectures), fall2024/
textbook/
  chapters/html/          Every book chapter as served by the site (offline-readable, see below)
  chapters/markdown/      Pandoc Markdown copies of the chapters (lossy for interactive parts and some equations)
  manipulation/           Whole snapshot of RussTedrake/manipulation: book sources, ~95 notebooks,
                          the `manipulation` Python package and models, plus the book/data and book/htmlbook submodules
project/                  Fall 2026 project guidelines, past projects with videos, example proposals, AI policies
course-site/fall2026/     Saved course home page and schedule (HTML and Markdown)
older-offerings/          Course home, schedule, project and problem set pages for Fall 2020 to Fall 2025
```

Where to find specific things:

- **Textbook exercise notebooks**: `textbook/manipulation/book/<chapter>/exercises/*.ipynb` (intro, robot, pick, pose, clutter, trajectories, mobile, force, segmentation, deep_perception, rl).
- **Chapter demo notebooks**: for example `textbook/manipulation/book/clutter/clutter_clearing.ipynb`, `book/pick/pick.ipynb`, `book/pose/icp.ipynb`.
- **Python package**: `textbook/manipulation/manipulation/` (stations, scenarios, pick, ICP, utils, `models/`, `exercises/` autograder support).
- **Book source HTML** is also at `textbook/manipulation/book/*.html` (byte-identical to the chapters in `textbook/chapters/html/`).
- **Lecture videos**: see "Official links" below.

## Course overview

The course builds a software stack that lets a simulated robot arm manipulate objects in cluttered scenes. Topics: coordinate frames and kinematics, differential IK, optimization-based kinematics, sampling-based motion planning, statics and grasping, behavior cloning, reinforcement learning, model-based learning, force and impedance control, perception (segmentation, deep perception), task and motion planning. Coding is in Python with [Drake](https://drake.mit.edu). 6.4210 is the undergraduate version (CI-M, with recitations); 6.4212 is the graduate version with extra problem set questions and higher project expectations. In Fall 2026, lectures were Mon/Wed 9:30-11:00, there were two quizzes and a team final project.

## Suggested self-study order (Fall 2026 problem sets)

Do the problem sets in order; each PDF has "bookwork" questions (answers entered in Gradescope, which you can only self-check against the solutions where posted) and "computational questions" with starter code. Read the matching chapters and slides first.

| PS | Title | Matching reading and slides | Code | Official solutions here |
|---|---|---|---|---|
| 1 | Getting Started | Ch. `intro`, `robot`; `lectures/fall2026/fall26-lec1.pdf` | `ps1/cartesian_2d_robot.py`, `iiwa_1.py`, `dtsystems.py`, `ps1_colab.ipynb` | `solutions/fall2026/ps1_solutions.pdf` |
| 2 | Transforms and Kinematics (**pick and place**) | Ch. `pick` 3.0 to 3.4, `spatial` (Appendix A); lec3, lec4 | `ps2/forward_kin.py`, `ps2/pick_and_place.py` (Kuka iiwa builds a block tower) | `ps2_solutions.pdf` |
| 3 | Optimization and Motion Planning | Ch. `pick` 3.5 to 3.9, `trajectories`; lec5, lec6 | `ps3/intro_to_qp.py`, `diffik_optimization.py`, `rrt_planning.py`, `mobile_base_ik.py`, `door_opening.py`, `exact_edge_check.py` | `ps3_solutions.pdf` |
| 4 | Grasping (**antipodal grasping**) | Ch. `clutter` 5.1 to 5.6; lec7 | `ps4/normal_estimation.py`, `analytic_antipodal_grasps.py`, `sampling_antipodal_grasps.py` | `ps4_solutions.pdf` |
| 5 | Behavior Cloning | `lectures/fall2026/bc_notes.pdf` | `ps5/diffusion_policy_pusht_state.ipynb` (diffusion policy on Push-T) | not posted |
| 6 | Reinforcement Learning (**residual PPO**, most project-like) | `lectures/fall2026/rl_notes.pdf`; lec10, lec11 | `ps6/reinforce.py`, `cartpole.ipynb`, `push_math.py`, `residual_push.py`, `ppo_push.ipynb`, plus supplied training results in `ps6/data/` | not posted |

Highlights:

- **PS2 (pick and place)** is the end-to-end integration exercise: frames, forward kinematics, differential IK and a gripper/arm system working together.
- **PS4 (antipodal grasping)** takes you from depth-image normal estimation to analytic and sampled antipodal grasps on a mustard bottle and letter meshes.
- **PS6 (residual PPO)** is the closest to a final project: after REINFORCE and PPO theory plus cart-pole, Problem 7 improves a cautious behavior-cloned push policy with a residual PPO policy. Full multi-hour training results are supplied in `ps6/data/` so you analyze them instead of rerunning them.

Exam practice after PS1-PS4: `solutions/fall2026/midterm_practice.pdf`, then check against `midterm_practice_solutions.pdf`; `quiz1_solutions.pdf` is the Quiz 1 answer key. Beyond the problem sets, the textbook exercise notebooks listed above are the older-style coding exercises (the earlier offerings' problem sets in `older-offerings/` just point to them). Fall 2025 slides in `lectures/fall2025/` give a second pass on most topics, including later lectures not yet posted for Fall 2026.

## Setup

Based on the upstream handouts README (`problem-sets-fall2026/README.md`, sections 3 and 4). Its relative paths assume that `problem-sets-fall2026/` is the root of the clone.

Local workflow (recommended):

```sh
# one-time prerequisites: git, uv (https://docs.astral.sh/uv/), Graphviz (brew install graphviz)
cd problem-sets-fall2026
uv python install 3.13
uv venv --python 3.13
uv pip install drake
uv pip install "manipulation[letter]"
uv pip install scipy rtree
uv pip install torch diffusers einops zarr gdown pymunk   # PS5
uv pip install torch gymnasium pandas ipykernel           # PS6 (torch is shared with PS5)
source .venv/bin/activate
cd ps1 && python cartesian_2d_robot.py
```

Notes:

- Python 3.13. Windows has no Drake builds; use WSL (Ubuntu).
- Put the venv on a path with no spaces or special characters (Drake's model downloader breaks otherwise; see the upstream README for the workaround with a venv in your home directory).
- Fill in the bodies marked `raise NotImplementedError("your code here")`.
- Drake downloads some robot models on first use, and the `manipulation` package fetches a few external model repositories at runtime (see `textbook/manipulation/manipulation/remotes.py`), so the first run needs internet access.
- This repo contains no solutions to the coding parts. The Colab notebooks (`psN_colab.ipynb`) bootstrap by cloning the **upstream public handouts repo** into your Google Drive, not this repo, so for Colab either follow the upstream README as written or upload this folder to Drive yourself.

Textbook notebooks use the PyPI `manipulation` package (installed above as `manipulation[letter]`); the vendored copy in `textbook/manipulation/` is the matching source snapshot (see its `README.md` and `pyproject.toml`).

### Reading the book offline

```sh
cd textbook/chapters/html
python3 -m http.server 8000     # then open http://localhost:8000/index.html
```

A local web server is needed because the pages load `chapters.json` with `fetch`. The `figures`, `data`, `htmlbook` and other asset entries in that folder are symlinks into `textbook/manipulation/book/`. Equations are typeset by MathJax loaded from a CDN, so they need an internet connection; the Markdown copies keep the raw LaTeX.

## Final project guidance

The official page is saved at `project/project-guidelines-fall2026.html` (and `.md`); read it in full. In short, as stated there:

- Teams of 3, scoped to roughly 40 x 3 = 120 person-hours over 4 to 5 weeks; the topic must be directly relevant to robot manipulation and build on course material.
- Every project should compare two or more approaches (for a learning method, include a classical baseline).
- Deliverables: pre-proposal (500-750 words), final proposal (about 1000 words), short progress updates, a roughly 5-minute video, an IEEE-format conference-style report of about 4500 words (with per-member contributions and a description of AI use), and in-class summaries.
- Coding agents are allowed, but you must own the robotics-level design choices. The AI policies are in `project/policies/`.
- ML projects are judged on ideas and framing, not on GPU time.
- The page lists project ideas by area (IK and trajectories, motion planning, grasping, control, TAMP, learning), each with starting papers.

For self-study: pick a PS-sized problem you already did (for example, PS4 antipodal grasping or PS6 residual PPO) and extend it with a second approach and a quantitative comparison; look at `project/example-proposals/` (rock skipping, throwing and catching) for what a proposal looks like, and `project/past-projects.md` for outstanding projects with video links.

## Official links

- Course site (Fall 2026): https://manipulation.csail.mit.edu/Fall2026/ (schedule, project, and problem set PDFs and solutions)
- Textbook: https://manipulation.csail.mit.edu (also https://manipulation.mit.edu)
- Handouts repo (do not fork publicly): https://github.com/tlpmit/6.4210-2-fall26-handouts
- Textbook repo: https://github.com/RussTedrake/manipulation
- Past projects: https://manipulation.csail.mit.edu/misc.html#projects
- Lecture videos: Fall 2026 recordings are on Canvas for registered students only. The public lecture playlist linked from the Fall 2025 schedule page is https://www.youtube.com/playlist?list=PLkx8KyIQkMfWr191lqbN8WfV08j-ui8WX (described there as the Fall 2023 term), and the textbook's YouTube channel is https://www.youtube.com/channel/UChfUOAhz7ynELF-s_1LPpWg.
- Online slides (some lectures have slides.com versions not in this repo): see `course-site/fall2026/schedule.md`.
- Convex optimization lectures recommended by the staff: https://youtube.com/playlist?list=PL3940DD956CDF0622

## Not available

Searched on the official sites and returned 404 (or are not posted); nothing was substituted from third-party sources.

- Fall 2026 solutions for PS5 and PS6 (`Fall2026/ps5.pdf`, `ps6.pdf`) and Quiz 2 solutions (`Fall2026/quiz2_solutions.pdf`): not posted as of 2026-10-22.
- Fall 2026 lecture PDFs for lectures 2, 8, 9 and 13 to 21: the schedule links only online slides (lectures 2, 8, 9) or nothing yet. Lectures 1, 3 to 7 and 10 to 12 are included.
- `michelangelo_proposal.pdf` (example proposal linked from Fall2023 to Fall2025 pages): 404 in every year, as expected.
- `Fall2026/GAI_policy.pdf`: 404; Fall 2026 uses `GAI-Comm.pdf` and `GAI-Tech.pdf` instead (included).
- Fall 2023 and Fall 2024 lecture slides (only `Fall2024/lec/lecture-1.pdf` is posted); Fall 2022, 2021 and 2020 have no lecture PDFs on the site. Fall 2022 has no problem set pages.
- `Fall2021/pset_0.html` and `Fall2020/pset_7.html` to `pset_10.html`: 404.
- Instructor solutions for the textbook exercise notebooks (the upstream autograder references a solutions version in `solutions_sha.txt`, but the solutions are not public).
- Slides.com online slide decks (no PDF export link on the course site) and Canvas recordings.
- Chapter videos and other video files embedded in the textbook were intentionally skipped. A few chapter pages reference Meshcat HTML recordings; those that live in the `manipulation-data` repo are included in `textbook/manipulation/book/data/`.
- Nothing was left out for size: all upstream repos were included in full (largest single file is about 28 MB), so Git LFS is not needed.

## Licenses

The handouts and the textbook repo are BSD 3-clause licensed (see `problem-sets-fall2026/LICENSE` and `textbook/manipulation/LICENSE.TXT`). Course website pages, slides, solutions and notes are copyright their authors (Robot Locomotion Group @ CSAIL and the course staff) and are archived here for personal study only.
