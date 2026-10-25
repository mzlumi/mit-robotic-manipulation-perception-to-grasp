# Sources

Every file or folder in this repo, with its original URL. All material was retrieved on **2026-10-22** from official MIT course sites or the official GitHub repositories of the course staff and of Russ Tedrake. Nothing was taken from third-party copies, and nothing was executed.

Base URL for the course site: `https://manipulation.csail.mit.edu` (abbreviated `SITE` below).

## Upstream git repositories (vendored snapshots, no `.git` folder)

Each was shallow-cloned (`--depth 1`) and its working tree copied without the `.git` directory.

| Repo folder | Upstream repository | Commit SHA | Notes |
|---|---|---|---|
| `problem-sets-fall2026/` | https://github.com/tlpmit/6.4210-2-fall26-handouts | `e8747a31b2b95df9ae547d9454616dc22f8cb98c` | Whole repo (ps1 to ps6, `utils/`, README, LICENSE, `.gitignore`). 269 files plus the upstream `.gitignore`. |
| `textbook/manipulation/` | https://github.com/RussTedrake/manipulation | `dd2bc0f3e5db9898d1aa5fc7a61c4bb4cd1ead03` | Whole repo: `book/` sources, all notebooks, the `manipulation` Python package and models, `docs/`, `setup/`. |
| `textbook/manipulation/book/data/` | https://github.com/RussTedrake/manipulation-data | `4fecc7339423e0368928377f3bc13f03e9bde20d` | Git submodule `book/data` of the repo above; empty in a plain clone, so it was cloned separately at the pinned commit and placed at its submodule path. |
| `textbook/manipulation/book/htmlbook/` | https://github.com/RussTedrake/htmlbook | `e9b1d5bf70629bb8674b58e0c14ff968fcab1c39` | Git submodule `book/htmlbook` (CSS and JS used by the chapter pages), same treatment. |

`textbook/manipulation/solutions_sha.txt` is an upstream file and is kept as is.

## Fall 2026 course pages

| Repo path | Original URL |
|---|---|
| `course-site/fall2026/index.html` (+ `index.md`) | SITE/Fall2026/index.html |
| `course-site/fall2026/schedule.html` | SITE/Fall2026/schedule.html |
| `course-site/fall2026/schedule.md` | Extracted by script from the JavaScript data embedded in SITE/Fall2026/schedule.html |
| `course-site/fall2026/project.html` | SITE/Fall2026/project.html |

## Solutions and exam material (`solutions/fall2026/`)

| Repo path | Original URL |
|---|---|
| `ps1_solutions.pdf` | SITE/Fall2026/ps1.pdf |
| `ps2_solutions.pdf` | SITE/Fall2026/ps2.pdf |
| `ps3_solutions.pdf` | SITE/Fall2026/ps3.pdf |
| `ps4_solutions.pdf` | SITE/Fall2026/ps4.pdf |
| `quiz1_solutions.pdf` | SITE/Fall2026/quiz1_solutions.pdf |
| `midterm_practice.pdf` | SITE/Fall2026/midterm_practice.pdf |
| `midterm_practice_solutions.pdf` | SITE/Fall2026/midterm_practice_solutions.pdf |

The solutions were renamed from `psN.pdf` to `psN_solutions.pdf` to avoid confusion with the handouts; contents are byte-for-byte as downloaded.

## Lecture slides and notes (`lectures/`)

| Repo path | Original URL |
|---|---|
| `lectures/fall2026/fall26-lec1.pdf` | SITE/Fall2026/lec/fall26-lec1.pdf |
| `lectures/fall2026/fall26-lec3.pdf` | SITE/Fall2026/lec/fall26-lec3.pdf |
| `lectures/fall2026/fall26-lec4.pdf` | SITE/Fall2026/lec/fall26-lec4.pdf |
| `lectures/fall2026/fall26-lec5.pdf` | SITE/Fall2026/lec/fall26-lec5.pdf |
| `lectures/fall2026/fall26-lec6.pdf` | SITE/Fall2026/lec/fall26-lec6.pdf |
| `lectures/fall2026/fall26-lec7.pdf` | SITE/Fall2026/lec/fall26-lec7.pdf |
| `lectures/fall2026/fall26-lec10.pdf` | SITE/Fall2026/lec/fall26-lec10.pdf |
| `lectures/fall2026/fall26-lec11.pdf` | SITE/Fall2026/lec/fall26-lec11.pdf |
| `lectures/fall2026/fall26-lec12.pdf` | SITE/Fall2026/lec/fall26-lec12.pdf |
| `lectures/fall2026/bc_notes.pdf` (behavior cloning course notes) | SITE/Fall2026/lec/bc_notes.pdf |
| `lectures/fall2026/rl_notes.pdf` (reinforcement learning course notes) | SITE/Fall2026/lec/rl_notes.pdf |
| `lectures/fall2025/fall25-lec1.pdf` to `fall25-lec21.pdf` | SITE/Fall2025/lec/fall25-lecN.pdf (N = 1 to 21) |
| `lectures/fall2024/lecture-1.pdf` | SITE/Fall2024/lec/lecture-1.pdf |

## Project (`project/`)

| Repo path | Original URL |
|---|---|
| `project-guidelines-fall2026.html` | SITE/Fall2026/project.html |
| `project-guidelines-fall2026.md` | pandoc conversion of the file above |
| `past-projects.md` | SITE/misc.html#projects (section "Some great final projects", with video links) |
| `example-proposals/rock_skipping_proposal.pdf` | SITE/Fall2025/example_proposals/rock_skipping_proposal.pdf (the same file is served under Fall2024 and Fall2026) |
| `example-proposals/throwing_and_catching_proposal.pdf` | SITE/Fall2025/example_proposals/throwing_and_catching_proposal.pdf |
| `policies/GAI-Comm.pdf` | SITE/Fall2026/GAI-Comm.pdf |
| `policies/GAI-Tech.pdf` | SITE/Fall2026/GAI-Tech.pdf |
| `policies/GAI_policy_fall2025.pdf` | SITE/Fall2025/GAI_policy.pdf |

## Textbook chapters (`textbook/chapters/`)

`html/` holds the pages exactly as served, one per URL `SITE/<name>.html`. They were checked to be byte-identical to `book/<name>.html` in the vendored repo. `html/index.html` is the table of contents (`SITE/index.html`). The `figures`, `data`, `htmlbook`, `notebooks.js` and `chapters.json` entries inside `html/` are relative symlinks into `textbook/manipulation/book/` so the pages can be read offline. `markdown/<name>.md` are pandoc conversions of those pages.

| Chapter file | Original URL |
|---|---|
| `intro` | SITE/intro.html |
| `robot` | SITE/robot.html |
| `pick` | SITE/pick.html |
| `pose` | SITE/pose.html |
| `clutter` | SITE/clutter.html |
| `trajectories` | SITE/trajectories.html |
| `mobile` | SITE/mobile.html |
| `force` | SITE/force.html |
| `segmentation` | SITE/segmentation.html |
| `deep_perception` | SITE/deep_perception.html |
| `rl` | SITE/rl.html |
| `tactile` | SITE/tactile.html |
| `spatial` (Appendix A) | SITE/spatial.html |
| `drake` | SITE/drake.html |
| `environments` | SITE/environments.html |
| `station` | SITE/station.html |
| `misc` | SITE/misc.html |
| `tamp`, `visuomotor`, `foundation_models` (draft chapters) | SITE/tamp.html, SITE/visuomotor.html, SITE/foundation_models.html |
| `underactuated` | SITE/underactuated.html |
| `index` | SITE/index.html |

## Older offerings (`older-offerings/FallYYYY/`)

Saved as served from `SITE/FallYYYY/<file>`. Only files that returned HTTP 200 are included.

| Folder | Files | Original URLs |
|---|---|---|
| `Fall2025/` | `index`, `schedule`, `project`, `pset_1` to `pset_10`, `pset_1_25`, `pset_X_25`, `pset_64212`, `subject_eval` (`.html`) | SITE/Fall2025/... |
| `Fall2024/` | `index`, `schedule`, `project`, `pset_1` to `pset_10`, `subject_eval` | SITE/Fall2024/... |
| `Fall2023/` | `index`, `schedule`, `project`, `pset_1` to `pset_10`, `subject_eval` | SITE/Fall2023/... |
| `Fall2022/` | `index`, `schedule`, `project` | SITE/Fall2022/... |
| `Fall2021/` | `index`, `schedule`, `project`, `pset_1` to `pset_10` | SITE/Fall2021/... |
| `Fall2020/` | `index`, `schedule`, `project`, `pset_0` to `pset_6` | SITE/Fall2020/... |

Several `pset_N.html` pages are served byte-identically under Fall2024 and Fall2025 (the server returns the same file), so those copies are duplicates by design.

## Files written for this repo

`README.md`, `SOURCES.md`, `.gitignore`, `textbook/chapters/markdown/*.md` (generated), `project/past-projects.md` (extracted), `course-site/fall2026/schedule.md` and `index.md` (generated), `project/project-guidelines-fall2026.md` (generated).
