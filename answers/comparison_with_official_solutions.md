# Comparison with the official solutions

All answers and code were committed (commit `0f946ba`) before I opened the official solutions for PS1 to PS4 in `solutions/fall2026/`. Solutions for PS5 and PS6 are not posted. This file records every place where my committed answer differed from the official one, and what I changed afterwards.

## Summary

| Set | Graded items compared | Differences | Changed afterwards |
|---|---|---|---|
| PS1 | all bookwork, all checkpoints | 1 (8.3, a modeling choice) | yes: the follower period, to match |
| PS2 | all bookwork, all checkpoints | 1 error (9.2), 1 design difference (16.1) | 9.2 corrected |
| PS3 | all bookwork, all checkpoints, Problem 17 | 1 omission (7.1, one of three blanks) | filled in |
| PS4 | all bookwork, all checkpoints, Problem 12 | none | no |

## Details

**PS1 8.3, path length around SQUARE.** Mine: 2.1507 m (ε = 0.1) and 2.3719 m (ε = 0.01). Official: 2.2520 and 2.3768 m. My waypoint follower checked for arrival every 0.01 s; the reference uses `SimpleTrajectoryFollower`'s default period of 0.1 s. With the default I reproduce the official values to four decimals (and at 0.001 s get 2.1384 and 2.3711). A follower that checks less often lets the finger run on past the ε ball before it switches target, which lengthens the path. I switched `create_2d_robot_diagram_with_waypoints` to the default so that the reported checkpoints match the assignment's convention.

**PS1, within tolerance.** The settling times (8.2: mine 1.3 to 1.7 s depending on tolerance, official "about 1 s, 1.3 ± 0.5 accepted"; 9.4: mine 4.0 s, official "about 3 s, 3 ± 2 accepted") depend on the tolerance used for "at rest". Both are inside the accepted ranges.

**PS2 9.2, redundant robots.** I ticked "non-singular means infinitely many solutions" and "a null space of dimension > 0 means infinitely many solutions". Official: only the first. The second statement is equivalent to "in all configurations", which is false at singularities. My reasoning missed that for a redundant robot the null-space condition holds everywhere. (The official solution notes that Gradescope originally accepted both.) Corrected in [`ps2.md`](ps2.md).

**PS2 16.1, waypoints in the plan.** Mine 20, official 16. This counts the plan's own design, not a fact about the task. My plan adds a stationary waypoint after each finger change, because the finger command is a zero-order hold that changes when a waypoint is reached; the extra leg gives the fingers time to close before the arm moves. Both plans stack the blocks; mine is verified for all six orders.

**PS2 Problem 17.** The official timing rule, Δt = √(‖Δp‖² + (rθ)²)/s with r = GRIPPER_RADIUS, is the same as mine (`pose_distance` of `pose_delta`).

**PS3 7.1, grid search.** I gave "resolution complete: all" and "practical: some" but left out the first blank, "complete: none". Filled in.

**PS3, otherwise.** Every other bookwork answer matches, including 14.2 (constrain x, y, pitch; slack along the handle, about it, and about the closing axis), the Problem 17 proofs and the bisection sequence in 17.5, and the 18.4 and 18.5 discussion. The numerical checkpoints match: the QP (0.1534, −0.1005, 0.3492; cost 0.5450), test5 (3.0000, 1.3128; two joints at the limit), test6 (−0.0885 m/s; 0.3000 m), the door's final configuration (identical to four decimals), and the PR2 answers.

**PS4.** Every answer matches, including the problem 2 to 9 numbers, the synthetic normal frame, test3 (0.0200 m, 0.0500 m, yes), and the Problem 12 classifications.
