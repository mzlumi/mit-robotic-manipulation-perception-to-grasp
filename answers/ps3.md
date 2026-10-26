# Problem Set 3: Optimization and Motion Planning

Code: [`psets/ps3/`](../psets/ps3). Logs: [`results/ps3_mobile_base_ik.log`](../results/ps3_mobile_base_ik.log), [`results/ps3_rrt.log`](../results/ps3_rrt.log), [`results/ps3_exact_edge.log`](../results/ps3_exact_edge.log).

## Bookwork

**Problem 1, pseudo-inverse.**
- 1.1 Well-defined everywhere: **(JᵀJ + λI)⁻¹Jᵀ** and **Jᵀ(JJᵀ + λI)⁻¹**. Not well-defined: (JᵀJ)⁻¹Jᵀ (for a redundant arm JᵀJ is 7 × 7 with rank at most 6, so it is never invertible) and Jᵀ(JJᵀ)⁻¹ (it fails at singularities).
- 1.2 (i) q̇ = (0, **1000**) rad/s. (ii) (0, **0.09999**) rad/s. (iii) V = (0, **0.0001**) m/s.

Going further: σ/(σ² + λ) peaks at σ = √λ, where it equals 1/(2√λ). Choosing λ caps the largest joint-velocity gain at 1/(2√λ), and decides below which singular value we stop trying to track.

**Problem 2.**
- 2.1 **OPT can enforce joint limits, velocity bounds and acceleration bounds**, and SRPI cannot. It is not true that SRPI misbehaves near singularities (it is singularity-robust by design), and neither method guarantees reaching the goal.
- 2.2 (i) J⁺V = (**0.2**, **0.4**). (ii) Clipped (0.2, 0.3) gives **0.8** m/s. (iii) QP: (**0.3**, **0.3**), giving **0.9** m/s.

Going further: with a 6D V_d, the least-squares QP may give up on some directions (for example rotation) to do better on others, so the hand leaves the commanded line. Constrain the direction instead, J v = α V_d with 0 ≤ α ≤ 1 and α maximized, so the hand moves along V_d and only the speed is scaled (the formulation in the notes).

**Problem 3, joint centering.**
- 3.1 True: q̇ is a **7-d joint velocity**; V_Gd is a **6-d spatial velocity**, **independent of the rotation representation**; J_G is **6 × 7**; K(q0 − q) has units of **rad/s**; P is **7 × 7**; **J(Pv) = 0**.
- 3.2 (i) (**0.1**, **0.1**). (ii) P12 = **−0.5**. (iii) (**−0.3**, **0.3**). (iv) q̇ = (**−0.2**, **0.4**), J q̇ = **0.2**. (v) Rank **1**.

Going further: joint centering keeps the arm away from limits and odd poses, using redundancy that costs nothing in tracking. In 3.2 it moves joint 2 away from center, but it lowers ‖q − q0‖ along the only direction that does not disturb the task, so it is still doing its job. On the iiwa the null space is 1-D, and at a singularity its dimension goes up by one.

**Problem 4.**
- 4.1 (i) **7.0** rad/s. (ii) **1.0** rad/s. (iii) **velocity**. (iv) **position**.
- 4.2 (i) **[0.4, 0.6]**. (ii) **0.6** rad/s.

Going further: at 2 rad/s heading for the limit, the one-step position constraint allows q̇ up to (q_max − q)/h, but stopping takes v²/(2a) = 0.2 rad, far more than one step's 0.02 rad. The joint overshoots unless the position constraint looks ahead by a braking distance, q + q̇²/(2 a_max) ≤ q_max. If q starts outside its limit, the constraint q + h q̇ ≤ q_max can conflict with the acceleration bound and the QP becomes infeasible. Soften the constraints with slacks, or command zero velocity and recover.

**Problem 5.** Sufficient minimal representations: (x, y) of the circular robot; (x, y) of the cylindrical robot; (x, y, θ) of an arbitrary planar rigid robot; joint angles of the 6-DOF arm; joint angles of the 7-DOF arm. The end-effector pose is not enough, because several arm configurations reach the same pose and collide differently. Going further: the 7th joint lets the elbow swing up or down around a shelf edge while the hand stays put.

**Problem 6.** **19** = 3 (platform x, y, θ) + 2 × 7 (arms) + 2 (one coupled opening per gripper). Going further: no. Locked openings can make a narrow passage impassable that a closed gripper would fit through.

**Problem 7.**
- 7.1 Grid search: resolution complete for **all**; practical for **some** (low-dimensional domains only).
- 7.2 PRM/RRT: complete for **none**; probabilistically complete for **all**; practical for **all**.

Going further: no, failure to find a path is not a proof that none exists. A narrow passage has tiny measure, so the probability of sampling it in a finite run can be very small.

**Problem 8.** Valid: **the convex hull of the Minkowski difference** (obstacle vertices minus robot vertices); **the star algorithm**; **the union with the negated robot**. Invalid: the Minkowski sum without negation, and the union with the robot un-negated. Going further: a disk centered on its reference point is symmetric, so forgetting the negation changes nothing. Testing only disks would hide the bug.

**Problem 9.** **Non-holonomic**, and **rotate, straight, rotate reaches any pose**. Going further: yes. Turn 90° left, drive, turn 90° right. A car cannot turn in place, so it needs a parallel-parking sequence of forward and reverse arcs.

**Problem 10.** True: complete planners are **exponential in the number of DOF** (worst case), and the best probabilistically complete planners can solve 7-DOF problems **in under a millisecond** (for example vectorized RRT-Connect). Complete planners are polynomial, not exponential, in the number of obstacles for fixed DOF, and tens of seconds is not typical. Going further: a narrow passage has a small probability of being sampled and connected, so the expected time scales inversely with it.

**Problem 11, grids.**
- 11.1 True: **positive clearance means a fine enough grid finds a path**; **failure at one granularity does not rule out a finer one**; **cells grow exponentially in the dimension**. Not true: shortest in continuous space; a coarser grid could succeed where a finer one fails (with nested cells marked occupied if any part is not free, a free coarse cell implies free fine cells); exponential in N (it is polynomial, Nᵈ).
- 11.2 True: **same number of cells**; **top and bottom cells must be neighbours**; **leaving those neighbours out can report failure at every granularity**. Leaving them out cannot produce an unfollowable path (it only removes edges), and a joint with limits needs no wrap-around.
- 11.3 True: **admissible**, and **stays admissible for any obstacle arrangement**. Not true: with unit-cost moves, a diagonal step costs 1 but covers √2 cell widths of Euclidean distance, so the heuristic overestimates. For the unlimited joint, coordinate distance can exceed the wrapped distance.
- 11.4 100⁷ = **10¹⁴** cells.

Going further: yes. A cell whose center is free can still contain an obstacle corner, so a path through cell centers can clip an obstacle. Example: a thin wall that passes between cell centers.

## Computational

| Part | Answer |
|---|---|
| 12.2 min x² − 2x on [−1, 3] | x* = 1.0000 (interior) |
| 12.4 the QP | x* = (**0.1534**, **−0.1005**, **0.3492**), cost **0.5450** |
| 13.2 pseudo-inverse, 0.1 m/s in +x | For 4.5 s the hand moves exactly at 0.1 m/s in +x (x from 0 to 0.45 m, y and z unchanged). Then the arm approaches a singularity, and the joint speeds jump from about 0.2 to 2.4 rad/s and later to 8 rad/s. The hand leaves the line (y swings from −0.47 to +0.17 m, z drops by 0.2 m). It ends up folded back at x ≈ 0.48 m, creeping, never reaching the commanded line again. J⁺ asks for unbounded joint speed in a direction the arm can barely move. |
| 13.3 velocity-limited QP | The same 4.5 s of exact tracking. After that, no joint exceeds 3 rad/s (peaks of 2.4 to 2.6 rad/s), and the hand keeps moving outward as far as it can (x reaches 0.79 m at 15 s, the edge of the reach), trading some error in y and z for progress in x, without the violent reversal. |
| 13.4 test5 | v[:2] = (**3.0000**, **1.3128**); **2** of 7 joints at the 3 rad/s limit; max \|v\| = **3.0000** (Clarabel and SNOPT agree, so the solution is unique) |
| 13.6 test6 (wall) | hand x velocity **−0.0885** m/s; predicted next x **0.3000** m (the wall constraint is active) |
| 14.2 exact constraints | **x position, y position, pitch** (slack: z, sliding along the handle; roll, about the closing axis through both contacts; yaw, about the handle axis) |
| 14.3 the cost | **bias toward a natural configuration** and **penalize displacement from q_nom** |
| 14.4 frames Ā, B̄ | **frames existing in the plant**; **reference frames with ĀR_A and B̄R_B constant**; **express task frames A and B in existing plant frames**; **define constraints on arbitrary axes rigidly attached to bodies** |
| 14.5 point Q | **fixed relative to frame B**; **we constrain its position in frame A**; enforces **C** |
| 14.7 final keyframe | q = **[−1.2018, 1.3960, 1.3571, −0.6645, −2.8583, −2.0944, 1.5812]** (`check_final_configuration` returns True); the door opens 79.4° in simulation |
| 15.2 restarts (fixed base) | 10 of 20: **C** |
| 15.3 cost | **C**, distance from the nominal configuration |
| 15.4 why hard | **15 DOF** (torso and both arms with the base fixed) and **it has to avoid the shelves** |
| 15.5 reachable only with the base moving | **C**, (−0.83, 1.30, 1.40) |
| 15.6 out of reach everywhere | **D**, (−0.83, 0.18, 2.30) (above the arm's reach for any base pose) |
| 15.7 base pose with no solution | **C**, [−1.8, 0.2, 0] |
| 16.3 RRT | succeeds in 4 of 10 trials within 1000 iterations (98 to 535 iterations when it does) |
| 16.5 RRT against RRT-Connect | RRT-Connect solved 10 of 10 in 11 to 84 iterations and 0.04 to 0.16 s; RRT solved 4 of 10, taking 0.1 to 3.3 s |
| 16.6 finer recheck | An edge is checked only at configurations 1° apart per joint. A thin obstacle or a corner can sit entirely between two samples, so the finer 0.1° walk can find a collision the planner never saw (it happened on the test3 path). Checking every 0.1° does not guarantee a continuous path is free either. Any finite set of samples can miss a thin enough obstacle unless it comes with a clearance bound, which is what Problem 17 provides. |
| 16.7 | Only "**we should use different spacing for different joints**". The pattern depends on the configuration. The table is roughly 1/‖column of J‖ per joint, neither J nor J⁻¹. |
| 16.8 | **B**: no warning / it approaches zero |
| 16.9 | Replay of the RRT-Connect path: run `python rrt_planning.py` with test3 |

### 6.4212 Problem 17: exact edge checking

**17.1 Proof.** Suppose a point p of the robot touches an obstacle at parameter s ∈ (0, 1) along the segment from q to q′. At q, p was at least η(q) from every obstacle, so it has travelled at least η(q) by then. At q′ it is again at least η(q′) away, so it must still travel at least η(q′). Its total path length is therefore at least η(q) + η(q′). But by the motion bound, no point travels farther than ρ‖q − q′‖ over the whole segment. If ρ‖q − q′‖ < η(q) + η(q′), the contact is impossible, and the segment is free.

**17.2.** A point on link 1 at distance s ≤ 1 from the base moves s|Δθ1| ≤ |Δθ1|. A point on link 2 is r = p1(θ1) + s u(θ1 + θ2) with s ≤ 1. Along the segment, its speed is at most |θ̇1| ‖p1 + s u‖ + |θ̇2| s ≤ 2|θ̇1| + |θ̇2|, since ‖p1 + s u‖ ≤ 2. Integrating gives a distance of at most 2|Δθ1| + |Δθ2| = (2, 1)·(|Δθ1|, |Δθ2|) ≤ √5 ‖Δθ‖₂ by Cauchy-Schwarz. So ρ = √5.

**17.3.** The fixed-step checker samples 0°, 10°, ..., 90°. The closest samples to the post are 40° and 50°, where the clearance is 1.697 · sin 5° − 0.1 = 0.048 > 0. It reports **free**, which is **wrong**: at 45° the arm penetrates the post (η = −0.1).

**17.4.** The ends have η = 1.1 each, and ρ‖Δq‖ = √5 · π/2 = 3.51 > 2.2, so the test fails. The checker bisects: η(45°) = −0.1 ≤ 0, so it **reports a collision**, correctly.

**17.5.** It evaluates η at 0° and 90° (1.5 each; 3.51 > 3.0, so it bisects), then at 45° (0.163 > 0). For [0°, 45°]: 1.76 > 1.663, so it bisects at 22.5° (0.771). Then [0°, 22.5°]: 0.878 < 2.271, certified; [22.5°, 45°]: 0.878 < 0.934, certified. By symmetry it evaluates 67.5° and certifies the other half. **Free**, after evaluating 0°, 90°, 45°, 22.5° and 67.5°.

**17.6.** A segment that grazes an obstacle runs forever: one where η reaches exactly 0 at a single interior point that is never a dyadic sample (for example the post moved so the arm just touches it at 45° + a tiny irrational offset). Pieces next to that point need ρΔ < η_a + η_b, which tends to 0, so they are bisected forever. With the margin δ, every accepted sample has η > δ, so any piece with Δ < 2δ/ρ satisfies ρΔ < 2δ < η_a + η_b and is certified on the spot. Pieces shorter than 2δ/ρ are never subdivided, and the recursion depth is bounded. The same argument with Δ = ε shows that fixed-spacing checking is sound if every sample (including both ends) has clearance greater than ρε/2.

### 6.4212 Problem 18: exact edge checking on the iiwa

- **18.1** `motion_bound(r) = ‖r‖₂` (each joint moves a point at most r_j|Δq_j|, then Cauchy-Schwarz). Joint radii are [1.121, 1.121, 0.722, 0.722, 0.322, 0.322, 0.077] m, so **ρ = 1.942 m/rad**. The clearance is 6.4 cm at q_start and 1.3 cm at q_goal.
- **18.2** `is_certified` is the theorem (with both clearances positive). `exact_edge_is_free` bisects, using an explicit stack of (q_a, η_a, q_b, η_b), so η is evaluated once per configuration and `collide` is asked about every sample. On a path from the fixed-step planner, 2 of its 22 edges are rejected (258 clearance and 256 collision queries; the fixed-step recheck uses 237 collision queries, and is about 40 times faster because a distance query costs more than a yes/no one).
- **18.3** With the exact check inside RRT-Connect, **0 of 5** paths fail the 0.1° recheck (planning takes 0.5 to 0.9 s). Robot-obstacle collisions are certified between samples, so they cannot fail. Self-collisions are still checked only at samples and could in principle slip through. None did here.
- **18.4** Making EDGE_STEP sound requires 4.5 cm of clearance at every sample, but q_goal itself has only 1.3 cm. The planner would reject its own goal and fail outright. A uniform margin is far too conservative where the task needs the arm close to things. The adaptive check only spends samples where clearance is small.
- **18.5** Including the link 5 to link 7 pair would put η ≤ 1.9 cm at q_start everywhere along the path, and with ρ = 1.94 that forces tiny pieces (about 2 cm/1.94 ≈ 0.01 rad), so many more queries. The better bound is to measure link 7's motion in link 5's frame, where only joints 6 and 7 move it: ρ₅₇ = ‖(r₆, r₇)‖ ≈ ‖(0.322, 0.077)‖ = 0.33 m/rad, about six times smaller.

## What the code is doing

- In 13.5 the active constraints are the wall (the x upper bound, hence predicted x = 0.3000 exactly) and, when the command is fast, some of the velocity bounds.
- Another fix for 13.3 is damping (Jᵀ(JJᵀ + λI)⁻¹) or scaling the command near singularities.
- When the QP becomes infeasible, command zero velocity (the arm is position-controlled, so stopping is safe). `_solve_for_velocity` does exactly that.
- Deleting the joint-centering cost in the door IK would leave the redundancy free, so solutions could jump between branches from one keyframe to the next. Warm-starting from the previous keyframe keeps neighbouring solutions on the same branch and the spline short.
- A free base adds variables but enlarges the feasible set. The fixed base is a special case, so with a free base a solution is much easier to find.
- RRT-Connect gains from growing toward the other tree with repeated extensions (greedy connect) instead of single random steps.
- EDGE_STEP = 1° can return paths that are not collision-free (thin obstacles between samples). It is also slow on long edges.
- GOAL_BIAS = 0 makes the tree reach the goal only by chance. GOAL_BIAS = 1 makes it a greedy straight-line planner that keeps hitting the same obstacle.
- A continuously rotating joint needs interpolation along the shorter arc (wrap Δq into [−π, π)) and a distance that wraps.
