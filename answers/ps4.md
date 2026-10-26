# Problem Set 4: Grasping

Code: [`psets/ps4/`](../psets/ps4). The letter pick-and-turn runs with `python run_letters.py B B` and `python run_letters.py P M --record`.

## Bookwork

**Problem 1.**
- 1.1 i **C**, ii **D**, iii **A**, iv **B**.
- 1.2 **f_z ≥ 0**, **‖f_t‖ ≤ μ f_z**, **τ = 0 at the contact**. (The tangential force need not equal μ f_z; it only reaches that when sliding.)
- 1.3 **False**: a force at a point off the center of mass has a moment about the center of mass.

Going further: for a marble, measure stiffness and friction (point contact is fine). For a rubber fingertip, measure the patch size and torsional friction (soft-finger or hydroelastic). For a suction cup, measure the seal and vacuum force, which can pull as well as push, so no unilateral model fits.

**Problem 2** (τ_y = z f_x − x f_z).
- 2.1 [τ1, τ2, τ_total] = [**0.1**, **−0.3**, **−0.2**] N m; force [**0**, **4**] N.
- 2.2 **Both** (net force (0, 4) and net torque −0.2).
- 2.3 F_total = [**−0.3**, **−1**, **4**].

Going further: to translate without rotating, place the contacts so their forces pass through the center of mass, or pair them so their moments cancel. To predict a second later, you also need the mass and inertia, the velocities, and how the contact forces change as the body moves.

**Problem 3.** 3.1 **C** (mg sin γ). 3.2 **B** (mg cos γ). 3.3 **D** (μ ≥ tan γ). Going further: it tips first when the line of gravity leaves the base (tan γ > half-width over center-of-mass height) before tan γ reaches μ. Compliance moves the center of pressure. Varying friction would show up as slip at different heights; measuring the center of pressure (force plates) separates the effects.

**Problem 4** (μ = 1/2).
- 4.1 2 · atan(0.5) = **53.13°**.
- 4.2 **Yes**; left **0.5** N, right **0.5** N; **boundary** (0.5 = μ · 1 N).
- 4.3 **No**. The normals now cancel each other and both tangential directions lie along one tilted line, so no combination gives a purely vertical (0, 1) N.
- 4.4 Both admissible: **yes** (tangential 0.5 ≤ 0.5 and 0.25 ≤ 0.5). F_total = [**0.025**, **0**, **−0.25**]. Equilibrium: **no**.
- 4.5 **No**.

Going further: change the contact locations or the fingertip material first, since that is where the margin is lost. Squeezing harder only scales the forces, and a fragile object limits it anyway. With flat faces, the approach direction does not change the normals.

**Problem 5.**
- 5.1 **Yes** (planar, antipodal, μ > 0).
- 5.2 With friction: **yes, yes, yes**. Frictionless: **no, no, yes** (it needs 4 = 3 + 1 contacts in the plane).
- 5.3 **No**: all four normals pass through the center, so no torque about y.

Going further: a grasp without force closure is fine when gravity or the table provides the missing wrench (a tray, a push), or a second hand does. The ideal test ignores force limits, compliance, control bandwidth and uncertain friction.

**Problem 6.** 6.1 **C** (torque about ℓ). 6.2 **3**. 6.3 **Yes** (soft fingers add torsion about ℓ).

**Problem 7** (α = 30°).
- A: antipodal **yes** (normals 20° from the chord); equilibrium **yes**; force closure **yes**.
- B: antipodal **no** (45° > 30°); equilibrium **yes** (both cones point upward, between 15° and 75° above horizontal, so the fingers can hold it up); force closure **no** (no available force points down).

Going further: antipodality is a local test, so the chord leaving the object in A does not matter (but the fingers must not hit the object along the way). In B the normals reveal that the facets slope inward, which finger directions cannot show. With errors of a few degrees, A (10° of margin) survives and B does not. On curved surfaces, a contact landing slightly off the planned point changes the normal, so curvature reduces the margin.

**Problem 8.**
- 8.1 Only "**origin strictly inside the full-dimensional hull**".
- 8.2 **B**.
- 8.3 **C** (−F_g: the fingers must supply −F_g, and the margin is measured from there).
- 8.4 The **first** and **third**.

Going further: run cheap filters first (normal alignment, width, table clearance), then IK and approach collision checks, and finally wrench-margin scores on the survivors. ℓ is roughly the object's size, or the distance from the center of mass to the contacts.

**Problem 9.**
- 9.1 **(2, 8)**, **(1, 6)**, **(8, 15)**. The path is start-1-2-6-8-10-11-14-15-goal; 13, 9 and 12 are not on it. The line 8-15 passes above the obstacle.
- 9.2 **C**, q_near.
- 9.3 **A**, q1: the cost through q1 directly is √2 + √5 = 3.65, against 4.41 through q_near. q_start and q2 are 3.6 units away, outside the 3-unit radius.

## Computational

| Part | Answer |
|---|---|
| 10.2 normal eigenvector | **A**, smallest eigenvalue. Proper rotation always? **No** (the determinant can be −1). |
| 10.3 mustard bottle | 187 normal frames from the 7 × 7 windows |
| 10.4 synthetic patch | ᵂp^N = [**0.1000**, **−0.2000**, **1.3000**] m; +z axis [**0.0000**, **0.0000**, **−1.0000**] (toward the camera) |
| 11.4 test3 | width **0.0200** m; moment arm **0.0500** m; table clearance **yes** |
| 11.5 test4 | pinch displacement (0, 0, 0); world yaw change −30.0000° |
| 11.5 test5 | B onto B: final letter orientation error **0.028°**. My initials, P onto M: **0.497°**. Replay: [`results/ps4_letters_P_M.html`](../results/ps4_letters_P_M.html) |

The sampler casts each surface point's ray inward and keeps the nearest hit. For the B, all 500 samples give a pair, and 338 survive the friction-cone, width, normal-tilt and table filters. The plan has ten keyframes: start, pre-pick, pick (open), pick (close, stationary), lift, turn 30° about G_y at the pinch point, lower, open (stationary), retreat, start.

### 6.4212 Problem 12: analytic antipodal grasps

- 12.1 **B**: ∇E = κ[(p1 − p2)·p′(t1), −(p1 − p2)·p′(t2)], which is zero when the chord is orthogonal to both tangents.
- 12.2 **All four**: coincident points, a vanishing tangent, an outward chord, and a colliding approach all have ∇E = 0, or are invisible to it.
- 12.3 `find_antipodal_pts` builds ∇E and the Hessian symbolically, adds ∇E = 0 as a nonlinear equality with the bounds 0 ≤ t ≤ 2π − ε and t1 − t2 ≥ ε, and restarts from random guesses until `is_antipodal_pair` accepts. Convex? **No**.
- 12.4 Ellipse: (0, π) has eigenvalues [−16, −12], a **maximum (B)**. (π/2, 3π/2) has [−4, 12], a **saddle (C)**.
- 12.5 **A positive-definite Hessian is a strict local minimum** and **a zero eigenvalue is inconclusive**. A maximum of E does not mean the boundary is concave (the ellipse's major axis is convex there), and minimizing E certifies nothing about the real grasp.
- test4 on the gear found all three signatures: a minimum at t = (4.91, 0.67), with eigenvalues (9329, 11796); a maximum at (5.90, 2.48), with (−1773, −1371); and a saddle at (4.90, 1.13), with (−1250, 27655). The energy treats the fingers as two points on a frictionless spring. It ignores friction cones (it only finds exactly normal pairs), finite fingertips (which average the normal over a patch), the gripper's single closing axis and opening limit, and how the grasp behaves when the object moves.

## What the code is doing

- A depth discontinuity mixes two surfaces in one window, and the PCA normal points somewhere in between. A narrow strip of points has two small eigenvalues, so the normal is undetermined, which is why patches whose second eigenvalue is near zero are skipped.
- Opposite normals are both valid PCA results; the sign is chosen to point toward the camera. Tangent axes are arbitrary.
- Translating the camera changes the centroid in world coordinates (it is a point) but not the normal (a direction). Transforming the normal as a point would add the camera's translation to it.
- min_pt_dist and max_pt_dist keep the chord within what the fingers can span, and reject self-intersections.
- The friction-cone test is physics; the normal tilt, moment arm and table check are heuristics. Failures that survive all of them: finger-object collisions, collisions on the approach path, slip under acceleration, and unreachable poses.
- −n1 points into the object, and d points from p1 toward p2. The chord must lie within α of the inward normal, so −n1·d ≥ cos α, and at the other end n2·d ≥ cos α. These are cosines, so both vectors must be unit length.
- The body origin sits FINGER_LENGTH behind the fingertip midpoint along G_y. Rotating the gripper about its own origin swings the midpoint, which is why the turn in `plan` is composed about the pinch point.
