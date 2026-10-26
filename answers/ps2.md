# Problem Set 2: Transforms and Kinematics

Code: [`psets/ps2/`](../psets/ps2). Numbers come from `python checkpoints.py` (planar arm, grasps, test3, the stack in all six orders, and 0.5 s legs) and `python extras_64212.py`.

## Bookwork

**Problem 1, roll-pitch-yaw (r, p, y) = (90°, 0°, 90°).**
- 1.1 The world x̂ goes onto the world **ŷ** axis. It is a single rotation of **120°** about **(1, 1, 1)/√3**. R maps x → y → z → x.
- 1.2 (i) **R_Z(y) R_Y(p) R_X(r)**. (ii) As intrinsic rotations: about **z, then y, then x**. (iii) **No**.
- 1.3 (i) M1: **y**, M2: **z**, M3: **x**. (ii) Columns (0, 1, 0), (0, 0, 1), (1, 0, 0). (iii) **Rᵀ** and **R_X,−r R_Y,−p R_Z,−y**. (iv) Columns (0, 0, 1), (1, 0, 0), (0, 1, 0).
- 1.4 (i) q1: **y**, q2: **z**, q3: **x**. (ii) q = q_z(90°) ⊗ q_x(90°) = **(0.5, 0.5, 0.5, 0.5)**. (iii) **(0.5, −0.5, −0.5, −0.5)**. Going further: q = (cos 60°, sin 60° · (1, 1, 1)/√3) is exactly the 120° rotation about (1, 1, 1)/√3 found in 1.1.

**Problem 2, representations.**
- 2.1 Unit quaternion: **A**. Euler angles: **B**. Axis-angle: **B**. Rotation matrix: **C**.
- 2.2 True: quaternions compose with fewer operations (16 multiplications against 27); both are easy to invert (conjugate, transpose); quaternions are more compact. False: easier to apply to vectors; one-to-one (q and −q are the same rotation).
- 2.3 **q1 represents the same rotation as q**; **q2 represents the inverse of q**.
- 2.4 The **first** option: [[0, −sin(r+y), −cos(r+y)], [0, cos(r+y), −sin(r+y)], [1, 0, 0]] (checked symbolically).
- 2.5 **A change in r is not distinguishable from a change in y.**
- 2.6 **(0, 0, −3π/2)** and **(0, 0, 5π/2)**.
- 2.7 Going further: (0, −π/2, 0) is an ordinary rotation (90° about −y). Nothing about the rotation is special. What goes singular is the map from (r, p, y) to rotations, whose Jacobian loses rank there, because r and y become the same rotation axis.
- 2.8 No, if the gripper's approach axis is its body x axis: pointing straight down is then exactly pitch = ±π/2, the gimbal-lock pitch. Store a quaternion or a rotation matrix. To keep three parameters, use a local chart: a small rotation vector about a nominal "pointing down" orientation, R = R_nominal exp([w]), which is singular only far from where you operate.
- 2.9 SO(3) is a compact 3-manifold, and no compact 3-manifold is homeomorphic to an open subset of R³. So no three-number parameterization can cover every rotation smoothly and one-to-one: every such map has singular points or discontinuities somewhere.

**Problem 3, interpolating rotations.**
- 3.1 **Y1, Y3, Y4** (for Y4, 90° is one of the two valid midpoints). Not correct for all inputs: it fails on Y2. The trouble is in the numbers written down (wrap-around), not in the rotations.
- 3.2 **Y1, Y2, Y3**. It averages the two unit vectors (cos θ, sin θ) and takes the direction of the result, so it is the midpoint on the circle. It has no answer when the vectors are opposite (Y4), and no method could do better: both 90° and −90° are equally halfway.
- 3.3 **P1** only. Averaging axes fails when the "same" axis is written with opposite signs (P2, P4) or is undefined (P5, the identity). Averaging angles fails even when the axes are fine: in P3, the true midpoint of RotX(90°) and RotY(90°) is a 70.5° rotation about (1, 1, 0)/√2, not 90°.
- 3.4 **P1, P2, P3, P5**. This is the 2D construction of 3.2 one dimension up: the chord midpoint on S³, projected back to the sphere. It fails on P4 because q1 and q2 lie in opposite hemispheres. The midpoint between q1 and q2 is the long way around, which is the identity.
- 3.5 **P1 to P5**, all of them. The sign test picks the representative of q2 in the same hemisphere as q1 (the double cover). For a quarter of the way, renormalizing ¾q1 + ¼q2 is not uniform in angle. Use slerp: q(t) = sin((1−t)Ω)/sin Ω · q1 + sin(tΩ)/sin Ω · q2. Method 3.5 is slerp at t = ½ with the shorter arc chosen.

**Problem 4, transforms** (computed with Drake's RigidTransform):
- 4.1 ᴮp^C_W = **(4.000, 1.000)**
- 4.2 ᴮp^C_B = **(2.121, 3.536)**
- 4.3 ᶜp^B_C = **(2.866, −2.964)**
- 4.4 ᴮp^C_A = **(3.964, −1.134)**
- 4.5 ᴬp^C_B = **(2.121, 4.950)**
- 4.6 ᴬX^B = **(1.366, 0.366, −1.309)**, where −1.309 rad = −75°
- 4.7 Q in the world: **(6.366, 2.634)**

Going further:
- 4.8 ᶜp^B_C = −ᴮp^C_B exactly when B and C have the same orientation.
- 4.9 No. ᴮX^A = (−R(−θ)(x, y), −θ). The naive answer is right only when θ = 0.
- 4.10 Sliding everything by (10, 10) changes only 4.7 (the world position of Q). Rotating the whole scene about the world origin changes 4.1 (expressed in W) and 4.7, and leaves the relative quantities 4.2 to 4.6 unchanged.

**Problem 5, gripper and object.** 5.1 **[0, 0.3, 0.1]**: the object lies mostly along the gripper's green y axis. 5.2 **[0.3, 0, −0.1]**.

Going further:
- 5.3 All three options have the same length because a change of frame is a rotation, which preserves length. So a magnitude check cannot tell a right frame from a wrong one.
- 5.4 Rolling the gripper changes ᴳp^O_G but not ᴳp^O_W.
- 5.5 ᴳp^O_O is just ᴳp^O_W rotated by R_OW, and R_OW is the identity here, so it equals 5.2.

**Problem 6, planar arm.**
- 6.1 The **second** option: ᵃX₀ ⁰X_b(q0) ᵇX₁ ¹X_c(q1) ᶜX₂.
- 6.2 **2** factors depend on q.
- 6.3 **(2, 0)** at q = 0, and **(1, 1)** at (π/2, −π/2).
- 6.4 Going further: ᴾX_p is the joint origin (ᵃX₀, ᵇX₁), ᵖX_c(q) is the joint rotation, and ᶜX_C = I. Without loops, any constant offset on the child can be absorbed into the next joint's origin, so choosing ᶜX_C = I loses nothing.

**Problem 7.**
- 7.1 **The coordinate transforms of the links and end effector.**
- 7.2 The first three: all solutions, detecting unreachable poses, and hard to derive.
- 7.3 The **second** option: sweep one joint and run analytic 6-DOF IK for the rest.

Going further:
- 7.4 The first option fails because fixing the last six joints with the root free is not a 6-DOF IK problem. The third fails because the halves are coupled through the middle joint's pose, and the 3-joint problems are over-constrained.
- 7.5 A numerical solver returns its best local residual, and "not converged" cannot be told apart from "unreachable".
- 7.6 Independent IK can switch solution branches between neighbouring poses, causing big jumps. Fix it by warm-starting from the previous solution, adding a cost toward it, or planning in joint space with continuity constraints.

**Problem 8.**
- 8.1 **The spatial velocity of any point given the joint velocities.**
- 8.2 The **second** option: a spatial velocity, independent of the orientation representation.
- 8.3 Only "**n is the number of robot degrees of freedom**".
- 8.4 **3** rows, **N** columns.
- 8.5 The point is on joint j's axis; the point is on an ancestor link; the point is on a different branch. (Being at the edge of the workspace makes J rank-deficient, but it does not zero out a column.)

Going further:
- 8.6 The z row is zero, so rank J ≤ 2.
- 8.7 A zero row is a task direction that no joint can ever move, the same in every configuration. A singularity is configuration-dependent.
- 8.8 The other four rows are the angular velocities about x and y (always zero for a planar arm), the angular velocity about z (q̇0 + q̇1, which we chose not to track), and the z velocity (zero).

**Problem 9.**
- 9.1 **1**.
- 9.2 The **first** (in non-singular configurations, infinitely many) and **third** (null space of dimension > 0 gives infinitely many).
- 9.3 **When the Jacobian is rank-deficient.**
- 9.4 **All five.**

Going further:
- 9.5 At the straight elbow, motion along the arm (radial) is impossible. Motion perpendicular to it is still available.
- 9.6 Joint velocities blow up like 1/σ: the arm whips and overshoots. Fixes: damped least squares, joint velocity limits (the QP of PS3), or slowing the command near the singularity.
- 9.7 Two reasons to shape the null-space motion: keep joints away from their limits and singularities (joint centering), and avoid obstacles or keep the elbow out of the way.

**Problem 10.** Only "**DiffIK serves to map end-effector velocities to joint velocities.**" The "well-behaved except on a measure-zero set" claim is false in practice. 10.1: the trouble is not the singular set itself but its neighbourhood, where ‖J⁺‖ is huge, and trajectories pass through such neighbourhoods all the time (any arm that reaches out to full extension does).

## Computational

| Part | Answer |
|---|---|
| 11.1 FK(0.3, 0.2) | **(1.8329, 0.7749)** |
| 12.1 IK(−1, −0.5) | **(−3.66, 1.96)** and **(−1.70, −1.96)**. The other options give (−1.00, 1.00), (0.69, −0.87) and (−1.26, −0.65). |
| 12.2 Three-link planar arm | **∞** |
| 13.1 J(0.3, 0.2) | ∂x/∂q0 = **−0.7749**, ∂x/∂q1 = **−0.4794**, ∂y/∂q0 = **1.8329**, ∂y/∂q1 = **0.8776** |
| 13.2 Singular configurations | **(0.4, 0)** and **(0.2, π)** (det J = L² sin q1) |
| 14.1 Grasp translation ᴼp^G | **(0.0000, 0.0000, 0.1330)**: pads at half the block height (0.025 m) plus GRIPPER_REACH (0.108 m) |
| 14.2 Blue block, world ᵂp^G | **(0.7000, 0.0000, 0.1330)** |
| 15.4 test3("blue") end time | **8.0 s** (two legs of 4 s: start, target, target) |
| 15.5 test3("blue") final q | **[−0.14, 1.14, 0.29, −1.00, −0.31, 1.04, −1.43]** (gripper within 0.1 mm of the target) |
| 15.6 test3("red") final q | **[−0.38, 1.01, −0.03, −1.24, 0.03, 0.90, −0.41]** |
| 16.1 Waypoints in plan("red", "blue", "yellow") | **20** (10 per pick-and-place; 84 s) |
| 16.2 Legs of 0.5 s | **The motion is too aggressive, and the stack is no longer reliable.** In simulation the yellow block ends up lying on its side, 0.57 m off target. |

How the plan works: for each block it goes pre-grasp → grasp → close → hold → lift → pre-place → place → open → hold → retreat. The fingers are a zero-order hold that changes when a waypoint is reached, so each finger change gets a second, stationary waypoint, which gives the fingers a whole leg to move before the arm sets off. Block c is stood on end by turning it −90° about its own y axis, so its long axis points up and the fingers point away from the robot. The arm then carries it in from its own side, approaching sideways with the same top grasp it picked it up with, and retreats backwards before rising.

Verified in simulation for **all six orders** of the three blocks. In every order, b ends within 0.1 mm and 0.5° of the pose exactly on top of a, and c ends upright (long axis at 0.999 or better along z) with its center 75 mm above b's top surface and within 6 mm of b's center. Replay: [`results/ps2_stack_red_blue_yellow.html`](../results/ps2_stack_red_blue_yellow.html).

### 6.4212 Problems 17 to 19

- **17, constant speed.** `speed_waypoint_times` gives each leg the duration pose_distance / speed, where pose_distance weighs rotation by GRIPPER_RADIUS (the distance a point on the rim moves). A move that translates 0.22 m, turns 90° in place, and translates again runs at a measured 0.0500 m/s (the 10th to 90th percentile is also 0.0500) for a requested 0.05 m/s, including the pure rotation. [figure](../psets/ps2/figures/p17_constant_speed.png)
- **18, smooth operation.** PiecewisePose.MakeLinear gives constant velocity on each leg, so the speed profile is a rectangle per leg, with jumps at every waypoint. The commanded acceleration is infinite there, and the controller rounds the corners. Tracing a 0.2 m square in a vertical plane gives max / RMS deviation of 0.7/0.1 mm at 4 s per leg, 2.8/0.6 mm at 1 s, 8.7/2.6 mm at 0.5 s and 32/16 mm at 0.25 s. That is a clear speed-accuracy trade-off, caused by the velocity discontinuities and the arm's tracking lag. A smoother time-scaling (trapezoidal or minimum-jerk per leg, or a spline through the waypoints) would reduce it. [figure](../psets/ps2/figures/p18_square_speed_accuracy.png)
- **19, painter.** A horizontal circle of radius 0.1 m drawn as 36 chords of 0.5 s each: the radius error is at most 0.8 mm (0.4 mm of that is the chord sag of the polygon) and the height error at most 0.6 mm. [figure](../psets/ps2/figures/p19_painter.png)

## What the code is doing

- A side grasp would use G_y horizontal, pointing at the block, with the pre-grasp backed off along −G_y rather than lifted.
- `arm_jacobian_columns` picks the seven columns of the iiwa's joints out of the 6 × 27 Jacobian of a plant that also contains the gripper fingers and three free blocks. Those other velocities are not ours to command.
- JointVelocity needs no state: it is a pure function of (V_WG, q). It needs q because the Jacobian depends on the configuration.
- Interpolating poses linearly needs a geodesic interpolation of the rotation (slerp), which is what MakeLinear does. MakeDerivative differentiates the piecewise pose to a spatial velocity, angular part first. Its magnitude is piecewise constant.
- Without the Integrator, the station would get velocities on a position port. Without `start_the_integrator`, it would start from q = 0 and the first command would snap the arm toward zero.
