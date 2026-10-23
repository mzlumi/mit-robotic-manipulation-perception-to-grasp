<!-- Source: https://manipulation.csail.mit.edu/spatial.html (retrieved 2026-10-27). Automatic conversion with pandoc; lossy for interactive elements, equations may need the HTML copy. -->

# [Robotic Manipulation](index.html)

Perception, Planning, and Control

[Russ Tedrake](http://people.csail.mit.edu/russt/)

© Russ Tedrake, 2020-2026\
Last modified .

[How to cite these notes, use annotations, and give feedback.](misc.html)\

**Note:** These are working notes used for [a course being taught at MIT](http://manipulation.csail.mit.edu/Fall2026/). They will be updated throughout the Fall 2026 semester.

|  |  |  |
|:---|:--:|---:|
| [Previous Chapter](tactile.html) | [Table of contents](index.html) | [Next Chapter](drake.html) |

# Spatial Algebra

Throughout these notes we've introduced the basic rules of spatial algebra. I find myself looking back at them often! For ease of reference, I've compiled them again here. These make heavy use of our "[monogram notation](pick.html#mongram)".

The Drake documentation also has a very nice summary of the [multibody quantities](https://drake.mit.edu/doxygen_cxx/group__multibody__quantities.html) and how they map to the monogram notation.

# Position, Rotation, and Pose

As we introduced [here](pick.html#spatial_algebra), we use \${}^Bp^A_C\$ to denote a position of point or frame \$A\$ relative to point or frame \$B\$ expressed in frame \$C\$. We use \${}^BR^A\$ to denote the orientation of frame \$A\$ measured from frame \$B\$; unlike vectors, pure rotations do not have an additional "expressed in" frame. Similarly, we use \${}^BX^A\$ to denote the pose/transform of frame \$A\$ measured from frame \$B\$. We do not use the "expressed in" frame subscript for pose; we always want the pose expressed in the reference frame.

The basic rules of spatial algebra are as follows:

- Positions expressed in the same frame can be added when their reference and target symbols match: \begin{equation}{}^Ap^B_F + {}^Bp^C_F = {}^Ap^C_F.\end{equation} Addition is commutative, and the additive inverse is well defined: \begin{equation}{}^Ap^B_F = - {}^Bp^A_F.\end{equation} Those should be pretty intuitive; make sure you confirm them for yourself.
- Multiplication by a rotation can be used to change the "expressed in" frame: \begin{equation}{}^Ap^B_G = {}^GR^F {}^Ap^B_F.\end{equation} You might be surprised that a rotation alone is enough to change the expressed-in frame, but it's true. The position of the expressed-in frame does *not* affect the relative position between two points.
- Rotations can be multiplied when their reference and target symbols match: \begin{equation}{}^AR^B \\: {}^BR^C = {}^AR^C.\end{equation} The inverse operation is also simply defined: \begin{equation}\left\[{}^AR^B\right\]^{-1} = {}^BR^A.\end{equation} When the rotation is represented as a rotation matrix, this is literally the matrix inverse, and since rotation matrices are orthonormal, we also have \$R^{-1}=R^T.\$
- Transforms bundle this up into a single, convenient notation when positions are measured from a frame (and the same frame they are expressed in): \begin{equation}{}^Gp^A = {}^GX^F {}^Fp^A = {}^Gp^F + {}^Fp^A_G = {}^Gp^F + {}^GR^F {}^Fp^A.\end{equation}
- Transforms compose: \begin{equation}{}^AX^B {}^BX^C = {}^AX^C,\end{equation} and have an inverse \begin{equation}\left\[{}^AX^B\right\]^{-1} = {}^BX^A.\end{equation} Please note that for transforms, we generally do *not* have that \$X^{-1}\$ is \$X^T,\$ though it still has a simple form.

# Spatial velocity

Add acceleration

As we introduced [here](pick.html#jacobian), we represent the rate of change in pose using a six-component vector for [*spatial velocity*](https://drake.mit.edu/doxygen_cxx/group__multibody__spatial__vectors.html): \begin{equation}{}^AV^B_C = \begin{bmatrix} {}^A\omega^B_C \\\\ {}^A\text{v}^B_C \end{bmatrix}.\end{equation} \${}^AV^B_C\$ is the spatial velocity (also known as a "twist") of frame \$B\$ measured in frame \$A\$ expressed in frame \$C\$, \${}^A\omega^B_C \in \Re^3\$ is the *angular velocity* (of frame \$B\$ measured in \$A\$ expressed in frame \$C\$), and \${}^A\text{v}^B_C \in \Re^3\$ is the *translational velocity* (along with the same shorthands as for positions). Spatial velocities fit nicely into our spatial algebra:

- Angular velocities add when they are expressed in the same frame: \begin{equation} {}^A\omega^B_F + {}^B\omega^C_F = {}^A\omega^C_F, \end{equation} and have the additive inverse, \${}^A\omega^B_F = - {}^B\omega^A_F.\$
- Rotations can be used to change between the "expressed-in" frames: \begin{equation} {}^A\text{v}^B_G = {}^GR^F {}^A\text{v}^B_F, \qquad {}^A\omega^B_G = {}^GR^F {}^A\omega^B_F.\end{equation}
- Translational velocities compose across frames with: \begin{equation}{}^A\text{v}^C_F = {}^A\text{v}^B_F + {}^B\text{v}^C_F + {}^A\omega^B_F \times {}^Bp^C_F.\end{equation}
- This reveals that additive inverse for translational velocities is not obtained by switching the reference and measured-in frames; it is slightly more complicated: \begin{equation}-{}^A\text{v}^B_F = {}^B\text{v}^A_F + {}^A\omega^B_F \times {}^Bp^A_F.\end{equation}

# Spatial force

As we introduced [here](clutter.html#spatial_force), we define a six-component vector for [*spatial force*](https://drake.mit.edu/doxygen_cxx/group__multibody__spatial__vectors.html), using the monogram notation: \begin{equation}F^{B_p}\_{{\text{name}},C} = \begin{bmatrix} \tau^{B_p}\_{\text{name},C} \\\\ f^{B_p}\_{\text{name},C} \end{bmatrix} \quad \text{ or, if you prefer } \quad \left\[F^{B_p}\_{\text{name}}\right\]\_C = \begin{bmatrix} \left\[\tau^{B_p}\_{\text{name}}\right\]\_C \\\\ \left\[f^{B_p}\_{\text{name}}\right\]\_C \end{bmatrix}.\end{equation} \$F^{B_p}\_{\text{name},C}\$ is the named spatial force applied to a point, or frame origin, \$B_p\$, expressed in frame \$C\$. The name is optional, and the expressed in frame, if unspecified, is the world frame. For forces in particular, it is recommended that we include the body, \$B\$, that the force is being applied to in the symbol for the point \$B_p\$, especially since we will often have equal and opposite forces.

Spatial forces fit neatly into our spatial algebra:

- Spatial forces add when they are applied to the same body in the same frame, e.g.: \begin{equation}F^{B_p}\_{\text{total},C} = \sum_i F^{B_p}\_{i,C} .\end{equation}
- Shifting a spatial force from one application point, \$B_p\$, to another point, \$B_q\$, uses the cross product: \begin{equation} f^{B_q}\_C = f^{B_p}\_C, \qquad \tau^{B_q}\_C = \tau^{B_p}\_C + {}^{B_q}p^{B_p}\_C \times f^{B_p}\_C.\label{eq:spatial_force_shift}\end{equation}
- As with all spatial vectors, rotations can be used to change between the "expressed-in" frames: \begin{equation} f^{B_p}\_D = {}^DR^C f^{B_p}\_C, \qquad \tau^{B_p}\_D = {}^DR^C \tau^{B_p}\_C.\end{equation}

|  |  |  |
|:---|:--:|---:|
| [Previous Chapter](tactile.html) | [Table of contents](index.html) | [Next Chapter](drake.html) |

------------------------------------------------------------------------

|                                                 |                      |
|-------------------------------------------------|---------------------:|
| [Accessibility](https://accessibility.mit.edu/) | © Russ Tedrake, 2026 |
