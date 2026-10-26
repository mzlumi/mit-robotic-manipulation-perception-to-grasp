# python libraries
# ruff: noqa: F401
import sys
from pathlib import Path

# The shared modules live in utils/, a sibling of this directory, and these
# scripts are run from inside their own pset directory -- so the repository
# root has to go on the path before anything imports from it.
sys.path.append(str(Path(__file__).resolve().parents[1]))

import mpld3
import numpy as np
from manipulation import running_as_notebook
from manipulation.exercises.pick.plot_planar_manipulator import (
    plot_planar_manipulator,
    plot_two_planar_manipulators,
)
from pydrake.all import RigidTransform, RotationMatrix

from utils.plotting import plt

######################################################################
## Display code
######################################################################


def test_plot_fk():
    q = 2 * np.pi * np.random.rand(2)
    # q = [np.pi/2, np.pi/2]
    plot_planar_manipulator(q, forward_kinematics(q))


def test_plot_ik():
    theta = 2 * np.pi * np.random.rand()
    r = 2 * np.random.rand()
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    q1, q2 = inverse_kinematics(x, y)
    print(q1, q2)
    plot_two_planar_manipulators(q1, q2, np.array([x, y]))


def test_plot_manipulability(n: int = 200):
    """Heat map of manipulability(Jacobian(q)) over the joint space."""
    grid = np.linspace(-np.pi, np.pi, n)
    Q0, Q1 = np.meshgrid(grid, grid)
    w = np.array(
        [manipulability(Jacobian(np.array(q))) for q in zip(Q0.ravel(), Q1.ravel())]
    ).reshape(Q0.shape)

    plt.figure()
    # Nonnegative magnitude, anchored at zero. viridis has no white at either
    # end, so low manipulability can't be confused with empty background.
    plt.pcolormesh(Q0, Q1, w, cmap="viridis", vmin=0, vmax=np.max(w), shading="auto")
    plt.colorbar(label="manipulability  |det(J)|")
    # Near-zero level set = the singular configurations
    # plt.contour(Q0, Q1, w, levels=[0.05 * np.max(w)], colors="0.35", linewidths=1)

    ticks = np.array([-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi])
    labels = [r"$-\pi$", r"$-\pi/2$", "$0$", r"$\pi/2$", r"$\pi$"]
    plt.xticks(ticks, labels)
    plt.yticks(ticks, labels)
    plt.xlabel("$q_0$")
    plt.ylabel("$q_1$")
    plt.title("Manipulability over joint space")
    plt.gca().set_aspect("equal", adjustable="box")


def test_plot_manipulability_task_space(n: int = 200):
    """Heat map of manipulability over task space, at the IK configuration.

    The two IK solutions are elbow-up/elbow-down mirrors (q1 and -q1), so
    |det(J)| is the same for both; we take the first.
    """
    grid = np.linspace(-2.2, 2.2, n)
    X, Y = np.meshgrid(grid, grid)
    w = np.array(
        [
            manipulability(Jacobian(inverse_kinematics(x, y)[0]))
            if x**2 + y**2 <= 4.0  # unreachable outside, IK has no solution
            else np.nan
            for x, y in zip(X.ravel(), Y.ravel())
        ]
    ).reshape(X.shape)

    plt.figure()
    plt.pcolormesh(X, Y, w, cmap="viridis", vmin=0, vmax=np.nanmax(w), shading="auto")
    plt.colorbar(label="manipulability  |det(J)|")
    # Unreachable cells are nan, so they show the axes background: a neutral
    # gray that appears nowhere in the colormap.
    plt.gca().set_facecolor("0.85")
    # Outer boundary of the reachable set
    th = np.linspace(0, 2 * np.pi, 200)
    plt.plot(2 * np.cos(th), 2 * np.sin(th), "-", color="0.5", linewidth=1)

    plt.xlabel("$x$")
    plt.ylabel("$y$")
    plt.title("Manipulability over task space")
    plt.gca().set_aspect("equal", adjustable="box")


######################################################################
## Code for students to study
######################################################################


def X2D(x, y, th):
    return RigidTransform(RotationMatrix.MakeZRotation(th), [x, y, 0])


def X2p(X):
    return X.translation()[:2]


######################################################################
## Code for students to implement
######################################################################

## One of the "bookwork problems"
X_WA = X2D(1, 2, np.pi / 6)
X_WB = X2D(2, 3, -np.pi / 4)
X_WC = X2D(6, 4, -np.pi * 2 / 3)


# Link lengths, as encoded in the joint origins of Problem 6.
L = 1.0


# Use Drake tools!
def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """Position of the two-link arm's end-effector frame C in base frame a."""
    X_a0 = X2D(0, 0, 0)  # joint 0 on the base
    X_0b = X2D(0, 0, q[0])  # rotation of joint 0
    X_b1 = X2D(L, 0, 0)  # joint 1 on link b
    X_1c = X2D(0, 0, q[1])  # rotation of joint 1
    X_c2 = X2D(L, 0, 0)  # end-effector frame on link c
    return X2p(X_a0 @ X_0b @ X_b1 @ X_1c @ X_c2)


def inverse_kinematics(x: float, y: float) -> tuple[np.ndarray, np.ndarray]:
    """The two joint configurations that put the end effector at (x, y)."""
    # r^2 = |p|^2 = 2 L^2 (1 + cos q1), so q1 = +/- acos(r^2 / 2L^2 - 1).
    # Clip so that points on the boundary don't fail on rounding.
    c1 = np.clip((x**2 + y**2) / (2 * L**2) - 1, -1.0, 1.0)
    solutions = []
    for q1 in (np.arccos(c1), -np.arccos(c1)):
        # p = R(q0) [L + L cos q1, L sin q1], so q0 is the direction of p minus
        # the direction of the elbow-frame vector.
        q0 = np.arctan2(y, x) - np.arctan2(L * np.sin(q1), L + L * np.cos(q1))
        solutions.append(np.array([q0, q1]))
    return solutions[0], solutions[1]


def Jacobian(q: np.ndarray) -> np.ndarray:
    """The 2 x 2 translational Jacobian of the two-link arm at q."""
    # x = L cos q0 + L cos(q0 + q1), y = L sin q0 + L sin(q0 + q1)
    s0, c0 = np.sin(q[0]), np.cos(q[0])
    s01, c01 = np.sin(q[0] + q[1]), np.cos(q[0] + q[1])
    return np.array(
        [
            [-L * s0 - L * s01, -L * s01],
            [L * c0 + L * c01, L * c01],
        ]
    )


def manipulability(J):
    """Yoshikawa manipulability measure for a 2 x 2 Jacobian."""
    return np.abs(np.linalg.det(J))


if __name__ == "__main__":
    # test_plot_fk()
    # test_plot_ik()
    # test_plot_manipulability()
    # test_plot_manipulability_task_space()
    plt.show()
