"""PS3: the QPs, differential IK, RRT on a toy problem, and the exact edge
checker on the planar arm of Problem 17."""

import random

import numpy as np
import pytest
from manipulation.exercises.trajectories.rrt_planner.robot import ConfigurationSpace, Range

import diffik_optimization as dik
import exact_edge_check as eec
import intro_to_qp as qp
import rrt_planning as rrt


def test_scalar_and_vector_qps():
    assert qp.scalar_example_interior() == pytest.approx(1.0, abs=1e-6)
    x = qp.solve_the_qp()
    assert qp.A_EQ @ x == pytest.approx(qp.B_EQ, abs=1e-8)
    assert np.all(np.abs(x) <= qp.BOX + 1e-8)
    assert x == pytest.approx([0.15343917, -0.1005291, 0.34920635], abs=1e-6)


def test_diffik_qp_checkpoint():
    v = dik.DiffIKQP(dik.J_G_GIVEN, dik.V_G_DESIRED_FAST, dik.Q_NOW_GIVEN, dik.V_NOW_GIVEN, dik.P_NOW_GIVEN)
    assert np.abs(v).max() <= 3.0 + 1e-6
    assert v[:2] == pytest.approx([3.0, 1.3128], abs=1e-4)
    assert np.sum(np.isclose(np.abs(v), 3.0, atol=1e-4)) == 2


def test_wall_keeps_the_predicted_hand_inside_the_box():
    v = dik.DiffIKQP_Wall(dik.J_G_GIVEN, dik.V_G_DESIRED_GIVEN, dik.Q_NOW_GIVEN, dik.V_NOW_GIVEN, dik.P_NOW_GIVEN)
    p_next = dik.P_NOW_GIVEN + dik.WALL_H * (dik.J_G_GIVEN @ v)[3:]
    assert p_next[0] == pytest.approx(0.3, abs=1e-6)
    assert (dik.J_G_GIVEN @ v)[3] == pytest.approx(-0.0885, abs=1e-4)


class Wall:
    """A unit square with a wall at x = 0.5 that leaves a gap above y = 0.8."""

    def collide(self, q):
        return 0.45 < q[0] < 0.55 and q[1] < 0.8


def toy_problem():
    cspace = ConfigurationSpace(
        [Range(0.0, 1.0), Range(0.0, 1.0)],
        lambda dq: float(np.hypot(*dq)),
        2 * [rrt.EDGE_STEP],
    )
    return rrt.PlanningProblem(cspace, np.array([0.1, 0.1]), np.array([0.9, 0.1]), Wall())


@pytest.mark.parametrize("planner", [rrt.rrt_planning, rrt.rrt_connect_planning])
def test_planners_return_collision_free_paths(planner):
    random.seed(3)
    np.random.seed(3)
    problem = toy_problem()
    path, _ = planner(problem, max_iterations=3000)
    assert path is not None
    assert path[0] == problem.start and path[-1] == problem.goal
    assert all(problem.edge_is_free(a, b) for a, b in zip(path, path[1:]))
    assert max(q[1] for q in path) > 0.8  # it had to go through the gap


def test_extend_statuses():
    problem = toy_problem()
    tools = rrt.RRT_tools(problem)
    tree = tools.new_tree(problem.start)
    status, node = rrt.extend(tools, tree, (0.1, 0.105))
    assert status == "Reached" and node.value == (0.1, 0.105)
    status, node = rrt.extend(tools, tree, (0.1, 0.9))
    assert status == "Advanced" and node.parent is not None
    blocked = rrt.RRT_tools(problem)
    t2 = blocked.new_tree((0.44, 0.1))
    assert rrt.extend(blocked, t2, (0.9, 0.1)) == ("Trapped", None)


# ---- Problem 17: the planar arm and the round post ----

POST_RADIUS = 0.1


def arm_clearance(post):
    """eta for the straight two-link arm turning about the origin (theta2 = 0)."""

    def eta(q):
        direction = np.array([np.cos(q[0]), np.sin(q[0])])
        along = np.clip(np.dot(post, direction), 0.0, 2.0)
        return float(np.linalg.norm(post - along * direction) - POST_RADIUS)

    return eta


def test_planar_motion_bound_is_sqrt5():
    assert eec.motion_bound(np.array([2.0, 1.0])) == pytest.approx(np.sqrt(5))


def test_certificate_is_the_theorem():
    assert eec.is_certified((0.0,), 1.0, (1.0,), 1.0, rho=1.9)
    assert not eec.is_certified((0.0,), 1.0, (1.0,), 1.0, rho=2.1)
    assert not eec.is_certified((0.0,), -0.1, (0.1,), 5.0, rho=1.0)


@pytest.mark.parametrize(
    "post, free, evaluated",
    [
        ((1.2, 1.2), False, None),  # A: the sweep goes through the post
        ((1.6, 1.6), True, [0, 90, 45, 22.5, 67.5]),  # B: the 17.5 sequence
    ],
)
def test_bisection_on_the_problem_17_sweep(post, free, evaluated):
    eta = arm_clearance(np.array(post))
    seen = []

    def clearance(q):
        seen.append(round(np.degrees(q[0]), 6))
        return eta(q)

    result = eec.exact_edge_is_free(
        clearance, lambda q: False, (0.0, 0.0), (np.pi / 2, 0.0), rho=np.sqrt(5), delta=0.0
    )
    assert result is free
    if evaluated is not None:
        assert sorted(seen) == sorted(evaluated)


def test_fixed_step_checking_misses_the_post_at_a():
    eta = arm_clearance(np.array([1.2, 1.2]))
    assert all(eta((np.radians(d), 0.0)) > 0 for d in range(0, 91, 10))
    assert eta((np.pi / 4, 0.0)) < 0
