"""PS2: planar kinematics, the transform exercise and the grasp geometry."""

import numpy as np
import pytest
from pydrake.all import RigidTransform, RotationMatrix

import forward_kin as fk
import pick_and_place as pp


def numeric_jacobian(q, eps=1e-6):
    return np.column_stack(
        [(fk.forward_kinematics(q + eps * e) - fk.forward_kinematics(q - eps * e)) / (2 * eps) for e in np.eye(2)]
    )


def test_forward_kinematics_values():
    assert fk.forward_kinematics(np.zeros(2)) == pytest.approx([2, 0])
    assert fk.forward_kinematics(np.array([np.pi / 2, -np.pi / 2])) == pytest.approx([1, 1])
    assert fk.forward_kinematics(np.array([0.3, 0.2])) == pytest.approx([1.83291905, 0.77494575])


@pytest.mark.parametrize("seed", range(20))
def test_inverse_kinematics_round_trip_and_jacobian(seed):
    q = np.random.default_rng(seed).uniform(-np.pi, np.pi, 2)
    p = fk.forward_kinematics(q)
    for solution in fk.inverse_kinematics(*p):
        assert fk.forward_kinematics(solution) == pytest.approx(p, abs=1e-9)
    assert fk.Jacobian(q) == pytest.approx(numeric_jacobian(q), abs=1e-6)


def test_jacobian_singular_exactly_when_elbow_is_straight_or_folded():
    for q1 in (0.0, np.pi):
        assert abs(np.linalg.det(fk.Jacobian(np.array([0.4, q1])))) < 1e-12
    assert abs(np.linalg.det(fk.Jacobian(np.array([0.0, 0.3])))) == pytest.approx(np.sin(0.3))


def test_problem_4_transforms():
    X_AB = fk.X_WA.inverse() @ fk.X_WB
    assert X_AB.translation()[:2] == pytest.approx([1.366, 0.366], abs=1e-3)
    assert X_AB.rotation().ToRollPitchYaw().yaw_angle() == pytest.approx(-5 * np.pi / 12)
    assert (fk.X_WC @ np.array([1.0, 1.0, 0.0]))[:2] == pytest.approx([6.366, 2.634], abs=1e-3)


def test_overhead_grasp_points_down_and_closes_across_the_short_side():
    X_OG = pp.make_overhead_box_grasp()
    R = X_OG.rotation().matrix()
    assert R[:, 1] == pytest.approx([0, 0, -1])  # fingers reach down
    assert R[:, 0] == pytest.approx([0, 1, 0])  # close along the 50 mm axis
    assert np.linalg.det(R) == pytest.approx(1)
    pads = X_OG @ np.array([0, pp.GRIPPER_REACH, 0])
    assert pads == pytest.approx([0, 0, pp.BLOCK_SIZE[2] / 2])


def test_timed_trajectories_integrate_back_to_the_waypoints():
    start = RigidTransform()
    goal = RigidTransform(RotationMatrix.MakeZRotation(0.8), [0.2, -0.1, 0.3])
    V_G, wsg = pp.timed_trajectories([pp.Waypoint(goal, 0.01)], start, speed=0.05)
    t = np.linspace(V_G.start_time(), V_G.end_time(), 4001)
    v = np.array([V_G.value(ti).ravel() for ti in t])
    displacement = np.trapezoid(v[:, 3:], t, axis=0)
    assert displacement == pytest.approx(goal.translation(), abs=1e-3)
    assert wsg.value(0.0).item() == pytest.approx(pp.FINGER_OPEN)
    assert wsg.value(V_G.end_time() - 1e-3).item() == pytest.approx(0.01)


def test_plan_shape():
    waypoints = pp.plan("red", "blue", "yellow")
    assert len(waypoints) == 20
    # every finger change is followed by a stationary leg
    for a, b, c in zip(waypoints, waypoints[1:], waypoints[2:]):
        if a.opening != b.opening:
            assert b.X_WG.IsExactlyEqualTo(c.X_WG) or b.X_WG.IsNearlyEqualTo(a.X_WG, 1e-12)
