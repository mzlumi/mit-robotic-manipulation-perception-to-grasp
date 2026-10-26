"""PS4: normals from a depth patch, antipodal sampling on a box, the grasp
frame, the turn about the pinch point, and the analytic ellipse."""

import numpy as np
import pytest
import trimesh
from pydrake.all import RigidTransform, RotationMatrix, cos, sin

import analytic_antipodal_grasps as aag
import normal_estimation as ne
import sampling_antipodal_grasps as sag


def test_normal_of_a_tilted_plane_points_at_the_camera():
    # A plane through (0, 0, 1) tilted about x, sampled on a 5 x 5 grid.
    u, v = np.meshgrid(np.linspace(-0.02, 0.02, 5), np.linspace(-0.02, 0.02, 5))
    tilt = 0.3
    points = np.dstack([u, v * np.cos(tilt), 1 + v * np.sin(tilt)])
    frames = ne.estimate_normal_by_nearest_pixels(RigidTransform(), points, np.ones((5, 5), bool))
    n = frames[0].rotation().matrix()[:, 2]
    expected = np.array([0, np.sin(tilt), -np.cos(tilt)])
    assert n == pytest.approx(expected, abs=1e-9)
    assert np.linalg.det(frames[0].rotation().matrix()) == pytest.approx(1)


def test_synthetic_checkpoint():
    x, y = np.meshgrid([-0.01, 0.0, 0.01], [-0.01, 0.0, 0.01])
    points = np.dstack([x, y, np.ones_like(x)])
    X = ne.estimate_normal_by_nearest_pixels(RigidTransform([0.1, -0.2, 0.3]), points, np.ones((3, 3), bool))[0]
    assert X.translation() == pytest.approx([0.1, -0.2, 1.3])
    assert X.rotation().matrix()[:, 2] == pytest.approx([0, 0, -1])


def test_degenerate_patches_are_skipped():
    line = np.dstack([np.linspace(0, 0.02, 3)[None].repeat(3, 0), np.zeros((3, 3)), np.ones((3, 3))])
    assert ne.estimate_normal_by_nearest_pixels(RigidTransform(), line, np.ones((3, 3), bool)) == []


def test_box_pairs_are_antipodal_and_opposite():
    box = trimesh.creation.box(extents=(0.03, 0.2, 0.05))
    pairs = sag.sample_colinear_points(box, 200, rng=np.random.default_rng(0))
    assert len(pairs) == 200
    for p1, p2, n1, n2 in pairs:
        assert n2 == pytest.approx(-n1, abs=1e-9)
        assert (p2 - p1) @ (-n1) > 0


def test_grasp_frame_from_points():
    p1, p2 = np.array([-0.01, 0.0, 0.05]), np.array([0.01, 0.0, 0.05])
    X = sag.compute_grasp_from_points((p1, p2, np.array([-1.0, 0, 0]), np.array([1.0, 0, 0])))
    R = X.rotation().matrix()
    assert R[:, 0] == pytest.approx([-1, 0, 0])  # from p2 toward p1
    assert R[:, 1] == pytest.approx([0, 0, -1])  # as close to down as possible
    assert X @ np.array([0, sag.FINGER_LENGTH, 0]) == pytest.approx((p1 + p2) / 2)
    assert sag.grasp_moment_arm(X, np.array([0.03, 0.04, 0.05])) == pytest.approx(0.05)
    vertical = (np.zeros(3), np.array([0, 0, 0.02]), np.array([0, 0, -1.0]), np.array([0, 0, 1.0]))
    assert sag.compute_grasp_from_points(vertical) is None


def test_filter_rejects_chords_outside_the_friction_cone():
    p1, p2 = np.array([0.0, 0.0, 0.2]), np.array([0.02, 0.0, 0.2])
    inward = np.array([1.0, 0, 0])
    good = (p1, p2, -inward, inward)
    tilted = RotationMatrix.MakeYRotation(np.radians(60)).matrix() @ inward
    bad = (p1, p2, -tilted, inward)
    kept = sag.get_filtered_grasps([good, bad], 0.8, 0.04, 0.005, RigidTransform(), mu=np.tan(np.radians(30)))
    assert len(kept) == 1


def test_turn_is_about_the_pinch_point():
    candidate = (np.array([-0.01, 0.0, 0.05]), np.array([0.01, 0.0, 0.05]), np.array([-1.0, 0, 0]), np.array([1.0, 0, 0]))
    X = sag.compute_grasp_from_points(candidate)
    keyframes = sag.plan(X, X)
    assert len(keyframes) == 10
    pinch = np.array([0, sag.FINGER_LENGTH, 0])
    before, after = keyframes[4][0], keyframes[5][0]
    assert after @ pinch == pytest.approx(before @ pinch)
    R = after.rotation().matrix() @ before.rotation().matrix().T
    assert np.degrees(np.arctan2(R[1, 0], R[0, 0])) == pytest.approx(-30)


def test_ellipse_antipodal_pair():
    def ellipse(t):
        return np.array([2 * cos(t), sin(t)])

    t, eigenvalues = aag.find_antipodal_pts(ellipse, np.random.default_rng(1), orientation=1)
    # stationary pairs on an ellipse are the two axes
    d = (t[0] - t[1]) % (2 * np.pi)
    assert d == pytest.approx(np.pi, abs=1e-5)
    assert np.sort(eigenvalues).tolist() in (
        pytest.approx([-16, -12], abs=1e-4),
        pytest.approx([-4, 12], abs=1e-4),
    )
