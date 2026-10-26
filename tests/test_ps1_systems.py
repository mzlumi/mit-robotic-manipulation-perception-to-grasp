"""PS1: the discrete-time systems, checked against hand calculations."""

import numpy as np
import pytest

from dtsystems import (
    Body1D,
    Constant,
    Observer,
    PController,
    PController2,
    SimpleTrajectoryFollower,
    feedback_composition,
    get_state,
    series_composition,
    simulate,
)


def test_body1d_forward_euler_step():
    body = Body1D(mass=2, dt=0.1)
    x, v = body.next_state_fun(np.array([0.0, 0.5]), np.array([0.3]))
    assert x == pytest.approx(0.05)
    assert v == pytest.approx(0.515)
    assert body.output_fun(np.array([0.7, 9.0]), None) == pytest.approx([0.7])


def test_constant_force_for_five_steps():
    diagram = series_composition(Constant([0.1]), Body1D(mass=2, dt=0.1))
    sim = simulate(diagram, [[0.0, 0.0]], T=0.5)
    x, v = get_state(diagram.GetSystems()[1], sim)
    # v_k = 0.005 k, x_5 = 0.1 * 0.005 * (0 + 1 + 2 + 3 + 4)
    assert v == pytest.approx(0.025)
    assert x == pytest.approx(0.005)


def test_controllers_and_observer():
    assert PController(1, [1.0], gain=10).output_fun(None, np.array([0.2])) == pytest.approx([8.0])
    assert PController2(2, gain=3).output_fun(
        None, [np.array([1.0, 2.0]), np.array([0.5, 0.0])]
    ) == pytest.approx([1.5, 6.0])
    assert Observer(4, [0, 1]).output_fun(None, np.array([1.0, 2.0, 3.0, 4.0])) == pytest.approx([1, 2])


def test_follower_advances_only_within_epsilon_and_stops_at_the_end():
    follower = SimpleTrajectoryFollower([np.zeros(2), np.ones(2)], epsilon=0.1)
    step = follower.next_state_fun
    assert step(np.array([0.0]), np.array([0.5, 0.0]))[0] == 0  # not there yet
    assert step(np.array([0.0]), np.array([0.05, 0.0]))[0] == 1  # arrived
    assert step(np.array([1.0]), np.ones(2))[0] == 1  # last waypoint holds
    assert follower.output_fun(np.array([1.0]), None) == pytest.approx([1, 1])


def test_feedback_of_p_controller_and_forward_euler_body_diverges():
    """The 7.4 answer: forward Euler on an undamped oscillator gains energy."""
    diagram = feedback_composition(PController(1, [1.0], gain=10), Body1D(mass=2, dt=0.1))
    sim = simulate(diagram, [[0.2, 0.5]], T=10)
    log = next(s for s in diagram.GetSystems() if s.get_name() == "log_Body1D")
    x = log.FindLog(sim.get_context()).data()[0]
    assert np.abs(x[-20:] - 1).max() > 5 * np.abs(x[:20] - 1).max()
