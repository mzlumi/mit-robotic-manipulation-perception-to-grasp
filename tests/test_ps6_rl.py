"""PS6: REINFORCE, BC, GAE and PPO losses, against the hand calculations."""

import numpy as np
import pytest
import torch

import push_math as pm
import reinforce as rf


def test_reward_to_go():
    assert rf.reward_to_go([1, 1, 1]) == pytest.approx([3, 2, 1])
    assert rf.reward_to_go([]).shape == (0,)
    rewards = np.array([1.0, 2.0])
    rf.reward_to_go(rewards)
    assert rewards == pytest.approx([1, 2])  # input untouched


def test_policy_loss_value_and_gradient():
    logp = torch.tensor([-0.2, -1.0, -0.4], requires_grad=True)
    loss = rf.policy_loss(logp, torch.tensor([2.0, 1, 1]), torch.tensor([1.0, 2, 0]), 2)
    assert float(loss.detach()) == pytest.approx(-0.2)
    loss.backward()
    assert logp.grad.tolist() == pytest.approx([-0.5, 0.5, -0.5])


def test_joint_log_prob_adds():
    cat = torch.distributions.Categorical(probs=torch.tensor([[0.2, 0.8]]))
    normal = torch.distributions.Normal(torch.tensor([0.0]), torch.tensor([1.0]))
    value = pm.joint_log_prob(cat, normal, torch.tensor([0]), torch.tensor([0.0]))
    assert float(value) == pytest.approx(np.log(0.2) - 0.5 * np.log(2 * np.pi))


def test_bc_loss_hand_value():
    probs = torch.tensor([[0.8, 0.2 / 3, 0.2 / 3, 0.2 / 3], [0.5, 0.5 / 3, 0.5 / 3, 0.5 / 3]])
    loss = pm.bc_loss(probs.log(), torch.tensor([0.01, -0.02]), torch.tensor([0, 0]), torch.zeros(2))
    assert float(loss) == pytest.approx(0.4831, abs=1e-4)


def test_gae_terminal_and_cutoff():
    assert pm.gae_advantages([0, 1], [0.2, 0.4], [0.4, 0.0]) == pytest.approx([0.77, 0.6])
    assert pm.gae_advantages([0, 1], [0.2, 0.4], [0.4, 0.5]) == pytest.approx([1.245, 1.1])
    # lambda = 1 with exact values gives Monte Carlo returns minus values
    assert pm.gae_advantages([1, 1, 1], [0, 0, 0], [0, 0, 0], lam=1.0) == pytest.approx([3, 2, 1])


def test_ppo_losses():
    ratios = torch.tensor([1.4, 0.7, 1.1, 0.9])
    loss = pm.ppo_actor_loss(ratios.log(), torch.zeros(4), torch.tensor([1.0, -1, 2, -2]), clip=0.2)
    assert float(loss) == pytest.approx(-0.2)
    assert float(pm.critic_loss(torch.tensor([1.0, 3]), torch.tensor([2.0, 1]))) == pytest.approx(2.5)
