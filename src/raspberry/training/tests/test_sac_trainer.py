"""Unit tests for SAC trainer and related functionality."""

import unittest
import numpy as np
import torch
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sac_trainer import SACTrainer, QNetwork, PolicyNetwork
from memory.replay_buffer import ReplayBuffer
from robot_interface import RobotInterface


class MockRobotInterface:
    """Mock robot interface for testing."""

    def __init__(self):
        self.connected = True
        self.parameters = {}
        self.state = [0, 0, 0, 0, 0, 0, 0]
        self.metrics = {
            "angle_mean": 5.0,
            "angle_max": 10.0,
            "speed_error": 1.0,
            "acceleration_mean": 0.5,
            "success_rate": 0.8,
            "overall_score": 30.0,
        }

    def get_state(self):
        return self.state

    def update_parameters(self, parameters):
        self.parameters.update(parameters)
        return True

    def run_test_pattern(self, duration=5.0, pattern_type="sine"):
        # Simulate improvement over time for certain parameters
        if (
            "balance_kp" in self.parameters
            and 4.0 <= self.parameters["balance_kp"] <= 7.0
        ):
            self.metrics["angle_mean"] *= 0.9
            self.metrics["overall_score"] *= 0.9

        return self.metrics

    def stop_and_reset(self):
        pass

    def disconnect(self):
        pass


class TestReplayBuffer(unittest.TestCase):
    """Test the replay buffer functionality."""

    def test_add_and_sample(self):
        buffer = ReplayBuffer(capacity=100)

        # Add 50 items
        for i in range(50):
            state = np.array([i, i + 1, i + 2, 0, 0, 0, 0])
            action = np.array([float(i) / 50.0])
            reward = float(i)
            next_state = np.array([i + 0.1, i + 1.1, i + 2.1, 0, 0, 0, 0])
            done = i % 10 == 0

            buffer.add(state, action, reward, next_state, done)

        # Sample batch
        batch_size = 10
        states, actions, rewards, next_states, dones = buffer.sample(batch_size)

        # Verify shapes
        self.assertEqual(states.shape[0], batch_size)
        self.assertEqual(actions.shape[0], batch_size)
        self.assertEqual(rewards.shape[0], batch_size)
        self.assertEqual(next_states.shape[0], batch_size)
        self.assertEqual(dones.shape[0], batch_size)

        # Verify types
        self.assertTrue(isinstance(states, np.ndarray))
        self.assertTrue(isinstance(actions, np.ndarray))
        self.assertTrue(isinstance(rewards, np.ndarray))
        self.assertTrue(isinstance(next_states, np.ndarray))
        self.assertTrue(isinstance(dones, np.ndarray))


class TestParameterConversion(unittest.TestCase):
    """Test parameter normalization and denormalization."""

    def test_normalize_action(self):
        param_ranges = {"balance_kp": (1.0, 10.0), "balance_ki": (0.0, 0.5)}

        trainer = SACTrainer(
            param_ranges=param_ranges, robot_interface=MockRobotInterface()
        )

        # Test normalization (-1 to 1 → min to max)
        action = np.array([-1.0, 0.0])
        params = trainer.normalize_action(action)

        self.assertEqual(params["balance_kp"], 1.0)  # -1 maps to min
        self.assertEqual(params["balance_ki"], 0.25)  # 0 maps to middle

        # Test extreme values
        action = np.array([1.0, 1.0])
        params = trainer.normalize_action(action)

        self.assertEqual(params["balance_kp"], 10.0)  # 1 maps to max
        self.assertEqual(params["balance_ki"], 0.5)  # 1 maps to max

    def test_denormalize_action(self):
        param_ranges = {"balance_kp": (1.0, 10.0), "balance_ki": (0.0, 0.5)}

        trainer = SACTrainer(
            param_ranges=param_ranges, robot_interface=MockRobotInterface()
        )

        # Test denormalization (min to max → -1 to 1)
        params = {"balance_kp": 5.5, "balance_ki": 0.25}
        action = trainer.denormalize_action(params)

        self.assertAlmostEqual(action[0].item(), 0.0, places=5)  # middle maps to 0
        self.assertAlmostEqual(action[1].item(), 0.0, places=5)  # middle maps to 0

        # Test extreme values
        params = {"balance_kp": 10.0, "balance_ki": 0.0}
        action = trainer.denormalize_action(params)

        self.assertAlmostEqual(action[0].item(), 1.0, places=5)  # max maps to 1
        self.assertAlmostEqual(action[1].item(), -1.0, places=5)  # min maps to -1


class TestSACNetworks(unittest.TestCase):
    """Test SAC neural networks."""

    def test_critic_network(self):
        state_dim = 7
        action_dim = 3
        batch_size = 10

        # Create critic network
        critic = QNetwork(state_dim, action_dim)

        # Create random state and action tensors
        states = torch.randn(batch_size, state_dim)
        actions = torch.randn(batch_size, action_dim)

        # Forward pass
        q1, q2 = critic(states, actions)

        # Check shapes
        self.assertEqual(q1.shape, (batch_size, 1))
        self.assertEqual(q2.shape, (batch_size, 1))

    def test_policy_network(self):
        state_dim = 7
        action_dim = 3
        batch_size = 10

        # Create policy network
        actor = PolicyNetwork(state_dim, action_dim)

        # Create random state tensor
        states = torch.randn(batch_size, state_dim)

        # Forward pass
        mean, log_std = actor(states)

        # Check shapes
        self.assertEqual(mean.shape, (batch_size, action_dim))
        self.assertEqual(log_std.shape, (batch_size, action_dim))

        # Test sampling
        action, log_prob, _ = actor.sample(states)

        # Check shapes
        self.assertEqual(action.shape, (batch_size, action_dim))
        self.assertEqual(log_prob.shape, (batch_size, 1))

        # Check action range (-1 to 1)
        self.assertTrue(torch.all(action >= -1.0))
        self.assertTrue(torch.all(action <= 1.0))


if __name__ == "__main__":
    unittest.main()
