"""Replay buffer for reinforcement learning algorithms."""

import numpy as np
from collections import deque


class ReplayBuffer:
    """Standard replay buffer for storing experiences."""

    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = deque(maxlen=capacity)

    def add(self, state, action, reward, next_state, done):
        """Add experience to the buffer."""
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        """Sample a batch of experiences."""
        indices = np.random.choice(len(self.buffer), batch_size, replace=False)
        states, actions, rewards, next_states, dones = zip(
            *[self.buffer[i] for i in indices]
        )

        return (
            np.array(states),
            np.array(actions),
            np.array(rewards),
            np.array(next_states),
            np.array(dones),
        )

    def __len__(self):
        return len(self.buffer)
