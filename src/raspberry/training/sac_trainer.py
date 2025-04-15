"""SAC (Soft Actor-Critic) algorithm for robot parameter optimization."""

import os
import time
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal
from datetime import datetime
import pickle

from raspberry.training.memory.replay_buffer import ReplayBuffer
from raspberry.training.robot_interface import RobotInterface
from raspberry.training.visualization import TrainingVisualizer


# Neural network modules for SAC
class QNetwork(nn.Module):
    """Critic network for SAC, estimates Q-value."""

    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(QNetwork, self).__init__()

        # Q1 architecture
        self.linear1_q1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.linear2_q1 = nn.Linear(hidden_dim, hidden_dim)
        self.linear3_q1 = nn.Linear(hidden_dim, 1)

        # Q2 architecture
        self.linear1_q2 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.linear2_q2 = nn.Linear(hidden_dim, hidden_dim)
        self.linear3_q2 = nn.Linear(hidden_dim, 1)

    def forward(self, state, action):
        """Forward pass for both Q-networks."""
        x = torch.cat([state, action], 1)

        # Q1
        q1 = F.relu(self.linear1_q1(x))
        q1 = F.relu(self.linear2_q1(q1))
        q1 = self.linear3_q1(q1)

        # Q2
        q2 = F.relu(self.linear1_q2(x))
        q2 = F.relu(self.linear2_q2(q2))
        q2 = self.linear3_q2(q2)

        return q1, q2


class PolicyNetwork(nn.Module):
    """Actor network for SAC, outputs a distribution over actions."""

    def __init__(
        self, state_dim, action_dim, hidden_dim=256, log_std_min=-20, log_std_max=2
    ):
        super(PolicyNetwork, self).__init__()

        self.linear1 = nn.Linear(state_dim, hidden_dim)
        self.linear2 = nn.Linear(hidden_dim, hidden_dim)

        self.mean_linear = nn.Linear(hidden_dim, action_dim)
        self.log_std_linear = nn.Linear(hidden_dim, action_dim)

        self.log_std_min = log_std_min
        self.log_std_max = log_std_max

    def forward(self, state):
        """Forward pass to get action distribution parameters."""
        x = F.relu(self.linear1(state))
        x = F.relu(self.linear2(x))

        mean = self.mean_linear(x)
        log_std = self.log_std_linear(x)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)

        return mean, log_std

    def sample(self, state):
        """Sample an action from the policy."""
        mean, log_std = self.forward(state)
        std = log_std.exp()

        # Create normal distribution
        normal = Normal(mean, std)

        # Reparameterization trick
        x_t = normal.rsample()  # Sample with reparameterization
        action = torch.tanh(x_t)  # Apply tanh for bounded actions

        # Calculate log probability, accounting for the tanh transformation
        log_prob = normal.log_prob(x_t) - torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)

        # Return action, log probability, and original mean (for deterministic evaluation)
        return action, log_prob, torch.tanh(mean)


class SACTrainer:
    """Soft Actor-Critic trainer for robot parameter optimization."""

    def __init__(self, param_ranges, robot_interface=None, device="cpu", **kwargs):
        """Initialize the SAC trainer.

        Args:
            param_ranges: Dictionary mapping parameter names to (min, max) tuples
            robot_interface: RobotInterface instance or None to create a new one
            device: Device to run neural network computations on ('cpu' or 'cuda')
        """
        self.param_ranges = param_ranges
        self.state_dim = 7  # Default state dimension
        self.action_dim = len(param_ranges)  # One action dimension per parameter
        self.device = torch.device(device)

        # SAC hyperparameters
        self.gamma = kwargs.get("gamma", 0.99)
        self.tau = kwargs.get("tau", 0.005)
        self.alpha = kwargs.get("alpha", 0.2)
        self.lr = kwargs.get("lr", 0.0003)
        self.batch_size = kwargs.get("batch_size", 64)
        self.autotuned_entropy = kwargs.get("autotuned_entropy", True)

        # Initialize robot interface
        self.robot = robot_interface or RobotInterface()

        # Initialize networks
        self.critic = QNetwork(self.state_dim, self.action_dim).to(self.device)
        self.critic_target = QNetwork(self.state_dim, self.action_dim).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=self.lr)

        self.actor = PolicyNetwork(self.state_dim, self.action_dim).to(self.device)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=self.lr)

        # Automatic entropy tuning
        if self.autotuned_entropy:
            self.target_entropy = -torch.prod(torch.Tensor([self.action_dim])).item()
            self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
            self.alpha_optimizer = optim.Adam([self.log_alpha], lr=self.lr)

        # Initialize replay buffer
        self.replay_buffer = ReplayBuffer(int(1e6))

        # Training tracking
        self.total_steps = 0
        self.episode_rewards = []
        self.best_parameters = {}
        self.best_score = float("inf")
        self.param_history = []

        # Add visualizer
        self.visualizer = TrainingVisualizer(param_ranges)
        self.use_visualization = kwargs.get("use_visualization", True)

    def normalize_action(self, action):
        """Normalize action from [-1, 1] to actual parameter ranges."""
        normalized = {}
        for i, (param_name, (min_val, max_val)) in enumerate(self.param_ranges.items()):
            # Convert from [-1, 1] to [min_val, max_val]
            normalized[param_name] = min_val + (action[i] + 1) * 0.5 * (
                max_val - min_val
            )
        return normalized

    def denormalize_action(self, parameters):
        """Convert from actual parameter values to [-1, 1] range."""
        denormalized = torch.zeros(self.action_dim)
        for i, (param_name, (min_val, max_val)) in enumerate(self.param_ranges.items()):
            if param_name in parameters:
                # Convert from [min_val, max_val] to [-1, 1]
                denormalized[i] = (parameters[param_name] - min_val) / (
                    max_val - min_val
                ) * 2 - 1
        return denormalized

    def select_action(self, state, evaluate=False):
        """Select an action using the policy network."""
        state = torch.FloatTensor(state).to(self.device).unsqueeze(0)

        if evaluate:
            # Use mean action (no exploration) during evaluation
            with torch.no_grad():
                _, _, action = self.actor.sample(state)
                return action.cpu().numpy()[0]
        else:
            # Sample from distribution during training
            with torch.no_grad():
                action, _, _ = self.actor.sample(state)
                return action.cpu().numpy()[0]

    def update_parameters(self, batch_size=None):
        """Update the network parameters using a batch from replay buffer."""
        batch_size = batch_size or self.batch_size

        # Sample a batch
        states, actions, rewards, next_states, dones = self.replay_buffer.sample(
            batch_size
        )

        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device).unsqueeze(1)
        next_states = torch.FloatTensor(next_states).to(self.device)
        dones = torch.FloatTensor(dones).to(self.device).unsqueeze(1)

        # Update critic
        with torch.no_grad():
            next_actions, next_log_probs, _ = self.actor.sample(next_states)
            q1_next, q2_next = self.critic_target(next_states, next_actions)
            q_next = torch.min(q1_next, q2_next)

            if self.autotuned_entropy:
                alpha = self.log_alpha.exp().item()
            else:
                alpha = self.alpha

            # Target Q-values
            q_target = rewards + (1 - dones) * self.gamma * (
                q_next - alpha * next_log_probs
            )

        # Current Q-values
        q1, q2 = self.critic(states, actions)

        # Critic loss
        critic_loss = F.mse_loss(q1, q_target) + F.mse_loss(q2, q_target)

        # Update critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Update actor
        new_actions, log_probs, _ = self.actor.sample(states)
        q1_new, q2_new = self.critic(states, new_actions)
        q_new = torch.min(q1_new, q2_new)

        if self.autotuned_entropy:
            alpha = self.log_alpha.exp().item()
        else:
            alpha = self.alpha

        actor_loss = (alpha * log_probs - q_new).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Update alpha (if auto-tuned)
        if self.autotuned_entropy:
            alpha_loss = -(
                self.log_alpha * (log_probs + self.target_entropy).detach()
            ).mean()

            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()

        # Soft update of target network
        for target_param, param in zip(
            self.critic_target.parameters(), self.critic.parameters()
        ):
            target_param.data.copy_(
                target_param.data * (1.0 - self.tau) + param.data * self.tau
            )

    def train(self, num_episodes=100, max_steps=200, warm_up_steps=1000, update_freq=1):
        """Train the SAC agent with enhanced error recovery."""
        print("Starting SAC training for parameter optimization...")
        total_steps = 0

        # Start visualization if enabled
        self._safe_visualize("start_real_time_visualization")

        # Initial random exploration to fill buffer
        print("Collecting initial experience...")
        current_params = {}
        for _ in range(warm_up_steps):
            # Generate random parameters
            action = np.random.uniform(-1, 1, size=self.action_dim)
            parameters = self.normalize_action(action)

            # Apply to robot - using updated interface
            if self.robot.update_parameters(parameters):
                current_params = parameters

                # Run test pattern and observe state
                state = self.robot.get_state()
                metrics = self.robot.run_test_pattern(duration=2.0)

                if "error" not in metrics:
                    # Calculate reward (negative of score, since lower score is better)
                    reward = -metrics["overall_score"]
                    next_state = self.robot.get_state()
                    done = False

                    # Store in replay buffer
                    self.replay_buffer.add(state, action, reward, next_state, done)

        # Main training loop
        for episode in range(num_episodes):
            episode_reward = 0
            step = 0
            done = False
            state = self.robot.get_state()

            print(f"\nEpisode {episode+1}/{num_episodes}")

            # Mark new episode in visualizer
            if self.use_visualization:
                self.visualizer.add_data_point(
                    current_params, 0, float("inf"), new_episode=True
                )

            # Add recovery mechanism for failed episodes
            recovery_attempts = 0
            max_recovery_attempts = 3

            while not done and step < max_steps:
                # Select action
                action = self.select_action(state)
                parameters = self.normalize_action(action)

                # Apply to robot with retry logic
                update_success = False
                for attempt in range(3):  # Try up to 3 times
                    if self.robot.update_parameters(parameters):
                        update_success = True
                        current_params = parameters
                        break
                    print(f"Retry {attempt+1}/3 for parameter update...")
                    time.sleep(1)  # Wait before retry

                if not update_success:
                    print("Failed to update parameters after multiple attempts")
                    # If we repeatedly fail, try emergency reset
                    if recovery_attempts < max_recovery_attempts:
                        print(
                            f"Attempting recovery ({recovery_attempts+1}/{max_recovery_attempts})..."
                        )
                        recovery_attempts += 1
                        try:
                            # Send emergency stop command
                            self.robot.stop_and_reset()
                            time.sleep(2)  # Wait for reset
                            continue  # Try again with same action
                        except Exception as e:
                            print(f"Recovery failed: {e}")

                    # If recovery attempts exhausted, end episode
                    print("Ending episode due to persistent failure")
                    break

                # Run test pattern and observe result
                try:
                    metrics = self.robot.run_test_pattern(duration=3.0)
                except Exception as e:
                    print(f"Error during test pattern: {e}")
                    # Try emergency reset if test pattern fails
                    if recovery_attempts < max_recovery_attempts:
                        print(
                            f"Attempting recovery ({recovery_attempts+1}/{max_recovery_attempts})..."
                        )
                        recovery_attempts += 1
                        self.robot.stop_and_reset()
                        time.sleep(2)
                        continue
                    else:
                        break

                if "error" in metrics:
                    print(f"Error during test: {metrics['error']}")
                    reward = -1000  # Large penalty for errors
                    done = True
                else:
                    # Calculate reward (negative of score, since lower score is better)
                    reward = -metrics["overall_score"]
                    done = metrics["angle_max"] > 30  # Done if robot falls

                    # Track best parameters
                    score = metrics["overall_score"]
                    if score < self.best_score:
                        self.best_score = score
                        self.best_parameters = current_params.copy()
                        print(f"New best score: {score:.2f}")
                        print("Parameters:", end=" ")
                        for param, value in current_params.items():
                            print(f"{param}={value:.4f}", end=" ")
                        print()

                # Get next state
                next_state = self.robot.get_state()

                # Store transition in replay buffer
                self.replay_buffer.add(state, action, reward, next_state, done)

                # Move to next state
                state = next_state
                episode_reward += reward

                # Calculate reward and update visualizer
                if self.use_visualization:
                    self.visualizer.add_data_point(
                        current_params, reward, metrics["overall_score"]
                    )

                # Update networks
                if total_steps % update_freq == 0:
                    self.update_parameters()

                step += 1
                total_steps += 1

                # Print progress
                print(
                    f"  Step {step}: Reward={reward:.2f}, Score={metrics['overall_score']:.2f}"
                )
                print(
                    f"    Angle: mean={metrics['angle_mean']:.2f}°, max={metrics['angle_max']:.2f}°"
                )
                print(
                    f"    Speed error: {metrics['speed_error']:.2f}, Accel: {metrics['acceleration_mean']:.2f}"
                )

            # End of episode
            self.episode_rewards.append(episode_reward)
            self.param_history.append(current_params.copy())

            # Save visualization at end of episode
            if self.use_visualization:
                self.visualizer.save_plots(f"episode_{episode+1}")

            print(f"Episode {episode+1} finished: Total reward={episode_reward:.2f}")
            print(f"Best score so far: {self.best_score:.2f}")

            # Save checkpoint more frequently during early training
            if episode < 10 or (episode + 1) % 5 == 0:
                self.save_checkpoint(f"sac_checkpoint_{episode+1}.pkl")

        # Stop visualization
        if self.use_visualization:
            self.visualizer.stop_visualization()

        # Print final results
        print("\nTraining complete!")
        print(f"Best parameters:")
        for param, value in self.best_parameters.items():
            print(f"  {param}: {value:.4f}")

        return self.best_parameters

    def save_checkpoint(self, filename):
        """Save a checkpoint of the training progress."""
        checkpoint = {
            "actor_state_dict": self.actor.state_dict(),
            "critic_state_dict": self.critic.state_dict(),
            "critic_target_state_dict": self.critic_target.state_dict(),
            "critic_optimizer_state_dict": self.critic_optimizer.state_dict(),
            "actor_optimizer_state_dict": self.actor_optimizer.state_dict(),
            "param_ranges": self.param_ranges,
            "best_parameters": self.best_parameters,
            "best_score": self.best_score,
            "episode_rewards": self.episode_rewards,
            "param_history": self.param_history,
            "total_steps": self.total_steps,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }

        if self.autotuned_entropy:
            checkpoint["log_alpha"] = self.log_alpha
            checkpoint["alpha_optimizer_state_dict"] = self.alpha_optimizer.state_dict()

        os.makedirs("checkpoints", exist_ok=True)
        torch.save(checkpoint, os.path.join("checkpoints", filename))
        print(f"Checkpoint saved to checkpoints/{filename}")

    def load_checkpoint(self, filename):
        """Load a training checkpoint."""
        try:
            checkpoint = torch.load(os.path.join("checkpoints", filename))

            self.actor.load_state_dict(checkpoint["actor_state_dict"])
            self.critic.load_state_dict(checkpoint["critic_state_dict"])
            self.critic_target.load_state_dict(checkpoint["critic_target_state_dict"])
            self.critic_optimizer.load_state_dict(
                checkpoint["critic_optimizer_state_dict"]
            )
            self.actor_optimizer.load_state_dict(
                checkpoint["actor_optimizer_state_dict"]
            )

            if self.autotuned_entropy and "log_alpha" in checkpoint:
                self.log_alpha = checkpoint["log_alpha"]
                self.alpha_optimizer.load_state_dict(
                    checkpoint["alpha_optimizer_state_dict"]
                )

            self.best_parameters = checkpoint["best_parameters"]
            self.best_score = checkpoint["best_score"]
            self.episode_rewards = checkpoint["episode_rewards"]
            self.param_history = checkpoint["param_history"]
            self.total_steps = checkpoint["total_steps"]

            print(f"Loaded checkpoint from {filename}")
            print(f"Best score: {self.best_score:.2f}")
            return True
        except Exception as e:
            print(f"Error loading checkpoint: {e}")
            return False

    def _safe_visualize(self, func_name, *args, **kwargs):
        """Safely call visualization functions with error handling."""
        if not self.use_visualization:
            return

        try:
            # Get the function by name
            vis_func = getattr(self.visualizer, func_name)
            # Call it with provided arguments
            vis_func(*args, **kwargs)
        except Exception as e:
            print(f"Visualization error ({func_name}): {e}")
            # Disable visualization on error to prevent further issues
            if func_name != "stop_visualization":  # Still try to stop if needed
                self.use_visualization = False
