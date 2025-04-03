"""
Parameter Training for Self-Balancing Robot using Reinforcement Learning

This module implements reinforcement learning algorithms to optimize the 
hyperparameters of the self-balancing robot's driving controller.
"""

import time
import numpy as np
import random
from collections import deque
import pickle
import os
import math
from datetime import datetime

# For more advanced implementations with neural networks
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

from drive import Driving
from controllers.motor_controller import MotorController
# Import the simulation
from robot_simulation import RobotSimulation


class TrainingEnvironment:
    """Environment manager that can switch between simulation and physical robot."""
    
    def __init__(self, use_physical=False, physical_params=None, sim_params=None):
        """Initialize the training environment.
        
        Args:
            use_physical: Whether to use the physical robot (True) or simulation (False)
            physical_params: Optional parameters for the physical robot
            sim_params: Optional parameters for the simulation
        """
        self.use_physical = use_physical
        self.physical_params = physical_params or {}
        self.sim_params = sim_params or {}
        self.robot = None
        
        # Safety limits for the physical robot
        self.safety_limits = {
            "max_tilt": 30.0,       # Maximum allowed tilt angle (degrees)
            "max_speed": 5.0,        # Maximum allowed speed (m/s)
            "emergency_stop_tilt": 35.0  # Tilt angle that triggers emergency stop
        }
        
        # Initialize the appropriate robot (simulation or physical)
        self.initialize_robot()
        
    def initialize_robot(self):
        """Initialize either the simulation or physical robot."""
        if self.use_physical:
            print("Initializing physical robot...")
            try:
                # Create motor controller with default config 
                motor_config = {
                    "IN1": 2,
                    "IN2": 3,
                    "IN3": 4,
                    "IN4": 5,
                    "ENA": 6,
                    "ENB": 7,
                    "PWM_FREQ": 1000,
                }
                motor_controller = MotorController(motor_config)
                self.robot = Driving(motor_controller)
                
                # Apply any custom parameters to the physical robot
                for param, value in self.physical_params.items():
                    if hasattr(self.robot, param):
                        setattr(self.robot, param, value)
                        
                print("Physical robot initialized successfully")
            except Exception as e:
                print(f"Error initializing physical robot: {e}")
                print("Falling back to simulation...")
                self.use_physical = False
                self.initialize_robot()  # Recursive call to initialize simulation
        else:
            print("Initializing robot simulation...")
            self.robot = RobotSimulation(self.sim_params)
            print("Simulation initialized successfully")
    
    def run_step(self, target_speed=0, turn_bias=0):
        """Run one step with the robot.
        
        Args:
            target_speed: Target speed for the robot
            turn_bias: Turn bias value (-100 to 100)
            
        Returns:
            Tuple of (angle, left_power, right_power, speed, acceleration)
        """
        # Check if robot is initialized
        if self.robot is None:
            print("Robot not initialized. Initializing now...")
            self.initialize_robot()
            
        # Run a step with safety checks
        try:
            # For simulation, just call forward directly
            if not self.use_physical:
                return self.robot.forward(target_speed=target_speed, turn_bias=turn_bias)
            
            # For physical robot, add safety checks
            current_angle = self.robot.angle_data[-1] if self.robot.angle_data else 0
            
            # Emergency safety check - stop if tilt is too extreme
            if abs(current_angle) > self.safety_limits["emergency_stop_tilt"]:
                print(f"EMERGENCY STOP: Tilt angle {current_angle:.2f}° exceeds safety limit")
                self.robot.stop()
                return current_angle, 0, 0, 0, 0
                
            # Apply speed limiting based on tilt
            safe_speed = target_speed
            if abs(current_angle) > self.safety_limits["max_tilt"] * 0.7:
                # Reduce speed proportionally as we approach the max tilt
                tilt_factor = (self.safety_limits["max_tilt"] - abs(current_angle)) / (self.safety_limits["max_tilt"] * 0.3)
                safe_speed = target_speed * max(0, min(1, tilt_factor))
                
            # Apply absolute speed limit
            safe_speed = max(-self.safety_limits["max_speed"], min(self.safety_limits["max_speed"], safe_speed))
            
            # Run the robot with the safe speed
            return self.robot.forward(target_speed=safe_speed, turn_bias=turn_bias)
            
        except Exception as e:
            print(f"Error during robot step: {e}")
            # Try to stop the robot safely
            try:
                if self.robot:
                    self.robot.stop()
            except:
                pass
            # Return zeros as fallback
            return 0, 0, 0, 0, 0
    
    def stop(self):
        """Stop the robot safely."""
        if self.robot:
            try:
                self.robot.stop()
                print("Robot stopped")
            except Exception as e:
                print(f"Error stopping robot: {e}")
    
    def switch_to_simulation(self):
        """Switch from physical robot to simulation."""
        if self.use_physical:
            print("Switching from physical robot to simulation...")
            self.stop()  # Stop the physical robot first
            self.use_physical = False
            self.initialize_robot()
    
    def switch_to_physical(self):
        """Switch from simulation to physical robot."""
        if not self.use_physical:
            print("Switching from simulation to physical robot...")
            self.use_physical = True
            self.initialize_robot()
            
    def apply_parameters(self, parameters):
        """Apply a set of parameters to the current robot."""
        if self.robot:
            for param, value in parameters.items():
                if hasattr(self.robot, param):
                    setattr(self.robot, param, value)
                    
    def get_state(self):
        """Get the current state vector from the robot."""
        if not self.robot:
            return np.zeros(7)
            
        try:
            # Get current angle
            current_angle = self.robot.angle_data[-1] if self.robot.angle_data else 0
            
            # Get current speed
            current_speed = self.robot.speed_data[-1] if self.robot.speed_data else 0
            
            # Get recent acceleration values
            recent_accels = list(self.robot.acceleration_history) if hasattr(self.robot, 'acceleration_history') else [0]
            avg_accel = sum(recent_accels) / len(recent_accels) if recent_accels else 0
            
            # Calculate rate of change of error (derivative component)
            error_derivative = self.robot.last_error - (self.robot.angle_data[-2] if len(self.robot.angle_data) > 1 else 0)
            
            # Get recent motor powers
            left_power = self.robot.left_power_data[-1] if self.robot.left_power_data else 0
            right_power = self.robot.right_power_data[-1] if self.robot.right_power_data else 0
            
            # State vector with normalized values
            state = np.array([
                current_angle / 90.0,              # Normalized angle
                current_speed / 10.0,              # Normalized speed
                avg_accel / 10.0,                  # Normalized acceleration
                left_power / 100.0,                # Normalized left motor power
                right_power / 100.0,               # Normalized right motor power
                self.robot.error_sum / 300.0,      # Normalized error sum
                error_derivative / 10.0            # Normalized error derivative
            ])
            
            return state
        except Exception as e:
            print(f"Error getting state: {str(e)}")
            # Return zero state as fallback
            return np.zeros(7)
    
    def is_terminal_state(self):
        """Check if the current state is terminal (robot has fallen, etc.)."""
        if not self.robot:
            return True
            
        try:
            # Check tilt angle
            current_angle = self.robot.angle_data[-1] if self.robot.angle_data else 0
            if abs(current_angle) > 45:  # Terminal tilt threshold
                return True
                
            # For simulation, also check if it's marked as fallen
            if not self.use_physical and hasattr(self.robot, 'fallen') and self.robot.fallen:
                return True
                
            # Check for oscillations (sign changes in recent angles)
            if hasattr(self.robot, 'angle_data') and len(self.robot.angle_data) > 10:
                recent_angles = list(self.robot.angle_data)[-10:]
                # Calculate sign changes in recent angles
                sign_changes = sum(1 for i in range(1, len(recent_angles)) 
                                if (recent_angles[i] * recent_angles[i-1] < 0))
                # If too many oscillations (sign changes) in recent history
                if sign_changes > 7:  # More than 7 sign changes in 10 steps
                    return True
                    
            return False
        except Exception as e:
            print(f"Error checking terminal state: {str(e)}")
            return True  # Assume terminal state on error for safety


class ParameterTrainer:
    """Reinforcement learning trainer for optimizing robot parameters."""
    
    def __init__(self, episodes=100, max_steps=1000, use_model=False, use_physical=False):
        """Initialize the parameter trainer.
        
        Args:
            episodes: Number of training episodes
            max_steps: Maximum steps per episode
            use_model: Whether to use a neural network model for action selection
            use_physical: Whether to use the physical robot for training
        """
        # Training settings
        self.episodes = episodes
        self.max_steps = max_steps
        self.exploration_rate = 1.0
        self.exploration_decay = 0.995
        self.exploration_min = 0.01
        self.learning_rate = 0.001
        self.discount_factor = 0.99
        self.batch_size = 64
        self.use_model = use_model and HAS_TORCH
        
        # Create the training environment
        self.env = TrainingEnvironment(use_physical=use_physical)
        
        # Experience replay buffer
        self.memory = deque(maxlen=10000)
        
        # Parameter ranges (min, max) for training
        self.parameter_ranges = {
            # PID parameters
            "balance_kp": (1.0, 10.0),
            "balance_ki": (0.0, 0.5),
            "balance_kd": (0.0, 2.0),
            
            # Balance model parameters
            "k_acc": (0.5, 5.0),
            "k_torque_per_pw": (0.1, 1.0),
            "drag": (0.5, 2.0),
            
            # Other control parameters
            "max_safe_tilt": (3.0, 8.0),
            "tilt_safety_threshold": (10.0, 20.0),
            "max_accel": (5.0, 15.0),
            "accel_smoothing": (0.1, 0.5)
        }
        
        # Parameter set for current episode
        self.current_parameters = self._get_default_parameters()
        
        # Best parameters found so far
        self.best_parameters = self._get_default_parameters()
        self.best_reward = float('-inf')
        
        # Statistics tracking
        self.episode_durations = []
        self.avg_rewards = []
        
        # Results logging
        self.results = {
            'episode_rewards': [],
            'parameter_history': [],
            'best_parameters': None,
            'best_reward': float('-inf'),
            'training_metadata': {
                'start_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'episodes': episodes,
                'max_steps': max_steps,
                'use_model': self.use_model,
                'use_physical': use_physical
            }
        }
        
        # Initialize the neural network model if enabled
        if self.use_model:
            self.state_dim = 7  # Size of state vector defined in _get_state
            self.action_dim = len(self.parameter_ranges)  # Number of adjustable parameters
            self.q_network = self._create_q_network()
            self.target_network = self._create_q_network()
            self.target_network.load_state_dict(self.q_network.state_dict())
            self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.learning_rate)
            self.target_update_frequency = 10  # Update target network every n episodes
    
    def _create_q_network(self):
        """Create a Q-network for deep reinforcement learning."""
        if not HAS_TORCH:
            print("PyTorch not available. Using random exploration instead.")
            return None
            
        # Simple feedforward neural network
        model = nn.Sequential(
            nn.Linear(self.state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, self.action_dim)
        )
        return model
    
    def _get_default_parameters(self):
        """Get default parameters based on the middle of the ranges."""
        return {param: (min_val + max_val) / 2 
                for param, (min_val, max_val) in self.parameter_ranges.items()}
    
    def _get_state(self):
        """Get the current state of the robot for RL."""
        return self.env.get_state()
    
    def _get_reward(self, state, prev_state=None):
        """Calculate the reward based on the current state.
        
        Higher rewards are given for:
        - Maintaining balance (angle close to zero)
        - Achieving desired speed
        - Smooth acceleration
        - Efficient power usage
        - Smooth control (avoiding oscillations)
        
        Args:
            state: Current state vector
            prev_state: Previous state vector (optional, for calculating changes)
        
        Returns:
            (reward, done): Reward value and whether episode is complete
        """
        try:
            # Unpack state values
            angle, speed, acceleration, left_power, right_power, error_sum, error_derivative = state
            
            # Angle component (exponential penalty for large angles)
            # Using exponential decay to heavily penalize large angles
            angle_reward = 100 * math.exp(-5 * abs(angle)) - 50
            
            # Speed matching component
            # Reward for closely matching target speed, with diminishing returns
            # Normalized target between -1 and 1
            normalized_target = self.env.robot.target_speed / 10.0
            speed_diff = abs(normalized_target - speed)
            speed_reward = 30 * math.exp(-3 * speed_diff) - 15
            
            # Acceleration smoothness component
            # Penalize jerky movements
            accel_reward = -30 * abs(acceleration)
            
            # Power efficiency component
            # Penalize high power usage
            power_usage = (abs(left_power) + abs(right_power)) / 2
            power_reward = -10 * power_usage
            
            # Oscillation penalty - if previous state available
            oscillation_reward = 0
            if prev_state is not None:
                # Calculate rate of change of angle
                prev_angle = prev_state[0]
                angle_change = angle - prev_angle
                
                # Penalize sign changes (oscillations)
                if angle * prev_angle < 0 and abs(angle) > 0.05:  # Only penalize significant oscillations
                    oscillation_reward = -20
            
            # Cumulative reward
            reward = angle_reward + speed_reward + accel_reward + power_reward + oscillation_reward
            
            # Check if state is terminal
            done = self.env.is_terminal_state()
            
            # Add large penalty for terminal states (robot fallen or oscillating too much)
            if done:
                reward -= 500
                
            return reward, done
        except Exception as e:
            print(f"Error calculating reward: {str(e)}")
            # Return neutral reward as fallback
            return 0, True  # Also return done=True for safety
    
    def _select_action_random(self):
        """Select parameter adjustments using a simple random exploration strategy."""
        # Either explore (random) or exploit (small adjustments around best known)
        if random.random() < self.exploration_rate:
            # Explore: random parameter adjustments
            adjustments = {}
            for param, (min_val, max_val) in self.parameter_ranges.items():
                # Random adjustment between -10% and +10% of range
                range_size = max_val - min_val
                adjustment = random.uniform(-0.1, 0.1) * range_size
                adjustments[param] = adjustment
        else:
            # Exploit: use small random adjustments around best known
            adjustments = {}
            for param in self.parameter_ranges.keys():
                # Smaller adjustments for exploitation, weighted toward better performance
                range_size = self.parameter_ranges[param][1] - self.parameter_ranges[param][0]
                # Use Gaussian distribution for more natural exploration around best value
                adjustment = random.gauss(0, 0.01) * range_size
                adjustments[param] = adjustment
                
        return adjustments
    
    def _select_action_model(self, state):
        """Select parameter adjustments using the trained neural network."""
        if not self.use_model or not HAS_TORCH:
            return self._select_action_random()
            
        # Convert state to tensor for the neural network
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        
        # Either explore or exploit
        if random.random() < self.exploration_rate:
            # Random exploration
            return self._select_action_random()
        else:
            # Use neural network to predict best parameter adjustments
            with torch.no_grad():
                self.q_network.eval()
                q_values = self.q_network(state_tensor).squeeze()
                
                # Convert Q-values to parameter adjustments
                adjustments = {}
                for i, param in enumerate(self.parameter_ranges.keys()):
                    # Scale the Q-value output to a reasonable adjustment size
                    # Q-values typically range from -10 to 10 after training
                    q_val = q_values[i].item()
                    range_size = self.parameter_ranges[param][1] - self.parameter_ranges[param][0]
                    # Scale and clamp adjustment to avoid extreme changes
                    adjustment = np.clip(q_val / 100.0, -0.1, 0.1) * range_size
                    adjustments[param] = adjustment
                
                return adjustments
    
    def _select_action(self, state=None):
        """Select action based on the current strategy (random or model-based)."""
        if self.use_model and HAS_TORCH and state is not None:
            return self._select_action_model(state)
        else:
            return self._select_action_random()
    
    def _apply_action(self, parameters, adjustments):
        """Apply parameter adjustments and ensure they stay within bounds."""
        new_parameters = parameters.copy()
        
        for param, adjustment in adjustments.items():
            new_value = parameters[param] + adjustment
            min_val, max_val = self.parameter_ranges[param]
            # Clip to parameter range
            new_parameters[param] = max(min_val, min(max_val, new_value))
            
        return new_parameters
    
    def _update_q_network(self):
        """Update the Q-network using a batch of experiences from replay memory."""
        if not self.use_model or not HAS_TORCH or len(self.memory) < self.batch_size:
            return
            
        # Sample random batch from memory
        batch = random.sample(self.memory, self.batch_size)
        
        # Unpack batch
        states, actions, rewards, next_states, dones = [], [], [], [], []
        for state, action, reward, next_state, done in batch:
            states.append(state)
            
            # Convert action (dict) to tensor format
            action_tensor = np.zeros(len(self.parameter_ranges))
            for i, param in enumerate(self.parameter_ranges.keys()):
                action_tensor[i] = action.get(param, 0)
            actions.append(action_tensor)
            
            rewards.append(reward)
            next_states.append(next_state)
            dones.append(done)
            
        # Convert to tensors
        state_batch = torch.FloatTensor(states)
        action_batch = torch.FloatTensor(actions)
        reward_batch = torch.FloatTensor(rewards)
        next_state_batch = torch.FloatTensor(next_states)
        done_batch = torch.FloatTensor(dones)
        
        # Calculate current Q values
        self.q_network.train()
        q_values = self.q_network(state_batch)
        
        # Calculate target Q values
        with torch.no_grad():
            self.target_network.eval()
            next_q_values = self.target_network(next_state_batch)
            # Calculate max Q value for next state
            max_next_q = next_q_values.max(1)[0]
            # Calculate target Q value using Bellman equation
            target_q = reward_batch + (1 - done_batch) * self.discount_factor * max_next_q
            
        # Calculate loss
        loss = nn.MSELoss()(q_values.sum(1), target_q)
        
        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
    
    def train_episode(self):
        """Train the robot for one episode."""
        # Initialize parameters (either default, best known, or random)
        if self.results['episode_rewards'] and random.random() < 0.7:
            # Use best parameters with small variations 70% of the time after first episode
            self.current_parameters = self.best_parameters.copy()
            for param in self.current_parameters:
                range_size = self.parameter_ranges[param][1] - self.parameter_ranges[param][0]
                # Use smaller variations for more focused search
                variation_scale = max(0.05 * (1.0 - len(self.results['episode_rewards']) / self.episodes), 0.01)
                self.current_parameters[param] += random.uniform(-variation_scale, variation_scale) * range_size
        else:
            # Use random parameters within ranges
            self.current_parameters = {
                param: random.uniform(min_val, max_val) 
                for param, (min_val, max_val) in self.parameter_ranges.items()
            }
        
        # Apply parameters to the robot
        self.env.apply_parameters(self.current_parameters)
        
        # Initialize episode variables
        episode_reward = 0
        done = False
        step = 0
        state = self._get_state()
        prev_state = None
        
        # Run the episode
        while not done and step < self.max_steps:
            # Store previous state for reward calculation
            prev_state = state
            
            # Select action (parameter adjustments)
            action = self._select_action(state)
            
            # Apply action to get new parameters
            new_parameters = self._apply_action(self.current_parameters, action)
            
            # Apply new parameters to robot
            self.env.apply_parameters(new_parameters)
            
            # Run one simulation step with the robot
            try:
                # Vary target speed to ensure robust parameter learning
                if step % 20 == 0:  # Change target speed every 20 steps
                    target_pattern = step // 20 % 3  # Cycle through 3 patterns
                    
                    if target_pattern == 0:
                        # Constant speed
                        target_speed = random.uniform(-5, 5)
                    elif target_pattern == 1:
                        # Sinusoidal speed (smooth changes)
                        target_speed = 3.0 * math.sin(step / 10.0)
                    else:
                        # Step function (abrupt changes)
                        target_speed = 2.0 if step % 40 < 20 else -2.0
                
                # Run the robot step
                self.env.run_step(target_speed=target_speed)
                
                # Get new state and reward
                state = self._get_state()
                reward, done = self._get_reward(state, prev_state)
                
                # Store experience in replay memory
                self.memory.append((prev_state, action, reward, state, done))
                
                # Accumulate episode reward
                episode_reward += reward
                
                # Update parameters for next step
                self.current_parameters = new_parameters
                
                # Update the Q-network using experience replay
                if self.use_model and step % 5 == 0:  # Update every 5 steps
                    self._update_q_network()
                
            except Exception as e:
                print(f"Error during step: {str(e)}")
                # Penalize errors in episode
                reward = -50
                episode_reward += reward
                done = True
            
            # Update step counter
            step += 1
            
            # Small delay to prevent running too fast (especially for simulation)
            # Different delay for physical vs simulation
            if self.env.use_physical:
                time.sleep(0.02)  # Longer delay for physical robot
            else:
                time.sleep(0.005)  # Shorter delay for simulation
        
        # Stop the robot at the end of the episode
        self.env.stop()
        
        # Update exploration rate
        self.exploration_rate = max(
            self.exploration_min, 
            self.exploration_rate * self.exploration_decay
        )
        
        # Track results
        self.results['episode_rewards'].append(episode_reward)
        self.results['parameter_history'].append(self.current_parameters.copy())
        self.episode_durations.append(step)
        
        # Update target network periodically
        if self.use_model and len(self.results['episode_rewards']) % self.target_update_frequency == 0:
            self.target_network.load_state_dict(self.q_network.state_dict())
        
        # Update best parameters if this episode was better
        if episode_reward > self.best_reward:
            self.best_reward = episode_reward
            self.best_parameters = self.current_parameters.copy()
            self.results['best_parameters'] = self.best_parameters
            self.results['best_reward'] = self.best_reward
            
        # Calculate moving average of rewards
        if len(self.results['episode_rewards']) >= 10:
            avg_reward = sum(self.results['episode_rewards'][-10:]) / 10
            self.avg_rewards.append(avg_reward)
            
        return episode_reward, step
    
    def train(self):
        """Run the full training routine."""
        print("Starting parameter training...")
        print(f"Using {'physical robot' if self.env.use_physical else 'simulation'}")
        print(f"Using neural network model: {self.use_model}")
        
        start_time = time.time()
        
        for episode in range(self.episodes):
            print(f"\nStarting episode {episode+1}/{self.episodes}...")
            episode_reward, steps = self.train_episode()
            
            # Print progress
            print(f"Episode {episode+1}/{self.episodes} - Steps: {steps} - Reward: {episode_reward:.2f}")
            
            # Print current best parameters every 10 episodes
            if (episode + 1) % 10 == 0:
                print(f"Exploration rate: {self.exploration_rate:.4f}")
                print("Current best parameters:")
                for param, value in self.best_parameters.items():
                    print(f"  {param}: {value:.4f}")
                
                # Print moving average if available
                if self.avg_rewards:
                    print(f"10-episode moving average reward: {self.avg_rewards[-1]:.2f}")
                    
                # Save intermediate results
                self.save_results(f"parameter_training_checkpoint_{episode+1}.pkl")
        
        # Record training duration
        training_duration = time.time() - start_time
        self.results['training_metadata']['duration_seconds'] = training_duration
        self.results['training_metadata']['end_time'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Save final results
        self.save_results()
        
        print("\nTraining complete!")
        print(f"Total training time: {training_duration:.1f} seconds")
        print(f"Best reward: {self.best_reward:.2f}")
        print("Best parameters:")
        for param, value in self.best_parameters.items():
            print(f"  {param}: {value:.4f}")
            
        return self.best_parameters
    
    def save_results(self, filename="parameter_training_results.pkl"):
        """Save training results to a file."""
        try:
            with open(filename, 'wb') as f:
                pickle.dump(self.results, f)
            print(f"Results saved to {filename}")
            
            # Also save best parameters in a more readable format
            readable_file = filename.replace('.pkl', '.txt')
            with open(readable_file, 'w') as f:
                f.write(f"Best reward: {self.best_reward:.2f}\n\n")
                f.write("Best parameters:\n")
                for param, value in self.best_parameters.items():
                    f.write(f"{param}: {value:.6f}\n")
                    
            # If using a neural network model, save the model weights
            if self.use_model and HAS_TORCH:
                model_file = filename.replace('.pkl', '_model.pt')
                torch.save({
                    'q_network': self.q_network.state_dict(),
                    'target_network': self.target_network.state_dict(),
                    'optimizer': self.optimizer.state_dict()
                }, model_file)
                print(f"Model saved to {model_file}")
                
        except Exception as e:
            print(f"Error saving results: {str(e)}")
    
    def load_results(self, filename="parameter_training_results.pkl"):
        """Load training results from a file."""
        try:
            if os.path.exists(filename):
                with open(filename, 'rb') as f:
                    self.results = pickle.load(f)
                self.best_parameters = self.results.get('best_parameters', self._get_default_parameters())
                self.best_reward = self.results.get('best_reward', float('-inf'))
                
                # Load model if available and using model
                if self.use_model and HAS_TORCH:
                    model_file = filename.replace('.pkl', '_model.pt')
                    if os.path.exists(model_file):
                        checkpoint = torch.load(model_file)
                        self.q_network.load_state_dict(checkpoint['q_network'])
                        self.target_network.load_state_dict(checkpoint['target_network'])
                        self.optimizer.load_state_dict(checkpoint['optimizer'])
                        print(f"Model loaded from {model_file}")
                
                print(f"Results loaded from {filename}")
                return True
            return False
        except Exception as e:
            print(f"Error loading results: {str(e)}")
            return False
    
    def apply_best_parameters(self, to_physical=None):
        """Apply the best parameters to the robot.
        
        Args:
            to_physical: If True, apply to physical robot. If False, apply to simulation.
                         If None, use current environment setting.
        """
        # Switch environment if requested
        if to_physical is not None and to_physical != self.env.use_physical:
            if to_physical:
                self.env.switch_to_physical()
            else:
                self.env.switch_to_simulation()
                
        # Apply the best parameters
        self.env.apply_parameters(self.best_parameters)
        return self.env.robot


def test_robot_with_parameters(parameters, duration=10, test_patterns=True, use_physical=False):
    """Test the robot with given parameters.
    
    Args:
        parameters: Dictionary of parameter values to use
        duration: Duration of test in seconds
        test_patterns: Whether to test different motion patterns
        use_physical: Whether to use physical robot (True) or simulation (False)
    """
    # Create environment with the specified robot type
    env = TrainingEnvironment(use_physical=use_physical)
    
    # Apply parameters
    env.apply_parameters(parameters)
    
    print(f"Testing {'physical robot' if use_physical else 'simulation'} with parameters for {duration} seconds...")
    print("Parameters:")
    for param, value in parameters.items():
        print(f"  {param}: {value:.4f}")
    
    # Store metrics for evaluation
    angles = []
    speeds = []
    targets = []
    accelerations = []
    
    start_time = time.time()
    try:
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            
            # Different test patterns if enabled
            if test_patterns:
                phase = int(elapsed / 5) % 3  # Change pattern every 5 seconds
                
                if phase == 0:
                    # Sine wave pattern (smooth changes)
                    target = 3.0 * np.sin((elapsed % 5) / 2)
                elif phase == 1:
                    # Step function (sudden changes)
                    target = 3.0 if (elapsed % 5) < 2.5 else -3.0
                else:
                    # Increasing speed
                    target = min(5.0, elapsed % 5)
            else:
                # Simple sine wave if test patterns disabled
                target = 3.0 * np.sin(elapsed / 2)
            
            # Run simulation step
            angle, left, right, speed, accel = env.run_step(target_speed=target)
            
            # Store metrics
            angles.append(angle)
            speeds.append(speed)
            targets.append(target)
            accelerations.append(accel)
            
            # Print current status
            print(f"Target: {target:5.2f} | Angle: {angle:6.2f}° | Speed: {speed:5.2f} m/s | Accel: {accel:5.2f} m/s²")
            
            # Sleep for appropriate amount of time
            if use_physical:
                time.sleep(0.05)  # 20Hz update for physical robot
            else:
                time.sleep(0.02)  # 50Hz update for simulation
            
    except KeyboardInterrupt:
        print("\nTest interrupted.")
    finally:
        env.stop()
    
    # Calculate performance metrics
    if angles and speeds and targets:
        # Average absolute tilt angle (lower is better)
        avg_tilt = sum(abs(a) for a in angles) / len(angles)
        
        # RMSE of speed error (lower is better)
        speed_error = math.sqrt(sum((targets[i] - speeds[i])**2 for i in range(len(speeds))) / len(speeds))
        
        # Average absolute acceleration (lower is better for smoothness)
        avg_accel = sum(abs(a) for a in accelerations) / len(accelerations)
        
        print("\nPerformance Metrics:")
        print(f"Average tilt angle: {avg_tilt:.2f}° (lower is better)")
        print(f"Speed tracking error: {speed_error:.2f} m/s (lower is better)")
        print(f"Average acceleration: {avg_accel:.2f} m/s² (lower for smoother control)")


def transfer_learning(sim_parameters, physical_episodes=10, max_steps=200):
    """Refine parameters from simulation on the physical robot.
    
    Args:
        sim_parameters: Parameters trained in simulation
        physical_episodes: Number of episodes to train on physical robot
        max_steps: Maximum steps per episode for physical robot
        
    Returns:
        Refined parameters for the physical robot
    """
    print("\n=== Transfer Learning: Simulation to Physical Robot ===")
    print("Starting with parameters trained in simulation:")
    for param, value in sim_parameters.items():
        print(f"  {param}: {value:.4f}")
    
    # Create a trainer with fewer episodes and smaller parameter ranges around simulation values
    trainer = ParameterTrainer(episodes=physical_episodes, max_steps=max_steps, use_physical=True)
    
    # Define tighter parameter ranges around simulation values
    for param, value in sim_parameters.items():
        if param in trainer.parameter_ranges:
            # Create a range that's ±20% around the simulation value, within original bounds
            orig_min, orig_max = trainer.parameter_ranges[param]
            range_size = orig_max - orig_min
            new_min = max(orig_min, value - 0.2 * range_size)
            new_max = min(orig_max, value + 0.2 * range_size)
            trainer.parameter_ranges[param] = (new_min, new_max)
    
    # Set best parameters to simulation parameters initially
    trainer.best_parameters = sim_parameters.copy()
    
    # Run the training on physical robot
    refined_parameters = trainer.train()
    
    print("\nRefined parameters for physical robot:")
    for param, value in refined_parameters.items():
        print(f"  {param}: {value:.4f}")
        
    return refined_parameters


def main():
    """Main function to run the parameter training."""
    print("\n---- Self-Balancing Robot Parameter Training ----\n")
    
    # Check for PyTorch availability
    if HAS_TORCH:
        print("Neural network support available (PyTorch detected)")
        use_model = input("Use neural network for training? (y/n): ").lower().startswith('y')
    else:
        print("Neural network support not available (PyTorch not detected)")
        use_model = False
    
    # Ask whether to use physical robot or simulation
    use_physical = input("Use physical robot for training? (y/n, default n): ").lower().startswith('y')
    
    # Get training parameters
    try:
        episodes = int(input("Number of training episodes (default 100): ") or "100")
        max_steps = int(input("Maximum steps per episode (default 500): ") or "500")
    except ValueError:
        print("Invalid input. Using defaults.")
        episodes = 100
        max_steps = 500
    
    # Create trainer
    trainer = ParameterTrainer(episodes=episodes, max_steps=max_steps, 
                              use_model=use_model, use_physical=use_physical)
    
    # Check if previous results exist
    if trainer.load_results():
        print("\nLoaded previous training results.")
        print("Best parameters:")
        for param, value in trainer.best_parameters.items():
            print(f"  {param}: {value:.4f}")
            
        # Ask if user wants to continue training
        choice = input("\nDo you want to (c)ontinue training, (t)est parameters, (p)hysical transfer learning, or (q)uit? ").lower()
        if choice.startswith('c'):
            # Continue training
            trainer.train()
        elif choice.startswith('t'):
            # Test with best parameters
            duration = int(input("Test duration in seconds (default 15): ") or "15")
            test_patterns = input("Test different motion patterns? (y/n, default y): ").lower() != 'n'
            use_phys = input("Test on physical robot? (y/n, default n): ").lower().startswith('y')
            test_robot_with_parameters(trainer.best_parameters, duration, test_patterns, use_phys)
        elif choice.startswith('p') and not use_physical:
            # Transfer learning from simulation to physical robot
            phys_episodes = int(input("Number of physical robot episodes (default 10): ") or "10")
            phys_max_steps = int(input("Maximum steps per episode (default 200): ") or "200")
            refined_params = transfer_learning(trainer.best_parameters, phys_episodes, phys_max_steps)
            # Test the refined parameters
            test_choice = input("Test the refined parameters? (y/n, default y): ").lower()
            if test_choice != 'n':
                test_robot_with_parameters(refined_params, 15, True, True)
        else:
            print("Exiting.")
            return
    else:
        # Start new training
        print("\nStarting new training session...")
        best_parameters = trainer.train()
        
        # Ask if user wants to test the parameters
        test_choice = input("Test the trained parameters? (y/n, default y): ").lower()
        if test_choice != 'n':
            test_robot_with_parameters(best_parameters, 15, True, use_physical)
            
        # Ask about transfer learning if trained in simulation
        if not use_physical:
            transfer_choice = input("Transfer learning to physical robot? (y/n, default n): ").lower()
            if transfer_choice.startswith('y'):
                phys_episodes = int(input("Number of physical robot episodes (default 10): ") or "10")
                phys_max_steps = int(input("Maximum steps per episode (default 200): ") or "200")
                refined_params = transfer_learning(best_parameters, phys_episodes, phys_max_steps)
                # Test the refined parameters
                test_choice = input("Test the refined parameters? (y/n, default y): ").lower()
                if test_choice != 'n':
                    test_robot_with_parameters(refined_params, 15, True, True)


if __name__ == "__main__":
    main()