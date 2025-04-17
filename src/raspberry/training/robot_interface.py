"""Interface for communicating with the physical self-balancing robot."""

import time
import numpy as np
import sys
import os
import json
from collections import deque

# Add necessary path to import existing modules
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

# Import existing robot communication modules
from raspberry.bluetooth.BLEReceiver import BLEReceiver
from raspberry.main import SelfBalancingRobot


class RobotInterface:
    """Handles communication with the physical robot using existing modules."""

    def __init__(self, connection_method="bluetooth", config_file=None):
        """Initialize communication with the physical robot.

        Args:
            connection_method: How to connect to the robot ("bluetooth" or "direct")
            config_file: Path to configuration file (uses default if None)
        """
        self.connection_method = connection_method
        self.connected = False
        self.last_state = None
        self.current_parameters = {}
        self.telemetry_buffer = deque(
            maxlen=1000
        )  # Limited buffer size to prevent memory issues

        # Load configuration
        if config_file:
            with open(config_file, "r") as f:
                self.config = json.load(f)
        else:
            # Use default configuration
            self.config = None

        # Define allowed parameter ranges
        self.allowed_params = {
            "balance_kp": (1.0, 10.0),
            "balance_ki": (0.0, 0.5),
            "balance_kd": (0.0, 2.0),
            "k_acceleration": (0.5, 5.0),
            "k_torque_per_pw": (0.1, 1.0),
            "drag": (0.5, 2.0),
            "max_safe_tilt": (3.0, 8.0),
            "acceleration_smoothing": (0.5, 0.99),
            "balance2offset": (-0.5, 0.5),
        }

        # Connect to the robot
        self._connect()

    def _connect(self):
        """Establish connection to the robot."""
        try:
            if self.connection_method == "bluetooth":
                # Initialize BLE communication
                self.ble = BLEReceiver()
                # Register telemetry callback
                self.ble.set_telemetry_callback(self._handle_telemetry)
                self.connected = True
                print("Successfully connected to robot via Bluetooth")
            elif self.connection_method == "direct":
                # Create a direct instance of SelfBalancingRobot
                self.robot = SelfBalancingRobot(self.config)
                self.connected = True
                print("Successfully initialized direct robot control")
            else:
                raise ValueError(
                    f"Unsupported connection method: {self.connection_method}"
                )
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            self.connected = False

    def _handle_telemetry(self, data):
        """Process telemetry data from the robot.

        This handles the format from your existing BLEReceiver telemetry.
        """
        # Add to buffer for processing
        self.telemetry_buffer.append(data)

        try:
            # Check for telemetry data format
            # According to your src/raspberry/main.py, format is:
            # "A:{angle:.2f},B:{balance_angle:.2f},S:{speed},T:{turn},R:{running},L:{left_power},R:{right_power}"

            if data.startswith("A:"):
                # Parse telemetry format
                parts = data.split(",")
                values = {}

                for part in parts:
                    if ":" in part:
                        key, value = part.split(":", 1)
                        values[key] = value

                if "A" in values and "B" in values and "S" in values:
                    angle = float(values["A"])
                    balance_angle = float(values["B"])
                    speed = float(values["S"])

                    # Build state dictionary
                    self.last_state = {
                        "angle": angle,
                        "target_angle": balance_angle,
                        "speed": speed,
                        "timestamp": time.time(),
                    }

                    # Add optional values if available
                    if "L" in values and "R" in values:
                        self.last_state["left_power"] = float(values["L"])
                        self.last_state["right_power"] = float(values["R"])
                    else:
                        self.last_state["left_power"] = 0.0
                        self.last_state["right_power"] = 0.0

                    # Add acceleration (this might come from robot or we'll estimate it)
                    # For now, using a simple estimate based on speed changes
                    if hasattr(self, "prev_speed") and hasattr(self, "prev_time"):
                        dt = time.time() - self.prev_time
                        if dt > 0:
                            self.last_state["acceleration"] = (
                                speed - self.prev_speed
                            ) / dt
                        else:
                            self.last_state["acceleration"] = 0.0
                    else:
                        self.last_state["acceleration"] = 0.0

                    # Store current values for next calculation
                    self.prev_speed = speed
                    self.prev_time = time.time()

            # Process config confirmations
            elif data.startswith(",C:") or data.startswith("C:"):
                # Handle configuration response
                config_data = data[3:] if data.startswith(",C:") else data[2:]
                print(f"Configuration response: {config_data}")

            # Process messages
            elif data.startswith(",M:") or data.startswith("M:"):
                # Handle message
                message = data[3:] if data.startswith(",M:") else data[2:]
                print(f"Robot message: {message}")

        except Exception as e:
            print(f"Error parsing telemetry data: {e}")

    def get_state(self):
        """Get the current state of the robot.

        Returns:
            List of normalized state values for RL algorithm
        """
        if not self.last_state:
            # Default state vector if no telemetry received yet
            return [0, 0, 0, 0, 0, 0, 0]

        # Convert dictionary to normalized list for RL algorithm
        # Normalize to reasonable ranges for neural network inputs
        return [
            self.last_state["angle"] / 30.0,  # Normalize angle
            self.last_state["speed"] / 5.0,  # Normalize speed
            self.last_state.get("acceleration", 0.0) / 10.0,  # Normalize acceleration
            self.last_state.get("left_power", 0.0) / 100.0,  # Normalize motor power
            self.last_state.get("right_power", 0.0) / 100.0,  # Normalize motor power
            self.last_state.get("angle", 0.0)
            - self.last_state.get("target_angle", 0.0) / 30.0,  # Normalized error
            self.last_state.get("target_angle", 0.0) / 30.0,  # Normalize target angle
        ]

    def update_parameters(self, parameters):
        """Update the robot's control parameters with validation.

        Args:
            parameters: Dictionary of parameter name/value pairs

        Returns:
            True if successful, False otherwise
        """
        if not self.connected:
            print("Cannot update parameters: Robot not connected")
            return False

        # Validate parameters before sending to robot
        validated_params = {}
        for param, value in parameters.items():
            try:
                # Check if parameter exists in allowed ranges
                if param not in self.allowed_params:
                    print(f"Warning: Unknown parameter '{param}', skipping")
                    continue

                # Get allowed range
                min_val, max_val = self.allowed_params[param]

                # Clamp to allowed range
                safe_value = max(min_val, min(max_val, value))

                # Warn if value was changed
                if safe_value != value:
                    print(
                        f"Warning: Parameter '{param}' value {value} outside allowed range [{min_val}, {max_val}], clamped to {safe_value}"
                    )

                validated_params[param] = safe_value

            except Exception as e:
                print(f"Error validating parameter '{param}': {e}")

        # Nothing to update after validation
        if not validated_params:
            return False

        # Send validated parameters to robot
        success = True
        for param, value in validated_params.items():
            try:
                if self.connection_method == "bluetooth":
                    # Send parameter update command via BLE using existing format
                    # Based on your BLE implementation, use the appropriate command format
                    command = f"CONFIG:{param}:{value:.4f}"
                    self.ble.send_command(command)
                elif self.connection_method == "direct":
                    # Use the direct _update_config method of SelfBalancingRobot
                    self.robot._update_config(param, value)

                # Store current value
                self.current_parameters[param] = value
                print(f"Updated {param} to {value:.4f}")
            except Exception as e:
                print(f"Failed to update {param}: {e}")
                success = False

        return success

    def run_test_pattern(self, duration=5.0, pattern_type="sine"):
        """Run a test pattern on the robot using existing drive commands.

        Args:
            duration: Duration of test in seconds
            pattern_type: Type of test pattern ("sine", "step", "random")

        Returns:
            Performance metrics dictionary
        """
        if not self.connected:
            print("Cannot run test: Robot not connected")
            return {"error": "Robot not connected"}

        # Clear existing telemetry buffer
        self.telemetry_buffer.clear()

        # Make sure robot is balanced before starting test
        if self.connection_method == "bluetooth":
            self.ble.send_command("START")
        elif self.connection_method == "direct":
            # Use existing method to start balancing
            self.robot.start_balancing()

        time.sleep(1.0)  # Give time to stabilize

        # Store metrics
        states = []
        commands = []
        start_time = time.time()

        try:
            # Run the selected test pattern
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time

                if pattern_type == "sine":
                    # Sinusoidal speed pattern
                    target_speed = 2.0 * np.sin(elapsed)
                    turn = 0.0
                elif pattern_type == "step":
                    # Step pattern
                    target_speed = 1.0 if int(elapsed) % 2 == 0 else -1.0
                    turn = 0.0
                elif pattern_type == "random":
                    # Random changes
                    if int(elapsed * 5) % 5 == 0:  # Change every 0.2 seconds
                        target_speed = np.random.uniform(-2.0, 2.0)
                        turn = np.random.uniform(-0.5, 0.5)
                else:
                    target_speed = 0.0
                    turn = 0.0

                # Send command to robot using existing command format
                if self.connection_method == "bluetooth":
                    command = f"DRIVE:{target_speed:.2f}:{turn:.2f}"
                    self.ble.send_command(command)
                elif self.connection_method == "direct":
                    # Use existing method to drive
                    self.robot.drive(target_speed, turn)

                commands.append((target_speed, turn))

                # Get current state
                if self.last_state:
                    states.append(self.last_state.copy())

                # Short delay for stability
                time.sleep(0.05)

            # Calculate performance metrics
            return self._calculate_metrics(states, commands)

        except Exception as e:
            print(f"Error during test pattern: {e}")
            return {"error": str(e)}
        finally:
            # Stop the robot
            if self.connection_method == "bluetooth":
                self.ble.send_command("STOP")
            elif self.connection_method == "direct":
                # Use existing method to stop
                self.robot.stop()

    def _calculate_metrics(self, states, commands):
        """Calculate performance metrics from collected data."""
        if not states:
            return {"error": "No data collected"}

        # Extract data
        angles = [s["angle"] for s in states]
        speeds = [s["speed"] for s in states]
        target_speeds = [c[0] for c in commands]
        accelerations = [s.get("acceleration", 0) for s in states]

        # Calculate metrics
        metrics = {
            "angle_mean": np.mean(np.abs(angles)),
            "angle_max": np.max(np.abs(angles)),
            "speed_error": np.sqrt(
                np.mean(
                    [
                        (target_speeds[i] - speeds[i]) ** 2
                        for i in range(min(len(speeds), len(target_speeds)))
                    ]
                )
            ),
            "acceleration_mean": np.mean(np.abs(accelerations)),
            "success_rate": sum(1 for a in angles if abs(a) < 15) / max(1, len(angles)),
        }

        # Overall score (lower is better)
        metrics["overall_score"] = (
            5 * metrics["angle_mean"]
            + 3 * metrics["speed_error"]
            + metrics["acceleration_mean"]
        )

        return metrics

    def stop_and_reset(self):
        """Emergency stop and reset the robot."""
        try:
            if self.connection_method == "bluetooth":
                self.ble.send_command("STOP")
                time.sleep(0.5)
                self.ble.send_command("RESET")
            elif self.connection_method == "direct":
                self.robot.stop()
                time.sleep(0.5)
                self.robot.reset()
            return True
        except Exception as e:
            print(f"Error during emergency stop: {e}")
            return False

    def disconnect(self):
        """Disconnect from the robot."""
        if self.connected:
            try:
                if self.connection_method == "bluetooth":
                    # Ensure robot is stopped before disconnecting
                    self.ble.send_command("STOP")
                    # Close BLE connection
                    self.ble.disconnect()
                elif self.connection_method == "direct":
                    # Stop the robot
                    self.robot.stop()

                self.connected = False
                print("Disconnected from robot")
            except Exception as e:
                print(f"Error disconnecting from robot: {e}")
