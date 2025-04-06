"""
Physics-based simulation of a two-wheeled self-balancing robot.

This module simulates the dynamics of a two-wheeled self-balancing robot
with configurable physical parameters. It's designed to be used for training
control parameters in simulation before transferring to a physical robot.
"""

import numpy as np
import math
import time
from collections import deque


class RobotSimulation:
    """Simulates the dynamics of a two-wheeled self-balancing robot."""

    def __init__(self, physical_params=None):
        """Initialize the robot simulation with physical parameters.

        Args:
            physical_params: Dictionary of physical parameters of the robot.
                If None, default parameters are used.
        """
        # Set default physical parameters (can be overridden)
        self.physical_params = {
            # Robot body parameters
            "body_mass": 0.5,  # kg - Mass of the robot body (without wheels)
            "body_height": 0.15,  # m - Height of the robot body (center of mass height)
            "body_width": 0.1,  # m - Width of the robot body
            "body_depth": 0.08,  # m - Depth of the robot body
            "body_inertia": 0.001,  # kg*m^2 - Moment of inertia of the body around pivot point
            # Wheel parameters
            "wheel_mass": 0.1,  # kg - Mass of each wheel
            "wheel_radius": 0.04,  # m - Radius of the wheels
            "wheel_width": 0.02,  # m - Width of each wheel
            "wheel_distance": 0.12,  # m - Distance between the wheels (center to center)
            "wheel_inertia": 0.00008,  # kg*m^2 - Moment of inertia of each wheel
            # Motor parameters
            "motor_torque_constant": 0.01,  # N*m/A - Torque constant of the motors
            "motor_resistance": 1.0,  # Ohm - Motor winding resistance
            "motor_voltage": 5.0,  # V - Motor supply voltage
            "motor_efficiency": 0.85,  # Motor efficiency (0-1)
            # Environmental parameters
            "gravity": 9.81,  # m/s^2 - Gravitational acceleration
            "rolling_resistance": 0.01,  # Rolling resistance coefficient
            "ground_friction": 0.8,  # Ground friction coefficient
            "air_resistance": 0.01,  # Air resistance coefficient
            # Noise parameters (for simulation realism)
            "sensor_noise_std": 0.02,  # Standard deviation of sensor noise
            "motor_noise_std": 0.05,  # Standard deviation of motor noise
            "disturbance_magnitude": 0.1,  # Maximum magnitude of random disturbances
            "disturbance_frequency": 0.1,  # Frequency of random disturbances (0-1)
        }

        # Override default parameters with provided ones
        if physical_params is not None:
            for key, value in physical_params.items():
                if key in self.physical_params:
                    self.physical_params[key] = value

        # Initialize state vector:
        # [x, x_dot, theta, theta_dot, left_wheel_angle, right_wheel_angle]
        # x: position, theta: body angle from vertical (0 is upright, + is leaning forward)
        self.state = np.zeros(6)
        self.state[2] = np.radians(
            1.0
        )  # Start with a small tilt to make balance necessary

        # Control parameters - will be updated during training
        self.balance_kp = 5.0
        self.balance_ki = 0.1
        self.balance_kd = 0.5
        self.k_acc = 2.0
        self.k_torque_per_pw = 0.5
        self.drag = 1.0
        self.max_safe_tilt = 5.0
        self.max_accel = 10.0
        self.accel_smoothing = 0.2

        # PID state
        self.error_sum = 0
        self.last_error = 0
        self.last_update_time = time.time()
        self.balance_target = 0  # Target angle (0 = upright)
        self.target_speed = 0  # Target linear speed

        # For sensor simulation
        self.init_sensor_data()

        # Timestamp tracking
        self.last_time = time.time()

        # Track if the robot has "fallen over" in simulation
        self.fallen = False

    def init_sensor_data(self):
        """Initialize data structures for simulated sensor readings."""
        # Data history (to mimic the actual robot's data structures)
        self.angle_data = deque(maxlen=100)
        self.angle_data_queue = deque(
            maxlen=100
        )  # Directly provide queue for parameter training
        self.left_power_data = deque(maxlen=100)
        self.right_power_data = deque(maxlen=100)

        # Speed and acceleration tracking
        self.speed_data = deque(maxlen=100)
        self.speed_data_queue = deque(
            maxlen=100
        )  # Directly provide queue for parameter training
        self.rpm_data = deque(maxlen=50)
        self.time_data = deque(maxlen=50)
        self.direction = 0  # 0: stopped, 1: forward, -1: backward

        # For reinforcement learning feedback
        self.acceleration_history = deque(maxlen=20)

    def set_balance_target(self, current_speed=0, target_speed=0):
        """Set the balance target angle based on current speed and target speed.

        Identical to the method in the actual robot for compatibility.
        """
        # Calculate balance target using the same formula as the actual robot
        self.balance_target = (
            1 + (self.k_torque_per_pw * current_speed) ** 2
        ) * self.k_acc * (target_speed - current_speed) + self.drag * target_speed

        # Constrain the balance target to safe limits
        self.balance_target = max(
            -self.max_safe_tilt, min(self.max_safe_tilt, self.balance_target)
        )

    def simulate(self, dt=0.01):
        """Advance the simulation by one time step (dt seconds).

        Args:
            dt: Time step in seconds

        Returns:
            Current state after the time step
        """
        # Get current state
        x, x_dot, theta, theta_dot, left_wheel, right_wheel = self.state

        # Extract physical parameters for easier access
        m_b = self.physical_params["body_mass"]
        h = self.physical_params["body_height"]
        I_b = self.physical_params["body_inertia"]
        m_w = self.physical_params["wheel_mass"]
        r = self.physical_params["wheel_radius"]
        I_w = self.physical_params["wheel_inertia"]
        d = self.physical_params["wheel_distance"]
        g = self.physical_params["gravity"]
        b_r = self.physical_params["rolling_resistance"]
        b_f = self.physical_params["ground_friction"]

        # Convert motor powers (-100 to 100) to torques
        left_power = self.left_power_data[-1] if self.left_power_data else 0
        right_power = self.right_power_data[-1] if self.right_power_data else 0

        # Apply random motor noise to make simulation more realistic
        motor_noise = self.physical_params["motor_noise_std"]
        left_power += (
            np.random.normal(0, motor_noise * abs(left_power)) if left_power != 0 else 0
        )
        right_power += (
            np.random.normal(0, motor_noise * abs(right_power))
            if right_power != 0
            else 0
        )

        # Convert power (-100 to 100) to motor torque
        motor_torque_constant = self.physical_params["motor_torque_constant"]
        motor_efficiency = self.physical_params["motor_efficiency"]
        max_torque = (
            motor_torque_constant
            * self.physical_params["motor_voltage"]
            / self.physical_params["motor_resistance"]
            * motor_efficiency
        )

        # Calculate motor torques (scale from -100,100 to max torque)
        left_torque = (left_power / 100.0) * max_torque
        right_torque = (right_power / 100.0) * max_torque

        # Add random disturbances occasionally to test controller robustness
        if np.random.random() < self.physical_params["disturbance_frequency"] * dt:
            # Random impulse disturbance
            impulse = np.random.uniform(
                -self.physical_params["disturbance_magnitude"],
                self.physical_params["disturbance_magnitude"],
            )
            theta_dot += impulse

        # Physics simulation - simplified but captures the main dynamics

        # Total mass
        M = m_b + 2 * m_w

        # Calculate forces and accelerations using inverted pendulum dynamics

        # Gravity force tries to topple the robot
        pendulum_torque = m_b * g * h * np.sin(theta)

        # Motor torques try to keep the robot balanced
        motor_torque = left_torque + right_torque

        # Calculate friction/resistance effects
        friction = (
            b_f * x_dot * abs(x_dot)
        )  # Friction proportional to speed^2, opposing motion
        rolling_resistance = b_r * x_dot  # Linear rolling resistance

        # Compute acceleration of the pendulum (body) - rotational
        theta_acc = (pendulum_torque - motor_torque / r) / I_b

        # Compute acceleration of the base (wheels) - translational
        x_acc = (
            motor_torque
            - m_b * h * theta_acc * np.cos(theta)
            - friction
            - rolling_resistance
        ) / M

        # Update wheel angles based on linear motion and differential steering
        wheel_speed = x_dot / r
        steering_diff = (right_torque - left_torque) / (d * I_w)
        left_wheel_speed = wheel_speed - steering_diff * d / 2
        right_wheel_speed = wheel_speed + steering_diff * d / 2

        # Euler integration for state update
        theta_new = theta + theta_dot * dt
        theta_dot_new = theta_dot + theta_acc * dt
        x_new = x + x_dot * dt
        x_dot_new = x_dot + x_acc * dt
        left_wheel_new = left_wheel + left_wheel_speed * dt
        right_wheel_new = right_wheel + right_wheel_speed * dt

        # Update state
        self.state = np.array(
            [
                x_new,
                x_dot_new,
                theta_new,
                theta_dot_new,
                left_wheel_new,
                right_wheel_new,
            ]
        )

        # Calculate sensed values (with added noise)
        sensor_noise = self.physical_params["sensor_noise_std"]

        # Angle in degrees with noise
        angle_degrees = np.degrees(theta_new) + np.random.normal(0, sensor_noise)
        self.angle_data.append(angle_degrees)
        self.angle_data_queue.append(angle_degrees)

        # Calculate current speed based on wheel rotation (as in the real robot)
        wheel_rpm = (
            (left_wheel_speed + right_wheel_speed) * 30 / np.pi
        )  # Convert rad/s to RPM
        wheel_rpm += (
            np.random.normal(0, sensor_noise * abs(wheel_rpm)) if wheel_rpm != 0 else 0
        )
        self.rpm_data.append(wheel_rpm)

        # Store current time for RPM calculations
        current_time = time.time() * 1000  # ms
        self.time_data.append(current_time)

        # Update speed
        if x_dot_new > 0:
            self.direction = 1
        elif x_dot_new < 0:
            self.direction = -1
        else:
            self.direction = 0

        # Calculate speed in m/s
        speed = (
            x_dot_new + np.random.normal(0, sensor_noise * abs(x_dot_new))
            if x_dot_new != 0
            else 0
        )
        self.speed_data.append(speed)
        self.speed_data_queue.append(speed)

        # Calculate acceleration
        if len(self.speed_data) >= 3:
            speeds = list(self.speed_data)[-3:]
            times = np.linspace(0, 0.02, 3)  # Approximation of time differences

            # Simple acceleration calculation (central difference)
            accel = (speeds[2] - speeds[0]) / 0.02  # m/s²
            accel += np.random.normal(0, sensor_noise * abs(accel)) if accel != 0 else 0

            self.acceleration_history.append(accel)

        # Check if robot has fallen over
        if abs(np.degrees(theta_new)) > 45:  # 45 degrees threshold
            self.fallen = True

        return self.state

    def motor_a(self, power):
        """Set the left motor power.

        Args:
            power: Motor power from -100 to 100
        """
        # Convert power range to electric signal equivalently to the real robot
        power = max(-100, min(100, power))  # Clip power to valid range
        self.left_power_data.append(power)

    def motor_b(self, power):
        """Set the right motor power.

        Args:
            power: Motor power from -100 to 100
        """
        # Convert power range to electric signal equivalently to the real robot
        power = max(-100, min(100, power))  # Clip power to valid range
        self.right_power_data.append(power)

    def stop(self):
        """Stop both motors."""
        self.motor_a(0)
        self.motor_b(0)
        self.error_sum = 0
        self.direction = 0
        return self.state[2], 0, 0

    def balance(self, current_speed=0, target_speed=0):
        """Balance the robot using PID control.

        This function mimics the balance method in the actual robot.
        """
        # Get current angle from state
        current_angle = np.degrees(self.state[2])
        self.angle_data.append(current_angle)

        # Add some sensor noise
        sensor_noise = self.physical_params["sensor_noise_std"]
        current_angle += np.random.normal(0, sensor_noise)

        # Calculate the error relative to the balance target
        self.set_balance_target(current_speed, target_speed)
        error = self.balance_target - current_angle

        # Calculate time difference
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Accumulate the error for the integral term (with anti-windup limits)
        self.error_sum = max(-300, min(300, self.error_sum + error * dt))

        # Calculate PID components
        p_term = self.balance_kp * error
        i_term = self.balance_ki * self.error_sum
        d_term = self.balance_kd * ((error - self.last_error) / dt) if dt > 0 else 0

        # Sum the PID terms to get the control output (balance power)
        balance_power = p_term + i_term + d_term

        # Update last_error for the next derivative calculation
        self.last_error = error

        # Constrain the output power to safe limits
        balance_power = max(-100, min(100, balance_power))

        # Apply the same power to both motors to correct the tilt
        self.motor_a(int(balance_power))
        self.motor_b(int(balance_power))

        # Store the computed power for analysis
        self.left_power_data.append(balance_power)
        self.right_power_data.append(balance_power)

        return current_angle, balance_power

    def forward(self, target_speed=0, turn_bias=0):
        """Balance and move with optional turning.

        Args:
            target_speed: Target forward speed (-10 to 10)
            turn_bias: Turning bias (-100 to 100), positive turns right

        Returns:
            (current_angle, left_power, right_power, current_speed, current_acceleration)

        This function mimics the forward method in the actual robot.
        """
        # Get current speed
        current_speed = self.speed_data[-1] if self.speed_data else 0

        # Use the balance method with target speed
        current_angle, driving_power = self.balance(
            current_speed=current_speed, target_speed=target_speed
        )

        # Set direction for wheel encoder based on target
        if target_speed > 0:
            self.direction = 1
        elif target_speed < 0:
            self.direction = -1
        else:
            self.direction = 0

        # Implement safety check - reduce speed if robot is tilting too much
        tilt_safety_threshold = 15.0  # Degrees
        if abs(current_angle) > tilt_safety_threshold:
            # Reduce speed proportionally to how much we're tilting
            tilt_factor = min(1.0, (abs(current_angle) - tilt_safety_threshold) / 10.0)
            power_reduction = (
                tilt_factor * abs(target_speed) * 0.8
            )  # 80% reduction at max

            # Apply reduction in the direction of motion
            if target_speed > 0:
                driving_power -= power_reduction
            elif target_speed < 0:
                driving_power += power_reduction

        # Calculate left/right powers for turning
        if turn_bias == 0:
            left_power = right_power = driving_power
        else:
            # Reduce turning intensity at higher speeds
            speed_factor = min(1.0, abs(target_speed) / 5.0)
            adjusted_turn_bias = turn_bias * (1.0 - 0.5 * speed_factor)

            factor = abs(adjusted_turn_bias) / 100.0
            if adjusted_turn_bias > 0:
                left_power = driving_power
                right_power = driving_power * (1 - factor)
            else:
                left_power = driving_power * (1 - factor)
                right_power = driving_power

        # Constrain powers to valid range
        left_power = max(-100, min(100, left_power))
        right_power = max(-100, min(100, right_power))

        # Apply powers to motors
        self.motor_a(int(left_power))
        self.motor_b(int(right_power))

        # Advance simulation
        self.simulate()

        # Get current speed and acceleration from simulation
        current_speed = self.speed_data[-1] if self.speed_data else 0
        current_acceleration = (
            self.acceleration_history[-1] if self.acceleration_history else 0
        )

        return (
            current_angle,
            left_power,
            right_power,
            current_speed,
            current_acceleration,
        )


def test_simulation(duration=10, use_controller=True):
    """Test the robot simulation with a simple controller.

    Args:
        duration: Duration of simulation in seconds
        use_controller: Whether to use the built-in controller or apply fixed torques
    """
    # Create a simulation instance
    robot = RobotSimulation()

    # Simulation loop
    start_time = time.time()
    print("Starting simulation...")
    print("Time   |    Angle    |    Speed    |   Accel   |  Left PWM  |  Right PWM")
    print("-------|-------------|-------------|-----------|------------|------------")

    try:
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time

            if use_controller:
                # Use built-in controller with changing target speed
                target_speed = 2.0 * np.sin(elapsed / 2)  # Sinusoidal target speed
                angle, left, right, speed, accel = robot.forward(
                    target_speed=target_speed
                )
            else:
                # Apply fixed control inputs for testing
                # Alternate between leaning forward and backward
                if int(elapsed) % 4 < 2:
                    robot.motor_a(20)
                    robot.motor_b(20)
                else:
                    robot.motor_a(-20)
                    robot.motor_b(-20)

                # Advance simulation
                robot.simulate()
                angle = np.degrees(robot.state[2])
                speed = robot.state[1]
                accel = 0
                left = robot.left_power_data[-1] if robot.left_power_data else 0
                right = robot.right_power_data[-1] if robot.right_power_data else 0

            # Print current state
            print(
                f"{elapsed:5.1f}s | {angle:+8.2f}° | {speed:+8.2f}m/s | {accel:+8.2f} | {left:+8.1f} | {right:+8.1f}"
            )

            # Check if robot has fallen
            if abs(angle) > 45:
                print("Robot has fallen over!")
                break

            # Small delay to not spam console
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")

    # Stop motors
    robot.stop()
    print("Simulation complete.")


def create_simulation_with_measured_params():
    """Create a simulation instance with parameters measured from a physical robot."""
    print("Enter the measured parameters of your physical robot:")
    print("(Press Enter to use default values)")

    params = {}

    try:
        # Body parameters
        body_mass = input("Body mass in kg (default 0.5): ")
        if body_mass:
            params["body_mass"] = float(body_mass)

        body_height = input(
            "Height of center of mass from wheel axis in meters (default 0.15): "
        )
        if body_height:
            params["body_height"] = float(body_height)

        body_width = input("Width of the robot body in meters (default 0.1): ")
        if body_width:
            params["body_width"] = float(body_width)

        # Wheel parameters
        wheel_radius = input("Wheel radius in meters (default 0.04): ")
        if wheel_radius:
            params["wheel_radius"] = float(wheel_radius)

        wheel_distance = input("Distance between wheels in meters (default 0.12): ")
        if wheel_distance:
            params["wheel_distance"] = float(wheel_distance)

        wheel_mass = input("Mass of each wheel in kg (default 0.1): ")
        if wheel_mass:
            params["wheel_mass"] = float(wheel_mass)

        # Environmental parameters
        rolling_resistance = input("Rolling resistance coefficient (default 0.01): ")
        if rolling_resistance:
            params["rolling_resistance"] = float(rolling_resistance)

        # Create simulation with provided parameters
        robot = RobotSimulation(params)

        print("\nSimulation created with the following parameters:")
        for key, value in robot.physical_params.items():
            print(f"  {key}: {value}")

        return robot

    except ValueError:
        print("Invalid input. Using default parameters.")
        return RobotSimulation()


def measure_parameter_guide():
    """Print a guide for measuring physical parameters of the robot."""
    print("\n=== Guide to Measuring Physical Parameters ===\n")

    print("1. Body Mass:")
    print("   - Weigh the robot body without wheels")
    print("   - Units: kilograms (kg)")

    print("\n2. Center of Mass Height:")
    print("   - Measure from wheel axis to the center of mass")
    print("   - Method 1: Balance the robot body on a thin edge and measure the height")
    print(
        "   - Method 2: Hang the robot body from two different points and find the intersection"
    )
    print("   - Units: meters (m)")

    print("\n3. Wheel Radius:")
    print("   - Measure from the center to the edge of the wheel")
    print("   - Units: meters (m)")

    print("\n4. Distance Between Wheels:")
    print("   - Measure from the center of one wheel to the center of the other")
    print("   - Units: meters (m)")

    print("\n5. Wheel Mass:")
    print("   - Weigh each wheel")
    print("   - Units: kilograms (kg)")

    print("\n6. Rolling Resistance:")
    print(
        "   - Place the robot on a slight incline and find the angle where it starts rolling"
    )
    print("   - Rolling resistance ≈ sin(angle)")
    print("   - Alternative: typical values are 0.01-0.02 for rubber on hard surface")

    print("\n7. Moment of Inertia (advanced):")
    print("   - For body: Hang the body and measure oscillation period")
    print("   - I = m*g*d*T²/(4π²) where m=mass, d=distance to pivot, T=period")
    print("   - For wheels: I ≈ 0.5*m*r² for a solid disk, where m=mass, r=radius")

    print("\nReminder: Measure in the units specified (meters, kilograms)!")
    print("=================================================\n")


if __name__ == "__main__":
    print("\n=== Two-Wheeled Robot Simulation ===\n")
    print("Options:")
    print("1. Run simulation with default parameters")
    print("2. Create simulation with measured parameters")
    print("3. Guide for measuring physical parameters")
    print("4. Run simulation without controller (fixed inputs)")
    print("5. Exit")

    choice = input("\nEnter your choice (1-5): ")

    if choice == "1":
        duration = float(input("Simulation duration in seconds (default 10): ") or "10")
        test_simulation(duration)
    elif choice == "2":
        robot = create_simulation_with_measured_params()
        duration = float(input("Simulation duration in seconds (default 10): ") or "10")
        # Run simulation with created robot
        start_time = time.time()
        print("Starting simulation...")
        try:
            while time.time() - start_time < duration:
                target_speed = 2.0 * np.sin((time.time() - start_time) / 2)
                angle, left, right, speed, accel = robot.forward(
                    target_speed=target_speed
                )
                print(
                    f"Angle: {angle:+6.2f}° | Speed: {speed:+5.2f}m/s | Motors: {left:+5.1f}, {right:+5.1f}"
                )
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            robot.stop()
    elif choice == "3":
        measure_parameter_guide()
    elif choice == "4":
        duration = float(input("Simulation duration in seconds (default 10): ") or "10")
        test_simulation(duration, use_controller=False)
    else:
        print("Exiting.")
