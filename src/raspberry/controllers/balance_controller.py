"""
Self-balancing robot driving controller for Raspberry Pi Pico
"""

import time
from collections import deque

from controllers.motor_controller import MotorController
from controllers.encoder_controller import WheelEncoder
from controllers.gyroscope_controller import MPU6050

import parameters.parameters as params


class Driving:
    """A simple, self-balancing driving interface."""

    def __init__(self, motor_controller: MotorController = None) -> None:
        """
        Initialize the Driving controller, setting up sensors, encoders, and balance parameters.

        Args:
            motor_controller (MotorController, optional): An instance of MotorController. If None, a new one is created from config.
        Returns:
            None
        """
        # Initialize MPU and wheel encoder
        self.mpu = MPU6050()
        self.wheel_encoder_a = WheelEncoder(
            encoder_pin=params.data["ENCODER_CONFIG"]["pin_a"]
        )
        self.wheel_encoder_b = WheelEncoder(
            encoder_pin=params.data["ENCODER_CONFIG"]["pin_b"]
        )
        self.wheel_encoder_a.start()
        self.wheel_encoder_b.start()

        if motor_controller is None:
            motor_config = params.data["MOTOR_CONFIG"]
            # Pass as positional argument, not keyword argument
            self.motors = MotorController(motor_config)
        else:
            self.motors = motor_controller

        # Balance parameters
        self.balance_angle = 0
        self.balance_kp = params.data["PID_CONFIG"]["kp"]
        self.balance_ki = params.data["PID_CONFIG"]["ki"]
        self.balance_kd = params.data["PID_CONFIG"]["kd"]
        self.k_damping = params.data["PID_CONFIG"]["k_damping"]
        self.sample_time = params.data["PID_CONFIG"]["sample_time"]
        self.max_safe_tilt = params.data["PID_CONFIG"]["max_safe_tilt"]
        self.max_correction_tilt = params.data["PID_CONFIG"]["max_correction_tilt"]

        # Initialize PID state
        self.error_sum = 0
        self.last_error = 0
        self.last_update_time = time.time()

        # Data history (limited size to prevent memory issues on Pico)
        self.angle_data = deque([], 100)
        self.balance_angle_data = deque([], 100)
        self.acceleration_data = deque([], 100)
        self.left_power_data = deque([], 100)
        self.right_power_data = deque([], 100)

        # Speed and acceleration tracking
        self.speed_data = deque([], 100)
        self.rpm_data = deque([], 50)
        self.time_data = deque([], 50)
        self.direction = 0  # 0: stopped, 1: forward, -1: backward

        # Add acceleration control parameters (can be tuned with RL)
        self.max_accel = params.data["PID_CONFIG"][
            "max_acceleration"
        ]  # Maximum acceleration in units/second
        self.accel_smoothing = params.data["PID_CONFIG"][
            "acceleration_smoothing"
        ]  # Smoothing factor for acceleration
        self.target_speed = 0  # Target speed to reach
        self.current_target = 0  # Current interpolated target speed
        self.delta_encoder = 0   # number of expected endcoder pulses

    def set_balance_angle(self, target_speed: float = 0.0) -> None:
        """
        Set the balance target angle based on current speed and target speed.

        Args:
            current_speed: Current speed of the robot in m/s
            target_speed: Desired speed of the robot in m/s

        This function computes the optimal tilt angle using a physical model with parameters
        that can be optimized through reinforcement learning. The model includes:
        1. A quadratic term modeling how torque requirements vary with speed
        2. An acceleration component based on speed difference
        3. A drag/friction component proportional to target speed

        The equation satisfies the constraint that when both current_speed and
        target_speed are 0, the balance_angle will be 0.
        """
        # Parameters for tuning via reinforcement learning
        self.k_acc = params.data["PID_CONFIG"][
            "k_acceleration"
        ]  # Coefficient for acceleration/deceleration response
        self.k_torque_per_pw = params.data["PID_CONFIG"][
            "k_torque_per_pw"
        ]  # Models how torque requirements change with speed
        self.drag = params.data["PID_CONFIG"][
            "drag"
        ]  # Coefficient for overcoming friction/drag forces

        # Calculate balance target:
        # - First term: Non-linear acceleration component that increases with speed
        # - Second term: Compensates for friction/drag forces
        # self.balance_angle = (
        #    1 + (self.k_torque_per_pw * current_speed) ** 2
        # ) * self.k_acc * (target_speed - current_speed) + self.drag * target_speed

        # Add a component of stay still
        self.balance_angle = (
            params.data["PID_CONFIG"]["balance2offset"]
            * (self.wheel_encoder_a.pulse_count + self.wheel_encoder_b.pulse_count - 2 * self.delta_encoder)
            / 2
        )

    def balance(self, target_speed: float = 0.0) -> tuple[float, float]:
        """
        Balance the robot without moving (stationary balancing).

        Args:
            current_speed (float): Current speed of the robot in m/s.
            target_speed (float): Desired speed of the robot in m/s.
        Returns:
            tuple[float, float]: (current_angle, balance_power)
        """
        # Get current angle using both gyro and accelerometer for better accuracy
        current_angle = self.mpu.get_current_angle()

        # Calculate time difference
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        self.delta_encoder += dt * target_speed

        # Calculate the error relative to the balance target
        self.set_balance_angle(target_speed)
        error = self.balance_angle - current_angle

        self.angle_data.append(current_angle)
        if len(self.angle_data) > 1:
            self.balance_angle = (
                self.balance_angle
                - (self.balance_angle - self.balance_angle_data[-1]) * self.k_damping
            )
        self.balance_angle_data.append(self.balance_angle)

        self.balance_angle = max(
            -self.max_correction_tilt, min(self.max_correction_tilt, self.balance_angle)
        )

        print("Target Angle:", self.balance_angle)
        print("Current Angle:", current_angle)

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
        self.motors.motor_a(int(balance_power))
        self.motors.motor_b(int(balance_power))
        self.wheel_encoder_a.motor_direction = 1 if balance_power > 0 else -1
        self.wheel_encoder_b.motor_direction = 1 if balance_power > 0 else -1

        # Optionally, store the computed power for analysis
        self.left_power_data.append(balance_power)
        self.right_power_data.append(balance_power)

        return current_angle, balance_power

    def forward(self, target_speed: int = 0, turn_bias: int = 0) -> tuple[float, float, float, float, float]:
        """
        Balance and move with optional turning.

        Args:
            target_speed (int): Forward speed (-100 to 100).
            turn_bias (int): Turning bias (-100 to 100), positive turns right.
        Returns:
            tuple[float, float, float, float, float]: (current_angle, left_power, right_power, current_speed, current_acceleration)
        """
        # Use the balance method with smoothed target speed
        current_angle, driving_power = self.balance(target_speed=target_speed)

        # Set direction for wheel encoder based on current target
        if self.current_target > 0:
            self.direction = 1
        elif self.current_target < 0:
            self.direction = -1
        else:
            self.direction = 0

        # Implement safety check - reduce speed if robot is tilting too much
        tilt_safety_threshold = 15.0  # Degrees (can be tuned via RL)
        if abs(current_angle) > tilt_safety_threshold:
            # Reduce speed proportionally to how much we're tilting
            tilt_factor = min(1.0, (abs(current_angle) - tilt_safety_threshold) / 10.0)
            power_reduction = (
                tilt_factor * abs(self.current_target) * 0.8
            )  # 80% reduction at max

            # Apply reduction in the direction of motion
            if self.current_target > 0:
                driving_power -= power_reduction
            elif self.current_target < 0:
                driving_power += power_reduction

        # Calculate left/right powers for turning
        if turn_bias == 0:
            left_power = right_power = driving_power
        else:
            # Smooth turning too - reduce turning intensity at higher speeds
            speed_factor = min(1.0, abs(self.current_target) / 50.0)
            adjusted_turn_bias = turn_bias * (
                1.0 - 0.5 * speed_factor
            )  # Reduce turning up to 50% at high speeds

            factor = abs(adjusted_turn_bias) / 100.0
            if adjusted_turn_bias > 0:
                left_power = driving_power * (1 + factor)
                right_power = driving_power * (1 - factor)
            else:
                left_power = driving_power * (1 - factor)
                right_power = driving_power * (1 + factor)

        # Constrain powers to valid range
        left_power = max(-100, min(100, left_power))
        right_power = max(-100, min(100, right_power))

        # Apply powers to motors
        self.motors.motor_a(int(left_power))
        self.motors.motor_b(int(right_power))

        # Store data
        self.left_power_data.append(left_power)
        self.right_power_data.append(right_power)

        # Track speed and acceleration
        current_rpm = (
            self.wheel_encoder_a.get_rpm() + self.wheel_encoder_b.get_rpm()
        ) / 2
        current_time = time.ticks_ms()
        self.rpm_data.append(current_rpm)
        self.time_data.append(current_time)

        # Calculate speed and acceleration
        current_speed = (
            self.wheel_encoder_a.get_speed() + self.wheel_encoder_b.get_speed()
        ) / 2
        self.speed_data.append(current_speed)

        # Get acceleration if we have enough data points
        if len(self.rpm_data) >= 3 and len(self.time_data) >= 3:
            current_acceleration = (
                self.wheel_encoder_a.get_absolute_acceleration()
                + self.wheel_encoder_b.get_absolute_acceleration()
            ) / 2
        else:
            current_acceleration = 0

        # Store the acceleration value for RL feedback
        self.acceleration_data.append(current_acceleration)

        return (
            current_angle,
            left_power,
            right_power,
            current_speed,
            current_acceleration,
        )

    def stop(self) -> tuple[float, int, int]:
        """
        Stop both motors and reset error sum and direction.

        Returns:
            tuple[float, int, int]: (gyro_angle, 0, 0)
        """
        self.motors.stop()
        self.error_sum = 0
        self.direction = 0
        gyro_angle = self.mpu.get_current_angle()
        return gyro_angle, 0, 0

    def turn(self, angle: int = 90, direction: int = 1) -> tuple[float, int, int]:
        """
        Turn approximately by the specified angle.

        Args:
            angle (int): Angle to turn in degrees (positive).
            direction (int): 1 for right, -1 for left.
        Returns:
            tuple[float, int, int]: (gyro_angle, 0, 0) after turning.
        """
        print(f"Turning {angle}° {'right' if direction > 0 else 'left'}")

        # Get initial angle from gyro
        initial_z_angle = self.mpu.angle_z
        target_z_angle = initial_z_angle + (angle * direction)

        power = 30 * direction  # Use moderate power
        self.motors.motor_a(-power)
        self.motors.motor_b(power)

        # Monitor turning using gyro z-axis
        while abs(self.mpu.angle_z - target_z_angle) > 5:  # Allow 5° tolerance
            _ = self.mpu.get_current_angle()  # Update angles
            time.sleep(0.01)

            # Safety timeout
            if time.time() - self.last_update_time > 5:  # 5 second timeout
                break

        return self.stop()
