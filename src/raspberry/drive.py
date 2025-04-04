"""
Self-balancing robot driving controller for Raspberry Pi Pico
"""

import time
from collections import deque

from controllers.motor_controller import MotorController
from accelerometer import MPU6050, WheelEncoder 


class Driving:
    """A simple, self-balancing driving interface."""

    def __init__(self, motor_controller=None):
        """Set up sensor and balance parameters."""
        # Initialize MPU and wheel encoder
        self.mpu = MPU6050()
        self.wheel_encoder = WheelEncoder()

        if motor_controller is None:
            # Create motor configuration dictionary first
            motor_config = {
                "IN1": 2,
                "IN2": 3,
                "IN3": 4,
                "IN4": 5,
                "ENA": 6,
                "ENB": 7,
                "PWM_FREQ": 1000,
            }
            # Pass as positional argument, not keyword argument
            self.motors = MotorController(motor_config)
        else:
            self.motors = motor_controller

        # Balance parameters
        self.balance_target = 0
        self.balance_kp = 50
        self.balance_ki = 30
        self.balance_kd = 90
        self.sample_time = 0.0001
        self.max_safe_tilt = 100 # TODO: CHANGE TO 5

        # Initialize PID state
        self.error_sum = 0
        self.last_error = 0
        self.last_update_time = time.time()

        # Data history (limited size to prevent memory issues on Pico)
        self.angle_data = deque([], 100)
        self.left_power_data = deque([], 100)
        self.right_power_data = deque([], 100)
        
        # Speed and acceleration tracking
        self.speed_data = deque([], 100)
        self.rpm_data = deque([], 50)
        self.time_data = deque([], 50)
        self.direction = 0  # 0: stopped, 1: forward, -1: backward
        
        # Add acceleration control parameters (can be tuned with RL)
        self.max_accel = 10.0       # Maximum acceleration in units/second
        self.accel_smoothing = 0.2  # Lower values = smoother acceleration (0-1)
        self.target_speed = 0       # Target speed to reach
        self.current_target = 0     # Current interpolated target speed

    def set_balance_target(self, current_speed=0, target_speed=0):
        """Set the balance target angle based on current speed and target speed.
        
        Args:
            current_speed: Current speed of the robot in m/s
            target_speed: Desired speed of the robot in m/s
            
        This function computes the optimal tilt angle using a physical model with parameters
        that can be optimized through reinforcement learning. The model includes:
        1. A quadratic term modeling how torque requirements vary with speed
        2. An acceleration component based on speed difference
        3. A drag/friction component proportional to target speed
        
        The equation satisfies the constraint that when both current_speed and 
        target_speed are 0, the balance_target will be 0.
        """
        # Parameters for tuning via reinforcement learning
        self.k_acc = 2.0         # Coefficient for acceleration/deceleration response
        self.k_torque_per_pw = 0.5  # Models how torque requirements change with speed
        self.drag = 1.0          # Coefficient for overcoming friction/drag forces
        
        # Calculate balance target:
        # - First term: Non-linear acceleration component that increases with speed
        # - Second term: Compensates for friction/drag forces
        self.balance_target = (
            (1 + (self.k_torque_per_pw * current_speed) ** 2 ) * self.k_acc * (target_speed - current_speed) + 
            self.drag * target_speed
        )
        
        # Constrain the balance target to safe limits
        self.balance_target = max(-self.max_safe_tilt, 
                                  min(self.max_safe_tilt, self.balance_target))

    def balance(self, current_speed=0, target_speed=0):
        """Balance the robot without moving (stationary balancing)."""
        # Get current angle using both gyro and accelerometer for better accuracy
        gyro_angle_x, _, _ = self.mpu.angle_from_gyro()
        accel_angle_x, _ = self.mpu.calc_accel_angles()

        # Apply complementary filter to combine gyro and accelerometer data
        alpha = 0.97
        current_angle = alpha * gyro_angle_x + (1 - alpha) * accel_angle_x

        self.angle_data.append(current_angle)

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
        self.motors.motor_a(int(balance_power))
        self.motors.motor_b(int(balance_power))

        # Optionally, store the computed power for analysis
        self.left_power_data.append(balance_power)
        self.right_power_data.append(balance_power)

        return current_angle, balance_power

    def forward(self, target_speed=0, turn_bias=0):
        """Balance and move with optional turning.
        Args:
            speed: Forward speed (-100 to 100)
            turn_bias: Turning bias (-100 to 100), positive turns right
        Returns:
            (current_angle, left_power, right_power, current_speed, current_acceleration)
        """
        current_speed = self.wheel_encoder.get_speed(self.direction)

        # Use the balance method with smoothed target speed
        current_angle, driving_power = self.balance(
            current_speed=current_speed, 
            target_speed=target_speed
        )
        
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
            power_reduction = tilt_factor * abs(self.current_target) * 0.8  # 80% reduction at max
            
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
            adjusted_turn_bias = turn_bias * (1.0 - 0.5 * speed_factor)  # Reduce turning up to 50% at high speeds
            
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
        self.motors.motor_a(int(left_power))
        self.motors.motor_b(int(right_power))

        # Store data
        self.left_power_data.append(left_power)
        self.right_power_data.append(right_power)
        
        # Track speed and acceleration
        current_rpm = self.wheel_encoder.get_rpm()
        current_time = time.ticks_ms()
        self.rpm_data.append(current_rpm)
        self.time_data.append(current_time)
        
        # Calculate speed and acceleration
        current_speed = self.wheel_encoder.get_speed(self.direction)
        self.speed_data.append(current_speed)
        
        # Get acceleration if we have enough data points
        if len(self.rpm_data) >= 3 and len(self.time_data) >= 3:
            current_acceleration = self.wheel_encoder.get_absolute_acceleration(self.direction)
        else:
            current_acceleration = 0
            
        # Store the acceleration value for RL feedback
        if not hasattr(self, 'acceleration_history'):
            self.acceleration_history = deque([], 20)
        self.acceleration_history.append(current_acceleration)

        return current_angle, left_power, right_power, current_speed, current_acceleration

    def stop(self):
        """Stop both motors."""
        self.motors.stop()
        self.error_sum = 0
        self.direction = 0
        gyro_angle, _, _ = self.mpu.angle_from_gyro()
        return gyro_angle, 0, 0

    def turn(self, angle=90, direction=1):
        """Turn approximately by the specified angle.
        Args:
            angle: Angle to turn in degrees (positive)
            direction: 1 for right, -1 for left
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
            _, _, _ = self.mpu.angle_from_gyro()  # Update angles
            time.sleep(0.01)
            
            # Safety timeout
            if time.time() - self.last_update_time > 5:  # 5 second timeout
                break

        return self.stop()


def test_balance():
    """Simple test of the balancing behavior."""
    driver = Driving()
    print("Starting balance test...")
    print("Use Ctrl+C to stop")

    try:
        while True:
            angle, left, right, speed, accel = driver.forward()
            print(f"Angle: {angle:6.2f}° | Left: {left:4.0f}% | Right: {right:4.0f}% | Speed: {speed:.2f} m/s | Accel: {accel:.2f} m/s²")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        driver.stop()


if __name__ == "__main__":
    test_balance()
