import sys
import time
import numpy as np
from collections import deque

sys.path.append('..')
from raspberry.motor_controller import MotorController
from angle import AngleSensor

class Driving:
    """A simple, self-balancing driving interface."""

    def __init__(self, angle_sensor=None, balance_target=-2.5):
        """Set up sensor and balance parameters."""
        self.angle_sensor = angle_sensor if angle_sensor else AngleSensor()
        time.sleep(1)  # Stabilize sensor
        
        # Initialize motor controller
        MOTOR_CONFIG = {
            "IN1": 2,
            "IN2": 3,
            "IN3": 4,
            "IN4": 5,
            "ENA": 6,
            "ENB": 7,
            "PWM_FREQ": 1000
        }
        self.motors = MotorController(MOTOR_CONFIG)
        
        # Balance parameters
        self.balance_target = balance_target
        self.balance_kp = 40.0
        self.balance_ki = 40.0
        self.balance_kd = 0.05
        self.sample_time = 0.005
        self.max_safe_tilt = 25.0
        self.error_sum = 0
        self.last_angle = 0
        self.last_update_time = time.time()
        self.angle_data = deque(maxlen=1000)
        self.left_power_data = deque(maxlen=1000)
        self.right_power_data = deque(maxlen=1000)
    
    def _balance_update(self):
        """Balance controller update."""
        current_angle, gyro_angle, acc_angle = self.angle_sensor.update()
        if abs(current_angle) > self.max_safe_tilt:
            print("Tilt too high. Stopping.")
            self.motors.stop()
            return current_angle, 0
        
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        error = current_angle - self.balance_target
        self.error_sum = max(-300, min(300, self.error_sum + error))
        p_term = self.balance_kp * error
        i_term = self.balance_ki * self.error_sum * dt
        d_term = self.balance_kd * (current_angle - self.last_angle) / dt
        balance_power = p_term + i_term + d_term
        self.last_angle = current_angle
        
        return current_angle, balance_power
    
    def forward(self, speed=0, turn_bias=0, duration=None):
        """Forward/backward drive with optional turn."""
        speed = max(-100, min(100, speed))
        turn_bias = max(-100, min(100, turn_bias))

        if duration is not None:
            end_time = time.time() + duration
            while time.time() < end_time:
                angle, balance_power = self._balance_update()
                if abs(angle) > self.max_safe_tilt:
                    return angle, 0, 0
                left_power, right_power = self._calculate_motor_powers(balance_power, speed, turn_bias)
                self.motors.motor_a(left_power)
                self.motors.motor_b(right_power)
                self._record_data(angle, left_power, right_power)
                time.sleep(self.sample_time)
            return self.stop()
        
        angle, balance_power = self._balance_update()
        if abs(angle) > self.max_safe_tilt:
            return angle, 0, 0
        left_power, right_power = self._calculate_motor_powers(balance_power, speed, turn_bias)
        self.motors.motor_a(left_power)
        self.motors.motor_b(right_power)
        self._record_data(angle, left_power, right_power)
        return angle, left_power, right_power
    
    def _calculate_motor_powers(self, balance_power, speed, turn_bias):
        forward_bias = speed * 2.55
        if turn_bias == 0:
            left = balance_power + forward_bias
            right = balance_power + forward_bias
        else:
            factor = abs(turn_bias) / 100.0
            if turn_bias > 0:
                left = balance_power + forward_bias
                right = balance_power + forward_bias * (1 - factor)
            else:
                left = balance_power + forward_bias * (1 - factor)
                right = balance_power + forward_bias
        left = max(-255, min(255, left))
        right = max(-255, min(255, right))
        return int(left * 100 / 255), int(right * 100 / 255)
    
    def stop(self):
        """Stop movement but keep balancing."""
        print("Stopping.")
        for _ in range(20):
            angle, balance_power = self._balance_update()
            if abs(angle) > self.max_safe_tilt:
                return angle, 0, 0
            power_percent = int(balance_power * 100 / 255)
            self.motors.motor_a(power_percent)
            self.motors.motor_b(power_percent)
            self._record_data(angle, power_percent, power_percent)
            time.sleep(self.sample_time)
        print("Stopped.")
        return angle, power_percent, power_percent
    
    def turn(self, angle=None, direction=0):
        """Turn by angle or a small default if angle is None."""
        direction = max(-100, min(100, direction))
        if direction == 0:
            return self.forward(speed=0, turn_bias=0)
        if angle is None:
            angle = 15
        turn_strength = abs(direction) / 100.0
        degrees_per_second = 90 * turn_strength
        turn_time = abs(angle) / degrees_per_second if degrees_per_second != 0 else 0
        print(f"Turning {angle}°.")
        return self.forward(speed=0, turn_bias=direction, duration=turn_time)
    
    def _record_data(self, angle, left, right):
        """Minimal data logging."""
        self.angle_data.append(angle)
        self.left_power_data.append(left)
        self.right_power_data.append(right)