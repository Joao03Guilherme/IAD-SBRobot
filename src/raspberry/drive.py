"""
Self-balancing robot driving controller for Raspberry Pi Pico
"""

import time
from collections import deque
from machine import Pin, I2C

from motor_controller import MotorController
import accelerometer

class Driving:
    """A simple, self-balancing driving interface."""
    def __init__(self, motor_controller=None):
        """Set up sensor and balance parameters."""
        # Initialize gyroscope and accelerometer angle sensor
        accelerometer.mpu_init()
        # Use provided motor controller or create a new one
        if motor_controller is None:
            # Create motor configuration dictionary first
            motor_config = {
                "IN1": 2,
                "IN2": 3,
                "IN3": 4,
                "IN4": 5,
                "ENA": 6,
                "ENB": 7,
                "PWM_FREQ": 1000
            }
            # Pass as positional argument, not keyword argument
            self.motors = MotorController(motor_config)
        else:
            self.motors = motor_controller
        
        # Balance parameters
        self.balance_target = -2.5
        self.balance_kp = 5.0
        self.balance_ki = 0.1
        self.balance_kd = 0.5
        self.sample_time = 0.01
        self.max_safe_tilt = 25.0
        
        # Initialize PID state
        self.error_sum = 0
        self.last_error = 0
        self.last_update_time = time.time()
        
        # Data history (limited size to prevent memory issues on Pico)
        self.angle_data = deque([],100)
        self.left_power_data = deque([], 100)
        self.right_power_data = deque([], 100)
    
    def forward(self, speed=0, turn_bias=0):
        """Balance and move with optional turning.
        Args:
            speed: Forward speed (-100 to 100)
            turn_bias: Turning bias (-100 to 100), positive turns right
        Returns:
            (current_angle, left_power, right_power)
        """
        # Get current angle
        current_angle, _, _ = self.angle_sensor.update()
        self.angle_data.append(current_angle)
        
        # Calculate error
        error = current_angle - self.balance_target
        
        # Calculate PID terms
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        self.error_sum = max(-300, min(300, self.error_sum + error * dt))
        
        p_term = self.balance_kp * error
        i_term = self.balance_ki * self.error_sum
        d_term = self.balance_kd * (error - self.last_error) / dt if dt > 0 else 0
        
        # Calculate base power from PID controller
        balance_power = p_term + i_term + d_term
        
        # Add forward/backward motion
        power_with_speed = balance_power + speed
        
        # Calculate left/right powers for turning
        if turn_bias == 0:
            left_power = right_power = power_with_speed
        else:
            factor = abs(turn_bias) / 100.0
            if turn_bias > 0:
                left_power = power_with_speed
                right_power = power_with_speed * (1 - factor)
            else:
                left_power = power_with_speed * (1 - factor)
                right_power = power_with_speed
        
        # Constrain powers to valid range
        left_power = max(-100, min(100, left_power))
        right_power = max(-100, min(100, right_power))
        
        # Apply powers to motors
        self.motors.motor_a(int(left_power))
        self.motors.motor_b(int(right_power))
        
        # Store data
        self.left_power_data.append(left_power)
        self.right_power_data.append(right_power)
        
        # Update state for next iteration
        self.last_error = error
        
        return current_angle, left_power, right_power
    
    def stop(self):
        """Stop both motors."""
        self.motors.stop()
        self.error_sum = 0
        return self.angle_sensor.update()[0], 0, 0
    
    def turn(self, angle=90, direction=1):
        """Turn approximately by the specified angle.
        Args:
            angle: Angle to turn in degrees (positive)
            direction: 1 for right, -1 for left
        """
        print(f"Turning {angle}° {'right' if direction > 0 else 'left'}")
        
        # Simple time-based turning
        power = 30 * direction  # Use moderate power
        self.motors.motor_a(-power)
        self.motors.motor_b(power)
        
        # Wait based on approximate degrees per second
        time.sleep(angle / 90.0)  # Calibrated for ~90° per second
        
        return self.stop()

def test_balance():
    """Simple test of the balancing behavior."""
    driver = Driving()
    print("Starting balance test...")
    print("Use Ctrl+C to stop")
    
    try:
        while True:
            angle, left, right = driver.forward()
            print(f"Angle: {angle:6.2f}° | Left: {left:4.0f}% | Right: {right:4.0f}%")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        driver.stop()

if __name__ == "__main__":
    test_balance()