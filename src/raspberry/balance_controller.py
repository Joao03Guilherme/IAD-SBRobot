from machine import Pin
import math

class BalanceController:
    def __init__(self, pid_config, mpu, motors):
        self.kp = pid_config["Kp"]
        self.ki = pid_config["Ki"]
        self.kd = pid_config["Kd"]
        self.sample_time = pid_config["sample_time"]
        self.target_angle = pid_config["target_angle"]
        
        self.mpu = mpu
        self.motors = motors
        
        # Initialize tracking variables
        self.motor_power = 0
        self.current_angle = 0
        self.prev_angle = 0
        self.error = 0
        self.error_sum = 0
        self.count = 0
        
        # Status LED
        self.led = Pin(25, Pin.OUT)
    
    def map_value(self, x, in_min, in_max, out_min, out_max):
        """Arduino-like map function"""
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    
    def update(self):
        """Calculate PID values and update motor power"""
        # Read sensor data
        _, acc_y, acc_z = self.mpu.read_accel()
        gyro_x, _, _ = self.mpu.read_gyro()
        
        # Calculate angle (complementary filter)
        acc_angle = math.atan2(acc_y, acc_z) * 57.2957795  # RAD_TO_DEG
        gyro_rate = self.map_value(gyro_x, -32768, 32767, -250, 250)
        gyro_angle = gyro_rate * self.sample_time
        self.current_angle = 0.9934 * (self.prev_angle + gyro_angle) + 0.0066 * acc_angle
        
        # PID calculation
        self.error = self.current_angle - self.target_angle
        self.error_sum += self.error
        self.error_sum = max(-300, min(self.error_sum, 300))  # Constrain
        
        # PID formula
        self.motor_power = (self.kp * self.error + 
                           self.ki * self.error_sum * self.sample_time - 
                           self.kd * (self.current_angle - self.prev_angle) / self.sample_time)
        
        self.prev_angle = self.current_angle
        
        # Constrain motor power
        self.motor_power = max(-255, min(self.motor_power, 255))
        
        # Apply to motors
        self.motors.set_speeds(self.motor_power, self.motor_power)
        
        # LED blink
        self.count += 1
        if self.count >= 200:  # 1 second (200 * 5ms)
            self.count = 0
            self.led.toggle()
        
        # Return current values for debugging
        return self.current_angle, self.motor_power