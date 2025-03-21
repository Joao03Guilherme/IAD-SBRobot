from machine import Pin, PWM
import math

class MotorController:
    def __init__(self, config):
        # Setup motor pins
        self.IN1 = Pin(config["IN1"], Pin.OUT)
        self.IN2 = Pin(config["IN2"], Pin.OUT)
        self.IN3 = Pin(config["IN3"], Pin.OUT)
        self.IN4 = Pin(config["IN4"], Pin.OUT)
        
        # Setup PWM for speed control
        self.ENA = PWM(Pin(config["ENA"]))
        self.ENB = PWM(Pin(config["ENB"]))
        
        # Set PWM frequency
        self.ENA.freq(config["PWM_FREQ"])
        self.ENB.freq(config["PWM_FREQ"])
        
        # Initialize motors to stopped state
        self.stop()
    
    def motor_a(self, speed):
        """Control Motor A"""
        duty = int(abs(speed) * 65535 / 100)
        
        if speed > 0:
            self.IN1.value(1)
            self.IN2.value(0)
        elif speed < 0:
            self.IN1.value(0)
            self.IN2.value(1)
        else:
            self.IN1.value(0)
            self.IN2.value(0)

        self.ENA.duty_u16(duty)
    
    def motor_b(self, speed):
        """Control Motor B"""
        duty = int(abs(speed) * 65535 / 100)

        if speed > 0:
            self.IN3.value(1)
            self.IN4.value(0)
        elif speed < 0:
            self.IN3.value(0)
            self.IN4.value(1)
        else:
            self.IN3.value(0)
            self.IN4.value(0)

        self.ENB.duty_u16(duty)
    
    def set_speeds(self, left_speed, right_speed):
        """Set both motor speeds (-255 to 255 scale)"""
        # Convert from Arduino's -255 to 255 range to -100 to 100 percentage
        left_scaled = (left_speed * 100) // 255
        right_scaled = (right_speed * 100) // 255
        
        self.motor_a(left_scaled)
        self.motor_b(right_scaled)
    
    def stop(self):
        """Stop both motors"""
        self.motor_a(0)
        self.motor_b(0)

# ======== PID CONTROLLER CLASS ========
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