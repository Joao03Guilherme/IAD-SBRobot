from machine import Pin, I2C, PWM
import math
import time

# ======== CONFIGURATION ========
# PID Constants
PID_CONFIG = {
    "Kp": 40,
    "Ki": 40,
    "Kd": 0.05,
    "sample_time": 0.005,  # 5ms
    "target_angle": -2.5
}

# Motor pin configuration
MOTOR_CONFIG = {
    "IN1": 2,
    "IN2": 3,
    "IN3": 4,
    "IN4": 5,
    "ENA": 6,
    "ENB": 7,
    "PWM_FREQ": 1000
}

# MPU6050 configuration
MPU_CONFIG = {
    "i2c_id": 1,
    "sda_pin": 26,
    "scl_pin": 27,
    "address": 0x68
}

# ======== MPU6050 SENSOR CLASS ========
class MPU6050:
    # Register addresses
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43

    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.init_sensor()
    
    def init_sensor(self):
        """Initialize the MPU6050 sensor"""
        # Wake up MPU6050 from sleep mode
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
    
    def read_word(self, register):
        """Read 16-bit value from registers"""
        high = self.i2c.readfrom_mem(self.addr, register, 1)
        low = self.i2c.readfrom_mem(self.addr, register + 1, 1)
        value = (high[0] << 8) + low[0]
        
        # Convert to signed 16-bit
        if value >= 0x8000:
            value -= 0x10000
        return value
    
    def read_accel(self):
        """Read accelerometer values"""
        ax = self.read_word(self.ACCEL_XOUT_H)
        ay = self.read_word(self.ACCEL_YOUT_H)
        az = self.read_word(self.ACCEL_ZOUT_H)
        return ax, ay, az
    
    def read_gyro(self):
        """Read gyroscope values"""
        gx = self.read_word(self.GYRO_XOUT_H)
        gy = self.read_word(self.GYRO_XOUT_H + 2)
        gz = self.read_word(self.GYRO_XOUT_H + 4)
        return gx, gy, gz

# ======== MOTOR CONTROLLER CLASS ========
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

# ======== MAIN PROGRAM ========
def main():
    print("Initializing self-balancing robot...")
    
    # Initialize I2C for MPU6050
    i2c = I2C(MPU_CONFIG["i2c_id"], 
              sda=Pin(MPU_CONFIG["sda_pin"]), 
              scl=Pin(MPU_CONFIG["scl_pin"]))
    
    # Create objects
    mpu = MPU6050(i2c, MPU_CONFIG["address"])
    motors = MotorController(MOTOR_CONFIG)
    controller = BalanceController(PID_CONFIG, mpu, motors)
    
    print("Setup complete")
    
    # Wait for accelerometer to stabilize
    time.sleep(1)
    
    # Balance control loop
    last_time = time.ticks_ms()
    debug_count = 0
    
    try:
        print("Starting balance control...")
        while True:
            current_time = time.ticks_ms()
            
            # Update PID at regular intervals (5ms)
            if time.ticks_diff(current_time, last_time) >= 5:
                angle, power = controller.update()
                last_time = current_time
                
                # Debug output (every 100 iterations = 0.5 seconds)
                debug_count += 1
                if debug_count >= 100:
                    debug_count = 0
                    print(f"Angle: {angle:.2f}, Power: {power}")

    except KeyboardInterrupt:
        # Clean shutdown
        motors.stop()
        print("Program stopped")
    except Exception as e:
        # Error handling
        motors.stop()
        print(f"Error: {e}")
        raise

if __name__ == "__main__":
    main()