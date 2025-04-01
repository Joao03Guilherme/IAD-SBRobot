from machine import Pin, I2C
import time
import math

class MPU6050:
    # MPU6050 I2C Address
    MPU_ADDR = 0x68
    
    # Register addresses
    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40
    
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48
    
    # Gyroscope bias
    BIAS_X = -1440.115
    BIAS_Y = 226.0664
    BIAS_Z = -1314.369
    
    # Default is ±250 deg/s range, which gives 131 LSB/(deg/s)
    GYRO_SCALE_FACTOR = 131.0
    
    # Time constant for complementary filter (in seconds)
    FILTER_TIME_CONSTANT = 10
    
    def __init__(self, sda_pin=26, scl_pin=27):
        # Initialize I2C with SDA and SCL pins
        self.i2c = I2C(1, sda=Pin(sda_pin), scl=Pin(scl_pin))
        
        # Initialize angle tracking values
        self.prev_time = 0
        self.angle_x, self.angle_y, self.angle_z = 0, 0, 0
        
        # Initialization now explicitly named
        self.initialize_mpu()
    
    def initialize_mpu(self):
        # Wake up the MPU6050 (it is in sleep mode by default after power-up)
        try:
            self.i2c.writeto_mem(self.MPU_ADDR, 0x6B, b"\x00")  # Write to PWR_MGMT_1 register
        except OSError as e:
            print("MPU6050 initialization error:", e)
    
    def read_word(self, register):
        high = self.i2c.readfrom_mem(self.MPU_ADDR, register, 1)
        low = self.i2c.readfrom_mem(self.MPU_ADDR, register + 1, 1)
        value = (high[0] << 8) + low[0]
        # Convert to signed 16-bit if the value is negative
        if value >= 0x8000:
            value -= 0x10000
        return value
    
    def read_accel(self):
        ax = self.read_word(self.ACCEL_XOUT_H)
        ay = self.read_word(self.ACCEL_YOUT_H)
        az = self.read_word(self.ACCEL_ZOUT_H)
        return ax, ay, az
    
    def read_gyro(self):
        gx = self.read_word(self.GYRO_XOUT_H) - self.BIAS_X
        gy = self.read_word(self.GYRO_YOUT_H) - self.BIAS_Y
        gz = self.read_word(self.GYRO_ZOUT_H) - self.BIAS_Z
        return gx, gy, gz
    
    def calc_accel_angles(self):
        ax, ay, az = self.read_accel()
        # Convert to g forces (assuming ±2g range, which is 16384 LSB/g)
        ax_g = ax / 16384.0
        ay_g = ay / 16384.0
        az_g = az / 16384.0
        
        # Calculate angles using arctangent
        accel_angle_x = (
            math.atan2(ay_g, math.sqrt(ax_g**2 + az_g**2)) * 180.0 / math.pi
        )  # Roll
        accel_angle_y = (
            math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi
        )  # Pitch
        
        return accel_angle_x, accel_angle_y
    
    def angle_from_gyro(self):
        # Get current time in milliseconds
        curr_time = time.ticks_ms()
        
        # Calculate time difference in seconds
        if self.prev_time == 0:
            dt = 0
        else:
            dt = time.ticks_diff(curr_time, self.prev_time) / 1000.0
        
        self.prev_time = curr_time
        
        # Skip integration on first call
        if dt == 0:
            return 0, 0, 0
        
        # Read gyroscope data (angular velocity)
        gx, gy, gz = self.read_gyro()
        
        # Convert raw gyro values to degrees per second
        gx_dps = gx / self.GYRO_SCALE_FACTOR
        gy_dps = gy / self.GYRO_SCALE_FACTOR
        gz_dps = gz / self.GYRO_SCALE_FACTOR
        
        # Calculate gyro angle change
        gyro_angle_x = gx_dps * dt
        gyro_angle_y = gy_dps * dt
        gyro_angle_z = gz_dps * dt
        filter_coef = self.FILTER_TIME_CONSTANT / (self.FILTER_TIME_CONSTANT + dt)
        
        # Update angles with the filtered values
        self.angle_x = filter_coef * (self.angle_x + gyro_angle_x) - (1 - filter_coef) * self.angle_x
        self.angle_y = filter_coef * (self.angle_y + gyro_angle_y) - (1 - filter_coef) * self.angle_y
        self.angle_z = filter_coef * (self.angle_z + gyro_angle_z) - (1 - filter_coef) * self.angle_z
        
        return self.angle_x, self.angle_y, self.angle_z


class WheelEncoder:
    def __init__(self, encoder_pin=28, pulses_per_rev=20, wheel_diameter=0.067):
        self.ENCODER_PIN = encoder_pin
        self.PULSES_PER_REV = pulses_per_rev
        self.WHEEL_DIAMETER = wheel_diameter
        
        self.pulse_count = 0
        self.encoder = None
        
        # Initialization now explicitly named
        self.initialize_encoder()
    
    def initialize_encoder(self):
        # Configure the encoder pin as input with internal pull-up resistor
        self.encoder = Pin(self.ENCODER_PIN, Pin.IN, Pin.PULL_UP)
        # Attach an interrupt on the rising edge
        self.encoder.irq(trigger=Pin.IRQ_RISING, handler=self._irq_handler)
    
    def _irq_handler(self, pin):
        self.pulse_count += 1
    
    def get_rpm(self, interval_ms=1000):
        # Copy the pulse count and then reset it
        count = self.pulse_count
        self.pulse_count = 0
        # Calculate RPM: (pulses / pulses per rev) gives revolutions per interval,
        # then multiply by (60000 / interval_ms) to convert to revolutions per minute.
        rpm = (count / self.PULSES_PER_REV) * (60000 / interval_ms)
        return rpm
    
    def get_speed(self, rpm_mean, direction):
        wheel_circumference = math.pi * self.WHEEL_DIAMETER
        # Speed in m/s = (RPM * circumference) / 60
        speed_m_s = (rpm_mean * wheel_circumference) / 60
        # Adjust speed based on direction
        if direction == 1:  # Forward
            speed_m_s = abs(speed_m_s)
        elif direction == -1:  # Backward
            speed_m_s = -abs(speed_m_s)
        else:
            speed_m_s = 0  # No movement
        return speed_m_s
    
    def get_absolute_acceleration(self, rpms, times, direction):
        wheel_circumference = math.pi * self.WHEEL_DIAMETER
        
        # Calculate the slope (acceleration) using linear regression
        n = len(rpms)
        sum_x = sum(times)
        sum_y = sum(rpms)
        sum_xy = sum(x * y for x, y in zip(times, rpms))
        sum_x_squared = sum(x**2 for x in times)
        
        # Avoid division by zero
        if (n * sum_x_squared - sum_x**2) == 0:
            return 0
            
        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x_squared - sum_x**2)
        # Calculate the acceleration in m/s²
        acceleration = (slope * wheel_circumference * 1000) / 60
        # Adjust acceleration based on direction
        if direction == 1:  # Forward
            acceleration = abs(acceleration)
        elif direction == -1:  # Backward
            acceleration = -abs(acceleration)
        else:
            acceleration = 0  # No movement
        return acceleration


# Example usage
if __name__ == "__main__":
    # Initialize the MPU6050 sensor
    mpu = MPU6050()
    
    # Initialize the wheel encoder
    encoder = WheelEncoder()
    
    # Cycle to continuously read sensor data
    while True:
        # Read gyro angles from MPU6050
        angle_x, angle_y, angle_z = mpu.angle_from_gyro()
        # Read accelerometer angles
        accel_angle_x, accel_angle_y = mpu.calc_accel_angles()
        
        # Read wheel encoder data in a series of measurements
        wheel_rpm_list = []
        time_list = []
        for i in range(50):
            wheel_rpm = encoder.get_rpm()
            wheel_rpm_list.append(wheel_rpm)
            time_list.append(time.ticks_ms())
            time.sleep_us(100)
        
        # Calculate speed and acceleration from the collected data
        wheel_rpm = wheel_rpm_list[-1] if wheel_rpm_list else 0
        wheel_speed = encoder.get_speed(wheel_rpm, 1)
        wheel_acceleration = encoder.get_absolute_acceleration(wheel_rpm_list, time_list, 1)
        
        print(f"Angles gyro (x,y,z): ({angle_x:.2f}, {angle_y:.2f}, {angle_z:.2f})")
        print(f"Angles accel (x,y): ({accel_angle_x:.2f}, {accel_angle_y:.2f})")
        print(f"Wheel RPM: {wheel_rpm:.2f}, Speed: {wheel_speed:.2f} m/s, Acceleration: {wheel_acceleration:.2f} m/s²")