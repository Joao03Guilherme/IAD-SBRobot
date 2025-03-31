from machine import Pin, I2C
import time
import math

# MPU6050 I2C Address (default is 0x68)
MPU_ADDR = 0x68

# Register addresses for accelerometer and gyroscope data
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

# Gryroscope bias
BIAS_X = -1440.115
BIAS_Y =  226.0664
BIAS_Z = -1314.369

# Default is ±250 deg/s range, which gives 131 LSB/(deg/s)
GYRO_SCALE_FACTOR = 131.0

# Time constant for complementary filter (in seconds)
FILTER_TIME_CONSTANT = 10


i2c = I2C(1, sda=Pin(26), scl=Pin(27))  # Initialize I2C with SDA and SCL pins

# Function to initialize the MPU-6050
def mpu_init():
    # Wake up the MPU6050 (it is in sleep mode by default after power-up)
    i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')  # Write to PWR_MGMT_1 register (wake up the device)

# Function to read 16-bit data (high byte + low byte)
def read_word(register):
    high = i2c.readfrom_mem(MPU_ADDR, register, 1)
    low = i2c.readfrom_mem(MPU_ADDR, register + 1, 1)
    value = (high[0] << 8) + low[0]
    # Convert to signed 16-bit if the value is negative
    if value >= 0x8000:
        value -= 0x10000
    return value

# Function to get the accelerometer values
def read_accel():
    ax = read_word(ACCEL_XOUT_H)
    ay = read_word(ACCEL_YOUT_H)
    az = read_word(ACCEL_ZOUT_H)
    return ax, ay, az

# Function to get the gyroscope values
def read_gyro():
    gx = read_word(GYRO_XOUT_H) - BIAS_X
    gy = read_word(GYRO_YOUT_H) - BIAS_Y
    gz = read_word(GYRO_ZOUT_H) - BIAS_Z
    return gx, gy, gz


# Function to calculate angles from accelerometer data
def calc_accel_angles():
    ax, ay, az = read_accel()
    # Convert to g forces (assuming ±2g range, which is 16384 LSB/g)
    ax_g = ax / 16384.0
    ay_g = ay / 16384.0
    az_g = az / 16384.0
    
    # Calculate angles using arctangent
    # These formulas calculate roll and pitch angles
    accel_angle_x = math.atan2(ay_g, math.sqrt(ax_g**2 + az_g**2)) * 180.0 / math.pi  # Roll
    accel_angle_y = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi  # Pitch
    
    return accel_angle_x, accel_angle_y

prev_time = 0
angle_x, angle_y, angle_z = 0, 0, 0

def angle_from_gyro():
    global prev_time, angle_x, angle_y, angle_z
    
    # Get current time in milliseconds
    curr_time = time.ticks_ms()
    
    # Calculate time difference in seconds
    if prev_time == 0:
        dt = 0
    else:
        dt = time.ticks_diff(curr_time, prev_time) / 1000.0
    
    prev_time = curr_time
    
    # Skip integration on first call
    if dt == 0:
        return 0, 0, 0
    
    # Read gyroscope data (angular velocity)
    gx, gy, gz = read_gyro()
    
    # Convert raw gyro values to degrees per second
    gx_dps = gx / GYRO_SCALE_FACTOR
    gy_dps = gy / GYRO_SCALE_FACTOR
    gz_dps = gz / GYRO_SCALE_FACTOR
    
    # Calculate gyro angle change
    gyro_angle_x = gx_dps * dt
    gyro_angle_y = gy_dps * dt
    gyro_angle_z = gz_dps * dt
    filter_coef = FILTER_TIME_CONSTANT / (FILTER_TIME_CONSTANT + dt)
    
    # Update angles with the filtered values
    angle_x = filter_coef * (angle_x + gyro_angle_x) - (1 - filter_coef) * angle_x
    angle_y = filter_coef * (angle_y + gyro_angle_y) - (1 - filter_coef) * angle_y
    angle_z = filter_coef * (angle_z + gyro_angle_z) - (1 - filter_coef) * angle_z
    
    return angle_x, angle_y, angle_z



# Wheel Encoder Implementation

WHEEL_ENCODER_PIN = 28     # GPIO pin connected to the encoder's output 
PULSES_PER_REV = 20          # Set this to the number of pulses per wheel revolution
WHEEL_DIAMETER = 0.067      # Wheel diameter in meters (adjust as needed)

wheel_pulse_count = 0

# Function to initialize the wheel encoder by setting up its GPIO pin and attaching an interrupt
def wheel_encoder_init():
    global wheel_encoder
    # Configure the encoder pin as input with internal pull-up resistor (adjust if your hardware needs otherwise)
    wheel_encoder = Pin(WHEEL_ENCODER_PIN, Pin.IN, Pin.PULL_UP)
    # Attach an interrupt on the rising edge; adjust the trigger edge if needed
    wheel_encoder.irq(trigger=Pin.IRQ_RISING, handler=wheel_encoder_irq_handler)

# Interrupt handler for the wheel encoder pulses
def wheel_encoder_irq_handler(pin):
    global wheel_pulse_count
    wheel_pulse_count += 1

# Function to calculate the wheel RPM based on the pulse count measured over a given time interval (in ms)
def get_wheel_rpm(interval_ms=1000):
    global wheel_pulse_count
    # Copy the pulse count and then reset it
    count = wheel_pulse_count
    wheel_pulse_count = 0
    # Calculate RPM: (pulses / pulses per rev) gives revolutions per interval,
    # then multiply by (60000 / interval_ms) to convert to revolutions per minute.
    rpm = (count / PULSES_PER_REV) * (60000 / interval_ms)
    return rpm

def get_speed(rpm_mean, direction):
    wheel_circumference = math.pi * WHEEL_DIAMETER
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

def get_absolute_acceleration(rpms, times, direction):
    wheel_circumference = math.pi * WHEEL_DIAMETER
    
    # Calculate the slope (acceleration) using linear regression
    n = len(rpms)
    sum_x = sum(times)
    sum_y = sum(rpms)
    sum_xy = sum(x * y for x, y in zip(times, rpms))
    sum_x_squared = sum(x ** 2 for x in times)
    slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x_squared - sum_x ** 2)
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





# Main program loop
if __name__ == "__main__":
    wheel_encoder_init()
    
    while True:
        # read data from wheel encoder
        wheel_rpm_list = []
        time_list = []
        for i in range(50):
            wheel_rpm = get_wheel_rpm()
            wheel_speed = get_speed(wheel_rpm, 1)
            wheel_rpm_list.append(wheel_rpm)
            time_list.append(time.ticks_ms())
            time.sleep_ms(100)
        
        print(wheel_rpm_list)
        print(time_list)
        print(f"rpm length: {len(wheel_rpm_list)}, time length: {len(time_list)}")
        wheel_acceleration = get_absolute_acceleration(wheel_rpm_list, time_list, 1)
        print(f"Wheel RPM: {wheel_rpm:.2f}, Speed: {wheel_speed:.2f} m/s, Acceleration: {wheel_acceleration} m/s²")