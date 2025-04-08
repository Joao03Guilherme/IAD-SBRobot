import math
import time
from machine import I2C, Pin
from parameters.parameters import data, change_gyro_bias, change_accel_bias


class MPU6050:
    # MPU6050 I2C Address
    MPU_ADDR = data["MPU_CONFIG"]["address"]

    # Register addresses
    ACCEL_XOUT_H = data["MPU_CONFIG"]["accel_xout_h"]
    ACCEL_XOUT_L = data["MPU_CONFIG"]["accel_xout_l"]
    ACCEL_YOUT_H = data["MPU_CONFIG"]["accel_yout_h"]
    ACCEL_YOUT_L = data["MPU_CONFIG"]["accel_yout_l"]
    ACCEL_ZOUT_H = data["MPU_CONFIG"]["accel_zout_h"]
    ACCEL_ZOUT_L = data["MPU_CONFIG"]["accel_zout_l"]

    GYRO_XOUT_H = data["MPU_CONFIG"]["gyro_xout_h"]
    GYRO_XOUT_L = data["MPU_CONFIG"]["gyro_xout_l"]
    GYRO_YOUT_H = data["MPU_CONFIG"]["gyro_yout_h"]
    GYRO_YOUT_L = data["MPU_CONFIG"]["gyro_yout_l"]
    GYRO_ZOUT_H = data["MPU_CONFIG"]["gyro_zout_h"]
    GYRO_ZOUT_L = data["MPU_CONFIG"]["gyro_zout_l"]

    # Accelerometer bias
    BIAS_AX = data["MPU_CONFIG"]["bias_ax"]
    BIAS_AY = data["MPU_CONFIG"]["bias_ay"]

    # Gyroscope bias
    BIAS_X = data["MPU_CONFIG"]["bias_x"]
    BIAS_Y = data["MPU_CONFIG"]["bias_y"]
    BIAS_Z = data["MPU_CONFIG"]["bias_z"]

    # Default is ±250 deg/s range, which gives 131 LSB/(deg/s)
    GYRO_SCALE_FACTOR = data["MPU_CONFIG"]["gyro_scale_factor"]

    # Time constant for complementary filter (in seconds)
    alpha = data["MPU_CONFIG"]["alpha"]

    def __init__(self, sda_pin=26, scl_pin=27):
        # Initialize I2C with SDA and SCL pins
        self.i2c = I2C(1, sda=Pin(sda_pin), scl=Pin(scl_pin))

        # Initialize angle tracking values
        self.prev_time = 0
        self.angle_gyro_x, self.angle_gyro_y, self.angle_gyro_z = 0, 0, 0
        self.angle_accel_x, self.angle_accel_y, self.angle_accel_z = 0, 0, 0
        self.combined_angle_x, self.combined_angle_y = 0, 0 

        # Initialization now explicitly named
        self.initialize_mpu()

    def initialize_mpu(self):
        # Wake up the MPU6050 (it is in sleep mode by default after power-up)
        try:
            self.i2c.writeto_mem(
                self.MPU_ADDR, 0x6B, b"\x00"
            )  # Write to PWR_MGMT_1 register
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

        self.angle_gyro_x += gyro_angle_x
        self.angle_gyro_y += gyro_angle_y
        self.angle_gyro_z += gyro_angle_z
        return self.angle_gyro_x, self.angle_gyro_y, self.angle_gyro_z
    
    def combined_angle(self):
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
            return 0, 0

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

        # Read accelerometer angles
        accel_angle_x, accel_angle_y = self.calc_accel_angles()
        
        self.angle_gyro_x += gyro_angle_x
        self.angle_gyro_y += gyro_angle_y
        self.angle_gyro_z += gyro_angle_z
        
        # Complementary filter to combine gyro and accelerometer angles
        self.combined_angle_x = (
            self.alpha * (self.angle_gyro_x) + (1 - self.alpha) * accel_angle_x
        )
        self.combined_angle_y = (
            self.alpha * (self.angle_gyro_y) + (1 - self.alpha) * accel_angle_y
        )

        print(
            f"Combined Angle X: {self.combined_angle_x}, Combined Angle Y: {self.combined_angle_y}"
        )

        return (
            self.combined_angle_x,
            self.combined_angle_y
        )
    
    def calibrate_accel(self, num_samples=1000):
        # Calibrate the accelerometer using 1000000 samples
        bias_x, bias_y= 0, 0
        for _ in range(num_samples):
            ax, ay, az = self.read_accel()
            bias_x += ax
            bias_y += ay
        bias_x /= num_samples
        bias_y /= num_samples

        print("Bias X:", bias_x)
        print("Bias Y:", bias_y)

        set_bias_ax = self.BIAS_AX + bias_x
        set_bias_ay = self.BIAS_AY + bias_y

        self.BIAS_AX = set_bias_ax
        self.BIAS_AY = set_bias_ay

        change_accel_bias(set_bias_ax, set_bias_ay)
        print("Bias values updated in config.json")

    def calibrate_gyro(self, num_samples=1000):
        # Calibrate the gyroscope using 1000000 samples
        bias_x, bias_y, bias_z = 0, 0, 0
        for _ in range(num_samples):
            gx, gy, gz = self.read_gyro()
            bias_x += gx
            bias_y += gy
            bias_z += gz
        bias_x /= num_samples
        bias_y /= num_samples
        bias_z /= num_samples

        print("Bias X:", bias_x)
        print("Bias Y:", bias_y)
        print("Bias Z:", bias_z)

        set_bias_x = self.BIAS_X + bias_x
        set_bias_y = self.BIAS_Y + bias_y
        set_bias_z = self.BIAS_Z + bias_z

        self.BIAS_X = set_bias_x
        self.BIAS_Y = set_bias_y
        self.BIAS_Z = set_bias_z

        change_gyro_bias(set_bias_x, set_bias_y, set_bias_z)
        print("Bias values updated in config.json")