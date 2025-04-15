import math
import utime
from machine import I2C, Pin


class MPU6050:
    def __init__(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address
        self._init_mpu()

    def _init_mpu(self):
        # Wake up MPU6050
        self.i2c.writeto_mem(self.address, 0x6B, b"\x00")
        # Set accelerometer range to ±8g
        self.i2c.writeto_mem(self.address, 0x1C, b"\x10")
        # Set gyro range to ±500°/s
        self.i2c.writeto_mem(self.address, 0x1B, b"\x08")
        # Set DLPF to 94Hz bandwidth, 3ms delay
        self.i2c.writeto_mem(self.address, 0x1A, b"\x03")

    def read_raw_data(self):
        data = self.i2c.readfrom_mem(self.address, 0x3B, 14)
        accel = [
            self._convert(data[0], data[1]),
            self._convert(data[2], data[3]),
            self._convert(data[4], data[5]),
        ]
        temp = self._convert_temp(data[6], data[7])
        gyro = [
            self._convert(data[8], data[9]),
            self._convert(data[10], data[11]),
            self._convert(data[12], data[13]),
        ]
        return accel, gyro, temp

    def _convert(self, high, low):
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value

    def _convert_temp(self, high, low):
        temp_raw = self._convert(high, low)
        return temp_raw / 340.0 + 36.53


class Inclinometer:
    def __init__(self, mpu):
        self.mpu = mpu
        self.accel_bias = [0, 0, 0]
        self.gyro_bias = [0, 0, 0]
        self.alpha = 0.98  # Complementary filter coefficient
        self.dt = 0.01  # Sample time (seconds)
        self.last_time = utime.ticks_ms()
        self.pitch = 0.0
        self.roll = 0.0

    def calibrate_gyro(self, samples=1000):
        print("Calibrating gyro - keep robot stationary")
        gyro_sum = [0, 0, 0]
        for _ in range(samples):
            _, gyro, _ = self.mpu.read_raw_data()
            for i in range(3):
                gyro_sum[i] += gyro[i]
            utime.sleep_ms(1)
        self.gyro_bias = [x / samples for x in gyro_sum]
        print("Gyro bias:", self.gyro_bias)

    def calibrate_accel(self, samples=1000):
        print("Calibrating accelerometer - keep robot stationary")
        accel_sum = [0, 0, 0]
        for _ in range(samples):
            accel, _, _ = self.mpu.read_raw_data()
            for i in range(3):
                accel_sum[i] += accel[i]
            utime.sleep_ms(1)
        self.accel_bias = [x / samples for x in accel_sum]
        print("Accel bias:", self.accel_bias)

    def _get_calibrated_data(self):
        accel_raw, gyro_raw, temp = self.mpu.read_raw_data()

        # Apply calibration offsets
        accel = [
            (accel_raw[0] - self.accel_bias[0]) / 4096.0,
            (accel_raw[1] - self.accel_bias[1]) / 4096.0,
            (accel_raw[2] - self.accel_bias[2]) / 4096.0,
        ]

        gyro = [
            (gyro_raw[0] - self.gyro_bias[0]) / 65.5,
            (gyro_raw[1] - self.gyro_bias[1]) / 65.5,
            (gyro_raw[2] - self.gyro_bias[2]) / 65.5,
        ]

        return accel, gyro

    def update(self):
        accel, gyro = self._get_calibrated_data()

        # Calculate time difference
        current_time = utime.ticks_ms()
        self.dt = (current_time - self.last_time) / 1000.0
        self.last_time = current_time

        # Calculate angles from accelerometer
        accel_pitch = math.degrees(
            math.atan2(accel[1], math.sqrt(accel[0] ** 2 + accel[2] ** 2))
        )
        accel_roll = math.degrees(
            math.atan2(-accel[0], math.sqrt(accel[1] ** 2 + accel[2] ** 2))
        )

        # Integrate gyro measurements
        self.pitch += gyro[0] * self.dt
        self.roll += gyro[1] * self.dt

        # Apply complementary filter
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll

        return self.pitch, self.roll


# Initialize hardware
i2c = I2C(1, scl=Pin(27), sda=Pin(26), freq=400000)
mpu = MPU6050(i2c)
inclinometer = Inclinometer(mpu)

# Perform calibration
inclinometer.calibrate_gyro()
inclinometer.calibrate_accel()

# Main loop
while True:
    pitch, roll = inclinometer.update()
    print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
    utime.sleep_ms(10)
