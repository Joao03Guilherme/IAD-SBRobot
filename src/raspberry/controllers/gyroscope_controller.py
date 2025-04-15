from machine import I2C, Pin
import time
import math
import ustruct
import uasyncio as asyncio
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
    BIAS_AZ = data["MPU_CONFIG"]["bias_az"]

    # Gyroscope bias
    BIAS_GX = data["MPU_CONFIG"]["bias_gx"]
    BIAS_GY = data["MPU_CONFIG"]["bias_gy"]
    BIAS_GZ = data["MPU_CONFIG"]["bias_gz"]

    # Default is ±250 deg/s range, which gives 131 LSB/(deg/s)
    GYRO_SCALE_FACTOR = data["MPU_CONFIG"]["gyro_scale_factor"]

    # Time constant for complementary filter (in seconds)
    alpha = data["MPU_CONFIG"]["alpha"]

    sample_time = data["PID_CONFIG"]["sample_time"]  # Sample time in seconds

    def __init__(self, sda_pin=26, scl_pin=27):
        self.i2c = I2C(1, sda=Pin(sda_pin), scl=Pin(scl_pin))

        # Additional fixed offset to account for any sensor-to-robot misalignment.
        self.angle_offset = 0
        self._current_pitch = 0

        # Wake up the MPU6050 (exit sleep mode)
        self.i2c.writeto_mem(self.MPU_ADDR, 0x6B, b"\x00")
        time.sleep(0.1)

    def read_raw_data(self, reg_addr):
        # Read two bytes from the specified register and convert them into a signed 16-bit integer.
        data = self.i2c.readfrom_mem(self.MPU_ADDR, reg_addr, 2)
        value = ustruct.unpack(">h", data)[0]
        return value

    def read_accel(self):
        # Read the raw accelerometer data.
        ax = self.read_raw_data(0x3B)
        ay = self.read_raw_data(0x3D)
        az = self.read_raw_data(0x3F)
        # Correct the reading by subtracting the bias then converting to g's (16384 LSB/g for ±2g range).
        ax = (ax - self.BIAS_AX) / 16384.0
        ay = (ay - self.BIAS_AY) / 16384.0
        az = (az - self.BIAS_AZ) / 16384.0
        return ax, ay, az

    def read_gyro(self):
        # Read the raw gyroscope data.
        gx = self.read_raw_data(0x43)
        gy = self.read_raw_data(0x45)
        gz = self.read_raw_data(0x47)
        # Correct the reading by subtracting the bias then converting to degrees per second (131 LSB/dps for ±250°/s).
        gx = (gx - self.BIAS_GX) / 131.0
        gy = (gy - self.BIAS_GY) / 131.0
        gz = (gz - self.BIAS_GZ) / 131.0
        return gx, gy, gz

    def calibrate_mpu(self, num_samples=1000, delay=0.005):
        """
        Calibrate sensor biases.

        IMPORTANT:
         - Place the robot in a known, level position.
         - Expected raw readings for level configuration (±2g): X ~0, Y ~0, Z ~+16384.

        The calculated biases are used to ensure that, after correction, the Z axis shows about
        1g (and X & Y near 0) when the robot is level.
        """
        print("Calibrating sensor biases.")
        print("Place the robot in a known, level position and keep it still.")
        ax_sum, ay_sum, az_sum = 0, 0, 0
        gx_sum, gy_sum, gz_sum = 0, 0, 0

        for _ in range(num_samples):
            ax_sum += self.read_raw_data(0x3B)
            ay_sum += self.read_raw_data(0x3D)
            az_sum += self.read_raw_data(0x3F)
            gx_sum += self.read_raw_data(0x43)
            gy_sum += self.read_raw_data(0x45)
            gz_sum += self.read_raw_data(0x47)
            time.sleep(delay)

        avg_ax = ax_sum / num_samples
        avg_ay = ay_sum / num_samples
        avg_az = az_sum / num_samples

        # For level position, the expected raw measurement is:
        #   X: 0, Y: 0, Z: 16384 (1g)
        self.BIAS_AX = avg_ax
        self.BIAS_AY = avg_ay
        self.BIAS_AZ = avg_az - 16384
        change_accel_bias(self.BIAS_AX, self.BIAS_AY, self.BIAS_AZ)

        # Gyroscope bias: computed as average (should be nearly zero if there is no rotation)
        self.BIAS_GX = gx_sum / num_samples
        self.BIAS_GY = gy_sum / num_samples
        self.BIAS_GZ = gz_sum / num_samples
        change_gyro_bias(self.BIAS_GX, self.BIAS_GY, self.BIAS_GZ)
        print(
            "Accelerometer biases (raw units):",
            self.BIAS_AX,
            self.BIAS_AY,
            self.BIAS_AZ,
        )
        print("Gyroscope biases (raw units):", self.BIAS_GX, self.BIAS_GY, self.BIAS_GZ)

        self.calibrate_inclination_offset()
        print("Calibration complete.")
        self.start_update_task()

    def calibrate_inclination_offset(self):
        """
        Calibrate a fixed inclination offset.
        With the accelerometer calibrated, when the robot is level the computed pitch (inclination)
        should be 0°. Any systematic difference is stored and subtracted from future measurements.
        """
        print("Calibrating inclination offset. Ensure the robot is level.")
        time.sleep(2)
        ax, ay, az = self.read_accel()
        # Compute pitch (using the X axis as the forward direction)
        acc_angle = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        self.angle_offset = acc_angle
        print("Angle offset determined at: {:.2f}°".format(self.angle_offset))

    def complementary_filter(self, dt, prev_angle):
        """
        Apply a complementary filter to combine:
        - Gyro integrated angle (smooth but drifting).
        - Accelerometer angle (noisy but stable with proper calibration).
        """
        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()

        # Compute angle from accelerometer (pitch using X axis).
        acc_angle = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
        # Adjust by the fixed offset determined during calibration.
        acc_angle -= self.angle_offset

        # Integrate gyroscope reading for angle update.
        # Here we assume the gyroscope's Y axis is aligned with the pitch rotation.
        gyro_angle = prev_angle + gx * dt

        # Complementary filter blending.
        alpha = 0.95  # You may adjust this parameter as needed.
        filtered_angle = alpha * gyro_angle + (1 - alpha) * acc_angle

        return filtered_angle

    # Global variable to hold the current pitch angle.
    _current_pitch = 0

    def start_update_task(self, update_interval=0):
        if update_interval <= 0:
            update_interval = self.sample_time
        # Only create the task if it has not been created already.
        if not hasattr(self, "_update_task") or self._update_task.done():
            self._update_task = asyncio.create_task(self.update_pitch(update_interval))

    async def update_pitch(self, update_interval=0):
        """
        Asynchronous task that continuously updates the pitch angle.

        This function should run in a background task.
        """
        if update_interval <= 0:
            update_interval = self.sample_time

        last_time = time.ticks_ms()
        while True:
            current_time = time.ticks_ms()
            dt = time.ticks_diff(current_time, last_time) / 1000.0
            last_time = current_time

            self._current_pitch = self.complementary_filter(dt, self._current_pitch)
            # Optionally, you can log or process the updated pitch here.
            await asyncio.sleep(update_interval)

    def get_current_angle(self):
        """
        Returns the most recent pitch (inclination) angle in degrees.

        This function can be called from other asynchronous tasks.
        """
        return self._current_pitch


if __name__ == "__main__":

    async def main():
        # Initialize I2C; adjust I2C port and pin assignments as needed.
        mpu = MPU6050(
            sda_pin=data["MPU_CONFIG"]["sda_pin"], scl_pin=data["MPU_CONFIG"]["scl_pin"]
        )

        # Perform sensor calibration; ensure the robot is in the proper level configuration.
        mpu.calibrate_mpu(num_samples=500)

        # Start the background task that updates the pitch continuously.
        asyncio.create_task(mpu.update_pitch())

        # Here is an example loop that prints the current pitch every second.
        # In your application, you might use get_current_angle() as needed in other tasks.
        while True:
            angle = mpu.get_current_angle()
            print("Current Pitch (inclination): {:.2f}°".format(angle))
            await asyncio.sleep(0.5)

    # To run the asynchronous tasks:
    asyncio.run(main())
