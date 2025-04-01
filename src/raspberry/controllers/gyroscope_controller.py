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
        self.i2c.writeto_mem(self.addr, 0x6B, b"\x00")

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
