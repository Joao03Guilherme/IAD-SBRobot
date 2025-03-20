from machine import Pin, I2C
import time

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

i2c = I2C(1, sda=Pin(26), scl=Pin(27))  # Initialize I2C with SDA and SCL pins

power_pin = Pin(21, Pin.OUT)
power_pin.value(1)  # Set power pin high to turn on the MPU6050

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
    gx = read_word(GYRO_XOUT_H)
    gy = read_word(GYRO_YOUT_H)
    gz = read_word(GYRO_ZOUT_H)
    return gx, gy, gz

# Main program loop
if __name__ == "__main__":
    mpu_init()
    
    while True:
        # Read accelerometer and gyroscope values
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        
        # Print the raw sensor data
        print(f"Accelerometer: ax={ax}, ay={ay}, az={az}")
        print(f"Gyroscope: gx={gx}, gy={gy}, gz={gz}")
        
        time.sleep(1)  # Sleep for 1 second before the next reading
