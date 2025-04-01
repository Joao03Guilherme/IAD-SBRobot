"""
Simple direction finder for MPU6050 on Raspberry Pi Pico
Helps identify which axis corresponds to which movement
"""

import time
from machine import Pin, I2C
from raspberry.controllers.gyroscope_controller import MPU6050


def test_directions(mpu):
    """Perform ordered tests to identify sensor axes."""
    print("MPU6050 Direction Test")
    print("Follow the instructions and watch the values")

    def test_movement(duration, label):
        print(f"\nTesting: {label}")
        print("X     Y     Z")
        start = time.time()
        while (time.time() - start) < duration:
            ax, ay, az = mpu.read_accel()
            gx, gy, gz = mpu.read_gyro()
            print(
                f"{ax/16384:4.2f} {ay/16384:4.2f} {az/16384:4.2f} | {gx/131:4.0f} {gy/131:4.0f} {gz/131:4.0f}"
            )
            time.sleep(0.1)

    input = lambda x: print(x + " (wait 3s)")  # MicroPython doesn't have input()

    input("Press reset to begin the test...")
    time.sleep(3)

    movements = [
        ("Tilt Forward", 3),
        ("Tilt Backward", 3),
        ("Tilt Left", 3),
        ("Tilt Right", 3),
        ("Rotate Clockwise", 3),
        ("Rotate CounterClockwise", 3),
    ]

    for label, duration in movements:
        input(f"Now: {label}")
        test_movement(duration, label)
        print("\nReset position and wait...")
        time.sleep(2)

    print("\nTest complete!")
    print("Review the values to determine which axis corresponds to each movement")


def main():
    # Initialize I2C and MPU6050
    i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
    mpu = MPU6050(i2c)
    time.sleep(1)  # Let sensor stabilize

    try:
        test_directions(mpu)
    except KeyboardInterrupt:
        print("\nTest interrupted.")


if __name__ == "__main__":
    main()
