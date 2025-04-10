from machine import Pin, I2C
import asyncio

from controllers.gyroscope_controller_old import MPU6050
from controllers.encoder_controller import WheelEncoder


# Example usage with async
async def main():
    # Initialize the MPU6050
    mpu = MPU6050()
    dt = 0.0001

    anglesx = []
    anglesy = []
    anglesz = []

    for i in range(1000):
        angle_x, angle_y, angle_z = mpu.read_gyro()
        anglesx.append(angle_x)
        anglesy.append(angle_y)
        anglesz.append(angle_z)

        # Sleep for a short duration to simulate real-time data acquisition
        await asyncio.sleep(dt)

    # Print the average angles
    avg_angle_x = sum(anglesx) / len(anglesx)
    avg_angle_y = sum(anglesy) / len(anglesy)
    avg_angle_z = sum(anglesz) / len(anglesz)

    print(f"Average Angle X: {avg_angle_x} degrees")
    print(f"Average Angle Y: {avg_angle_y} degrees")
    print(f"Average Angle Z: {avg_angle_z} degrees")


# Run the async program
if __name__ == "__main__":
    asyncio.run(main())
