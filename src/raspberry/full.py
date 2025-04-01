from machine import Pin, I2C
import time

from raspberry.controllers.motor_controller import MotorController
from raspberry.controllers.balance_controller import BalanceController
from raspberry.controllers.gyroscope_controller import MPU6050

# ======== CONFIGURATION ========
# PID Constants
PID_CONFIG = {
    "Kp": 40,
    "Ki": 40,
    "Kd": 0.05,
    "sample_time": 0.005,  # 5ms
    "target_angle": -2.5,
}

# Motor pin configuration
MOTOR_CONFIG = {
    "IN1": 2,
    "IN2": 3,
    "IN3": 4,
    "IN4": 5,
    "ENA": 6,
    "ENB": 7,
    "PWM_FREQ": 1000,
}

# MPU6050 configuration
MPU_CONFIG = {"i2c_id": 1, "sda_pin": 26, "scl_pin": 27, "address": 0x68}


# ======== MAIN PROGRAM ========
def main():
    print("Initializing self-balancing robot...")

    # Initialize I2C for MPU6050
    i2c = I2C(
        MPU_CONFIG["i2c_id"],
        sda=Pin(MPU_CONFIG["sda_pin"]),
        scl=Pin(MPU_CONFIG["scl_pin"]),
    )

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
