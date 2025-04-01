import time

from raspberry.controllers.motor_controller import MotorController
from raspberry.stash.bluethooth_raspberry import BLEReceiver

if __name__ == "__main__":
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
    motor_controller = MotorController(MOTOR_CONFIG)
    motorA = motor_controller.motor_a
    motorB = motor_controller.motor_b

    def handle_command(cmd):
        print(f"Processing command: {cmd}")

        if cmd == "RUN":
            print("Running motors...")
            motorA(50)
            motorB(50)

        if cmd == "STOP":
            motorA(0)
            motorB(0)

    ble = BLEReceiver()

    ble.set_command_callback(handle_command)
    print("BLE Receiver is running...")

    # Keep the script running
    while True:
        time.sleep(1)
        # Send telemetry data every second
        telemetry_data = {"status": "running", "battery": 95}
        ble.send_telemetry(telemetry_data)
        time.sleep(1)
