"""
SBRobot - Full Prototype Implementation
Self-balancing robot main control program
"""

import time
from machine import Pin, I2C

import asyncio
from controllers.motor_controller import MotorController
from bluethooth.BLEReceiver import BLEReceiver
from drive import Driving

# Configuration
MOTOR_CONFIG = {
    "IN1": 2,
    "IN2": 3,
    "IN3": 4,
    "IN4": 5,
    "ENA": 6,
    "ENB": 7,
    "PWM_FREQ": 1000,
}

MPU_CONFIG = {"i2c_id": 1, "sda_pin": 26, "scl_pin": 27, "address": 0x68}

# Status LED
status_led = Pin("LED", Pin.OUT)

# Define the commands and their descriptions
COMMANDS = {
    "1": {"action": "START", "description": "Start balancing"},
    "2": {"action": "STOP", "description": "Stop all motors"},
    "3": {"action": "DRIVE", "description": "Drive with speed & turn (e.g., 3 20 5)"},
    "4": {"action": "TURN", "description": "Turn by angle (e.g., 4 90 1)"},
    "5": {"action": "CONFIG", "description": "Update configuration"},
    "6": {"action": "CALIBRATE", "description": "Start calibration procedure"},
    "7": {"action": "HELP", "description": "Show this help"},
    "8": {"action": "STATUS", "description": "Show current robot status"},
    "9": {"action": "RESET", "description": "Reset the robot"},
}


class SelfBalancingRobot:
    """Main robot control class."""

    def __init__(self):
        print("Initializing Self-Balancing Robot...")

        # Create motor controller first
        self.motor_controller = MotorController(MOTOR_CONFIG)

        # Use the Driving class from drive.py
        self.driver = Driving(self.motor_controller)

        # Wrapper for async callback
        def command_callback(cmd):
            asyncio.create_task(self.handle_command(cmd))
        
        self.ble = BLEReceiver()
        self.ble.set_command_callback(command_callback)

        # Robot state
        self.running = False
        self.speed = 0
        self.turn = 0

        print("Robot initialized successfully!")
        self.send_help_message()

    async def ble_listener(self):
        """Listen for BLE commands and handle them."""
        while True:
            if self.ble.connected:
                await asyncio.sleep(0.1)  # Adjust the sleep time as needed

    async def start(self):
        """Start the balancing loop."""
        self.running = True
        await asyncio.gather(self.main_loop(), self.ble_listener())

    async def stop(self):
        """Stop the robot."""
        self.running = False
        self.driver.stop()

    def send_help_message(self):
        """Send help information with available commands."""
        help_text = "Available commands:\n"
        for cmd_num, cmd_info in COMMANDS.items():
            help_text += (
                f"{cmd_num}: {cmd_info['action']} - {cmd_info['description']}\n"
            )

        help_text += "\nExamples:\n"
        help_text += "1 - Start balancing\n"
        help_text += "2 - Stop motors\n"
        help_text += "3 20 5 - Drive forward at speed 20, turn bias 5\n"
        help_text += "4 45 1 - Turn 45 degrees to the right\n"
        help_text += "7 - Show this help\n"

        print(help_text)
        if self.ble.connected:
            # Use send_telemetry instead of send_message
            self.ble.send_telemetry(help_text)

    def show_status(self):
        """Show current robot status."""
        status = {
            "running": self.running,
            "speed": self.speed,
            "turn": self.turn,
            "angle": round(
                self.driver.angle_data[-1] if self.driver.angle_data else 0, 2
            ),
            "balance_target": self.driver.balance_target,
            "max_safe_tilt": self.driver.max_safe_tilt,
        }

        status_text = "Robot Status:\n"
        for key, value in status.items():
            status_text += f"{key}: {value}\n"

        print(status_text)
        if self.ble.connected:
            # Use send_telemetry instead of send_message
            self.ble.send_telemetry(status_text)

    async def handle_command(self, cmd):
        """Process BLE commands."""
        # Make sure cmd is a string
        if isinstance(cmd, bytes):
            cmd = cmd.decode("utf-8")

        print(f"Command received [CALLBACK]: {cmd}")

        # Strip whitespace and handle empty commands
        cmd = cmd.strip()
        if not cmd:
            print("Empty command received")
            return

        # Handle numeric commands (like "1" or "2")
        parts = cmd.split()
        cmd_num = parts[0]

        if cmd_num in COMMANDS:
            action = COMMANDS[cmd_num]["action"]
            print(f"Executing {action} command")

            if action == "START":
                print("Starting robot...")
                await self.start()
                return

            elif action == "STOP":
                print("Stopping robot...")
                await self.stop()
                return

            elif action == "DRIVE":
                if len(parts) >= 2:  # turn bias is optional
                    try:
                        self.speed = int(parts[1])
                        self.turn = (
                            int(parts[2]) if len(parts) > 2 else 0
                        )  # Make turn optional
                        print(f"Setting drive: speed={self.speed}, turn={self.turn}")

                        # Start the robot if it's not already running
                        if not self.running:
                            print("Auto-starting robot for drive command...")
                            self.start()
                        return
                    except ValueError:
                        print("Invalid speed or turn value")
                else:
                    print("Error: DRIVE command requires at least a speed parameter")
                    print("Usage: 3 <speed> [turn]")
                return

            elif action == "TURN":
                if len(parts) >= 3:
                    try:
                        angle = int(parts[1])
                        direction = int(parts[2])
                        print(f"Turning {angle} degrees in direction {direction}")
                        self.driver.turn(angle, direction)
                        return
                    except ValueError:
                        print("Invalid angle or direction value")
                else:
                    print("Error: TURN command requires angle and direction parameters")
                    print("Usage: 4 <angle> <direction>")
                return

            elif action == "CONFIG":
                if len(parts) >= 3:
                    param = parts[1].upper()
                    try:
                        value = float(parts[2])
                        self._update_config(param, value)
                        return
                    except ValueError:
                        print("Invalid configuration value")
                else:
                    print("Error: CONFIG command requires parameter and value")
                    print("Usage: 5 <parameter> <value>")
                return

            elif action == "CALIBRATE":
                print("Starting calibration...")
                self.angle_sensor.calibrate()
                return

            elif action == "HELP":
                print("Showing help...")
                self.send_help_message()
                return

            elif action == "STATUS":
                print("Showing status...")
                self.show_status()
                return

        # If we get here, it's an unrecognized command
        print(f"Unknown command: {cmd}")
        self.send_help_message()  # Show available commands

    def _update_config(self, param, value):
        """Update robot configuration parameters."""
        param = param.upper()
        try:
            if param == "TARGET":
                self.driver.balance_target = value
            elif param == "KP":
                self.driver.balance_kp = value
            elif param == "KI":
                self.driver.balance_ki = value
            elif param == "KD":
                self.driver.balance_kd = value
            elif param == "SAMPLE":
                self.driver.sample_time = value
            elif param == "MAXTILT":
                self.driver.max_safe_tilt = value
            else:
                print(f"Unknown parameter: {param}")
                return
            print(f"Updated {param} to {value}")
        except Exception as e:
            print(f"Error updating config: {e}")

    def send_telemetry(self):
        """Send robot telemetry data via BLE."""
        if not self.ble.connected:
            return

        # Get the latest sensor data
        angle = 0
        if len(self.driver.angle_data) > 0:
            angle = self.driver.angle_data[-1]

        # Create telemetry string (simpler than JSON)
        telemetry = (
            f"A:{angle:.2f},S:{self.speed},T:{self.turn},R:{1 if self.running else 0}"
        )
        if self.driver.left_power_data and self.driver.right_power_data:
            telemetry += f",L:{self.driver.left_power_data[-1]},R:{self.driver.right_power_data[-1]}"

        # Send telemetry
        self.ble.send_telemetry(telemetry)

    def find_balance_point(self):
        """Utility function to find the natural balance point of the robot.
        This helps determine the optimal balance_target value."""

        print("Finding balance point. Place the robot upright and keep it still...")

        samples = []
        duration = 5  # seconds
        start_time = time.time()

        # Collect angle data for several seconds
        while time.time() - start_time < duration:
            angle = self.driver.angle_data[-1] if self.driver.angle_data else 0
            samples.append(angle)
            time.sleep(0.05)
            status_led.toggle()  # Blink during measurement

        # Calculate average angle
        if samples:
            avg_angle = sum(samples) / len(samples)
            print(f"Balance point found at approximately {avg_angle:.2f} degrees")
            return avg_angle
        else:
            print("Error: No samples collected")
            return None

    async def main_loop(self):
        """Main control loop."""
        last_telemetry_time = 0
        self.speed = 0
        self.turn = 0
        self.running = True
        
        print(
            f"Entering main loop with speed={self.speed}, turn={self.turn}, running={self.running}"
        )

        try:
            while self.running:
                # Update driving with current speed and turn
                print(f"Calling driver.forward({self.speed}, {self.turn})")
                angle, left, right, speed, acceleration = self.driver.forward(target_speed=self.speed, turn_bias=self.turn)
                print(
                    f"forward() result: angle={angle:.2f}, left={left}, right={right}"
                )

                # Safety check
                if abs(angle) > self.driver.max_safe_tilt:
                    print(f"Tilt too high ({angle:.2f}°). Stopping.")
                    self.running = False
                    self.driver.stop()
                    break

                # Send telemetry at 10Hz
                current_time = time.time()
                if current_time - last_telemetry_time >= 0.1:
                    self.send_telemetry()
                    last_telemetry_time = current_time
                    status_led.toggle()  # Blink to show activity

                await asyncio.sleep(self.driver.sample_time)

        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            print("Exiting main loop, stopping motors")
            self.driver.stop()
            print("Robot stopped")


def calibration_mode(robot):
    """Interactive mode to calibrate the robot."""
    print("==== Calibration Mode ====")
    print("1. Find balance point")
    print("2. Test motors")
    print("3. Exit calibration")

    choice = input("Enter choice (1-3): ")

    if choice == "1":
        balance_point = robot.find_balance_point()
        if balance_point is not None:
            print(f"Set balance_target to {balance_point:.2f} in your configuration")

    elif choice == "2":
        print("Testing motors...")
        motors = robot.motor_controller

        try:
            # Test left motor
            print("Testing left motor forward")
            motors.motor_a(30)
            time.sleep(1)
            motors.motor_a(0)
            time.sleep(0.5)

            print("Testing left motor backward")
            motors.motor_a(-30)
            time.sleep(1)
            motors.motor_a(0)
            time.sleep(0.5)

            # Test right motor
            print("Testing right motor forward")
            motors.motor_b(30)
            time.sleep(1)
            motors.motor_b(0)
            time.sleep(0.5)

            print("Testing right motor backward")
            motors.motor_b(-30)
            time.sleep(1)
            motors.motor_b(0)

        finally:
            motors.stop()
            print("Motor test complete")

    elif choice == "3":
        return

    else:
        print("Invalid choice")

async def main():
    """Program entry point."""
    robot = SelfBalancingRobot()

    print("Robot ready. Send commands via BLE:")
    print("1: Start balancing")
    print("2: Stop")
    print("3 <speed> [turn]: Set drive parameters (speed, optional turn)")
    print("4 <angle> <direction>: Turn by specified angle (1=right, -1=left)")
    print("7: Show help")
    
    # Start the BLE listener immediately
    ble_task = asyncio.create_task(robot.ble_listener())
    
    # Run telemetry task in the background
    last_time = time.time()   
    try:
        while True:
            if time.time() - last_time >= 1:
                robot.send_telemetry()
                last_time = time.time()
            
            # Process any pending BLE commands
            await asyncio.sleep(0.1)  # Prevent CPU overload

    except KeyboardInterrupt:
        print("\nProgram terminated")
    finally:
        robot.stop()
        # Cancel the BLE listener task
        ble_task.cancel()
        try:
            await ble_task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    asyncio.run(main())
