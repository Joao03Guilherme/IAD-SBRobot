"""
SBRobot - Full Prototype Implementation
Self-balancing robot main control program
"""

import time
from machine import Pin, I2C

import asyncio
from controllers.motor_controller import MotorController
from bluethooth.BLEReceiver import BLEReceiver
from controllers.balance_controller import Driving
from parameters.parameters import (
    data,
    change_kd,
    change_ki,
    change_kp,
    change_sample_time,
    change_max_safe_tilt,
)

# Load constants
MOTOR_CONFIG = data["MOTOR_CONFIG"]
COMMANDS = data["COMMANDS"]

# Status LED
status_led = Pin("LED", Pin.OUT)


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

    async def reset(self):
        """Rest the Driving class."""
        self.stop()
        self.driver = Driving(self.motor_controller)

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

    def _update_config(self, param, value):
        """Update robot configuration parameters."""
        param = param.upper()
        print(f"DEBUG: Starting _update_config with param={param}, value={value}")
        
        try:
            # Convert value to float explicitly
            value = float(value)
            print(f"DEBUG: Converted value to float: {value}")
            print(f"Updating {param} to {value}")

            success = False
            if param == "KP":
                success = change_kp(value)
                self.driver.balance_kp = value
            elif param == "KI":
                success = change_ki(value)
                self.driver.balance_ki = value
            elif param == "KD":
                success = change_kd(value)
                self.driver.balance_kd = value
            elif param == "SAMPLE":
                success = change_sample_time(value)
                self.driver.sample_time = value
            elif param == "MAXTILT":
                success = change_max_safe_tilt(value)
                self.driver.max_safe_tilt = value
            else:
                print(f"Unknown parameter: {param}")
                return

            # Add these debug prints to verify the update worked
            if success:
                print(f"✓ Updated {param} to {value}")
                if self.ble.connected:
                    self.ble.send_telemetry(f"Config updated: {param}={value}")
            else:
                print(f"✗ Failed to update {param}")
                if self.ble.connected:
                    self.ble.send_telemetry(f"Config update failed: {param}")
                    
        except Exception as e:
            error_msg = f"Error updating config: {e}"
            print(f"EXCEPTION in _update_config: {repr(e)}")
            print(error_msg)
            if self.ble.connected:
                self.ble.send_telemetry(f"ERROR: {error_msg}")

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
                print("Configuring robot...")
                if len(parts) >= 3:
                    param = parts[1].upper()
                    try:
                        print(f"Configuring {param}...")
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
                self.driver.mpu.calibrate_gyro()
                self.driver.mpu.calibrate_accel()
                return

            elif action == "STATUS":
                print("Showing status...")
                self.show_status()
                return

            elif action == "RESET":
                print("Resetting robot...")
                await self.reset()
                return

            else:
                print(f"Action not implemented: {action}")
                return

        # If we get here, it's an unrecognized command
        print(f"Unknown command: {cmd}")

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
                angle, left, right, speed, acceleration = self.driver.forward(
                    target_speed=self.speed, turn_bias=self.turn
                )
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


async def main():
    """Program entry point."""
    robot = SelfBalancingRobot()

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
        await robot.stop()
        # Cancel the BLE listener task
        ble_task.cancel()
        try:
            await ble_task
        except asyncio.CancelledError:
            pass


if __name__ == "__main__":
    asyncio.run(main())
