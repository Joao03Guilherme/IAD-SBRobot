"""
SBRobot - Full Prototype Implementation
Self-balancing robot main control program
"""

import time
from machine import Pin, I2C

import asyncio
import _thread  # Add this import for thread support
from controllers.motor_controller import MotorController
from controllers.buzzer_controller import (
    BuzzerController,
)  # Import the new buzzer controller
from bluetooth_controllers.BLEReceiver import BLEReceiver
from controllers.balance_controller import Driving
from parameters.parameters import (
    data,
    change_kd,
    change_ki,
    change_kp,
    change_k_damping,
    change_balance2offset,
    change_sample_time,
    change_max_correction_tilt,
    change_max_safe_tilt,
    change_alpha,
)

# Load constants
MOTOR_CONFIG = data["MOTOR_CONFIG"]
COMMANDS = data["COMMANDS"]

# Status LED
status_led = Pin("LED", Pin.OUT)


class SelfBalancingRobot:
    """
    Main robot control class for the self-balancing robot.
    Handles motor, buzzer, BLE, and driving logic.
    """

    def __init__(self) -> None:
        """
        Initialize the SelfBalancingRobot, setting up controllers and BLE.
        Returns:
            None
        """
        print("Initializing Self-Balancing Robot...")

        # Create motor controller first
        self.motor_controller = MotorController(MOTOR_CONFIG)

        # Use the Driving class from drive.py
        self.driver = Driving(self.motor_controller)

        # Initialize buzzer controller
        self.buzzer = BuzzerController()

        # Wrapper for async callback
        def command_callback(cmd):
            asyncio.create_task(self.handle_command(cmd))

        self.ble = BLEReceiver()
        self.ble.set_command_callback(command_callback)

        # Robot state
        self.running = False
        self.speed = 0
        self.turn = 0

    async def ble_listener(self) -> None:
        """
        Listen for BLE commands and handle them.
        Starts/stops periodic beeping based on BLE connection status.
        Returns:
            None
        """
        while True:
            if self.ble.connected:
                # Stop the periodic beeping once connection is established
                if self.buzzer.periodic_beeping:
                    print("Connection established, stopping periodic beep")
                    await self.buzzer.stop_periodic_beeping()
                await asyncio.sleep(0.1)  # Adjust the sleep time as needed
            else:
                # If connection is lost, restart periodic beeping
                if not self.buzzer.periodic_beeping:
                    print("Connection lost, restarting periodic beep")
                    self.buzzer.start_periodic_beeping()
                await asyncio.sleep(0.5)  # Check less frequently when disconnected

    async def start(self) -> None:
        """
        Start the balancing loop and BLE listener concurrently.
        Returns:
            None
        """
        self.running = True
        await asyncio.gather(self.main_loop(), self.ble_listener())

    async def reset(self) -> None:
        """
        Reset the Driving class and stop the robot.
        Sends telemetry about the reset.
        Returns:
            None
        """
        self.running = False
        self.driver.stop()
        self.motor_controller = MotorController(MOTOR_CONFIG)
        self.driver = Driving(self.motor_controller)

        self.ble.send_telemetry(",M:Robot stopped and reset")
        self.ble.send_telemetry(f",C{str(data['PID_CONFIG'])}")

    def _update_config(self, param: str, value: float) -> None:
        """
        Update robot configuration parameters.
        Args:
            param (str): The parameter name to update.
            value (float): The new value for the parameter.
        Returns:
            None
        """
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
            elif param == "K_DAMPING":
                success = change_k_damping(value)
            elif param == "SAMPLE":
                success = change_sample_time(value)
                self.driver.sample_time = value
            elif param == "ALPHA":
                success = change_alpha(value)
            elif param == "B2O":
                success = change_balance2offset(value)
            elif param == "MCT":
                success = change_max_correction_tilt(value)
            elif param == "MAXTILT":
                success = change_max_safe_tilt(value)
                self.driver.max_safe_tilt = value
            else:
                print(f"Unknown parameter: {param}")
                return

            # Add these debug prints to verify the update worked
            if success:
                print(f"Updated {param} to {value}")
                if self.ble.connected:
                    self.ble.send_telemetry(f",M:Config updated: {param}={value}")
            else:
                print(f"Failed to update {param}")
                if self.ble.connected:
                    self.ble.send_telemetry(f",M:Config update failed: {param}")

        except Exception as e:
            error_msg = f"Error updating config: {e}"
            print(f"EXCEPTION in _update_config: {repr(e)}")
            print(error_msg)
            if self.ble.connected:
                self.ble.send_telemetry(f",M:ERROR: {error_msg}")

    def send_telemetry(self) -> None:
        """
        Send robot telemetry data via BLE.
        Includes angle, balance angle, speed, turn, running state, and power data.
        Returns:
            None
        """
        if not self.ble.connected:
            return

        # Get the latest sensor data
        angle = 0
        if len(self.driver.angle_data) > 0:
            angle = self.driver.angle_data[-1]

        balance_angle = 0
        if len(self.driver.balance_angle_data) > 0:
            balance_angle = self.driver.balance_angle
        
        # Get the distance traveled 
        distance = (self.driver.wheel_encoder_a.pulse_count + self.driver.wheel_encoder_b.pulse_count)/2

        # Create telemetry string (simpler than JSON)
        telemetry = f"A:{angle:.2f},B:{balance_angle:.2f},S:{self.speed},T:{self.turn},R:{1 if self.running else 0},D:{distance:.2f},P:{self.driver.p_term:.2f},Y:{self.driver.d_term:.2f},I:{self.driver.i_term:.2f}"
        if self.driver.left_power_data and self.driver.right_power_data:
            telemetry += f",L:{self.driver.left_power_data[-1]},R:{self.driver.right_power_data[-1]}"

        # Send telemetry
        self.ble.send_telemetry(telemetry)

    def find_balance_point(self) -> float | None:
        """
        Utility function to find the natural balance point of the robot.
        This helps determine the optimal balance_angle value.
        Returns:
            float | None: The average balance angle, or None if no samples collected.
        """
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

    async def handle_command(self, cmd: str | bytes) -> None:
        """
        Process BLE commands.
        Args:
            cmd (str | bytes): The command received from BLE.
        Returns:
            None
        """
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
        parts = cmd.split(" ")
        cmd_num = parts[0]

        # Add debug print to see if command even reaches here
        print(f"DEBUG: Processing command num '{cmd_num}' from parts {parts}")

        # Make sure "7" is properly recognized as a SOUND command
        if cmd_num == "7":
            print("SOUND command detected via '7'")

            # If no sound type provided, show error
            if len(parts) < 2:
                error_msg = "Sound command requires a sound type (starwars, r2d2, stop)"
                print(error_msg)
                self.ble.send_telemetry(f",M:{error_msg}")
                return

            # Get sound type and set default parameters
            sound_type = parts[1].lower()
            print(f"Playing sound: {sound_type}")

            # Set default parameters
            tempo = data["BUZZER_CONFIG"]["default_tempo"]
            volume = data["BUZZER_CONFIG"]["default_volume"]

            # Set volume to maximum for better audibility
            self.buzzer.set_volume(1.0)

            # Process sound type
            if sound_type == "starwars":
                self.buzzer.play_star_wars_song()
                self.ble.send_telemetry(f",M:Playing Star Wars theme")
            elif sound_type == "r2d2":
                duration = 5
                self.buzzer.play_random_sound(duration_seconds=duration)
                self.ble.send_telemetry(f",M:Playing R2D2 sounds")
            elif sound_type == "stop":
                asyncio.create_task(self.buzzer.stop())
                self.ble.send_telemetry(",M:Stopped all sounds")
            else:
                self.ble.send_telemetry(f",M:Unknown sound type: {sound_type}")

            return

        # Continue with the normal command handling
        if cmd_num in COMMANDS:
            action = COMMANDS[cmd_num]["action"]
            print(f"Executing {action} command")

            if action == "START":
                print("Starting robot...")
                await self.start()
                return

            elif action == "STOP":
                print("Stopping robot...")
                await self.reset()
                return

            elif action == "DRIVE":
                if len(parts) == 3:
                    try:
                        self.speed = int(parts[1])
                        self.turn = int(parts[2])
                        print(f"Setting drive: speed={self.speed}, turn={self.turn}")

                        # Start the robot if it's not already running
                        if not self.running:
                            print("Auto-starting robot for drive command...")
                            await self.start()

                        self.driver.forward(
                            target_speed=self.speed, turn_bias=self.turn
                        )

                        return
                    except ValueError:
                        print("Invalid speed or turn value")
                else:
                    print("Error: DRIVE command requires at least a speed parameter")
                    print("Usage: 3 <speed> [turn]")
                return

            elif action == "CONFIG":
                print("Configuring robot...")
                if len(parts) == 3:
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
                self.ble.send_telemetry(",M:Calibrating gyro and accelerometer...")

                # Make sure volume is at maximum
                self.buzzer.set_volume(1.0)
                print("Starting calibration melody")

                # Create a simpler repeating melody that's easier to hear
                calibration_melody = self.buzzer.star_wars_melody * 2

                # Start the melody playing in a background task
                melody_task = self.buzzer.play_melody_async(
                    calibration_melody, base_tempo=100
                )

                # Create a flag to track calibration completion
                calibration_done = False

                # Define a function to run calibration in a separate thread
                def run_calibration():
                    nonlocal calibration_done
                    try:
                        print("Running MPU calibration in separate thread")
                        num_samples = data["MPU_CONFIG"]["calibration_samples"]
                        self.driver.mpu.calibrate_mpu(num_samples=num_samples)
                        print("Calibration thread complete")
                        calibration_done = True
                    except Exception as e:
                        print(f"Error in calibration thread: {e}")
                        calibration_done = True

                # Start calibration in a separate thread
                _thread.start_new_thread(run_calibration, ())

                # Wait for calibration to complete while allowing the event loop to run
                print("Waiting for calibration to complete...")
                while not calibration_done:
                    await asyncio.sleep(0.1)  # Yield to event loop regularly

                print("Calibration complete - stopping melody")
                # Stop the melody now that calibration is done
                await self.buzzer.stop()

                # Notify user
                num_samples = data["MPU_CONFIG"]["calibration_samples"]
                self.ble.send_telemetry(
                    f",M:Calibrated the gyro and accelerometer with {num_samples} samples!"
                )
                return

            elif action == "SOUND":
                print("Sound command received via COMMANDS dictionary")
                if len(parts) < 2:
                    print(
                        "Error: SOUND command requires a sound type (starwars, r2d2, stop)"
                    )
                    self.ble.send_telemetry(f",M:Sound command requires a sound type")
                    return

            else:
                print(f"Action not implemented: {action}")
                return

        # If we get here, it's an unrecognized command
        print(f"Unknown command: {cmd}")

    async def main_loop(self) -> None:
        """
        Main control loop for balancing and driving.
        Sends telemetry and checks for safety conditions.
        Returns:
            None
        """
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


async def main() -> None:
    """
    Program entry point.
    Initializes the robot and runs the BLE listener and telemetry loop.
    Handles graceful shutdown on KeyboardInterrupt.
    Returns:
        None
    """
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
        await robot.reset()  # Ensure the robot stops
        # Cancel the BLE listener task
        ble_task.cancel()
        try:
            await ble_task
        except asyncio.CancelledError:
            pass


if __name__ == "__main__":
    asyncio.run(main())
