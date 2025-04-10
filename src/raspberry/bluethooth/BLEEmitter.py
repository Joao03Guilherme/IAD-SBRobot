from bleak import BleakClient
from bleak import BleakScanner
import parameters.parameters as params
import matplotlib

# Set the backend before importing pyplot
matplotlib.use("TkAgg")  # Use TkAgg backend for better threading support
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time
from typing import Deque


class BLEEmitter:
    def __init__(
        self,
        device_address=None,
        device_name=params.data["PICO_NAME"],
        verbose_telemetry=True,
    ):
        self.device_address = device_address
        self.device_name = device_name
        self.client = None
        self.verbose_telemetry = verbose_telemetry
        self.angle_buffer: Deque[float] = deque(maxlen=100)
        self.balance_angle_buffer: Deque[float] = deque(maxlen=100)
        self.plot_initialized = False

    async def connect(self):
        """Connect to the PicoRobot"""
        # If no address specified, try to find by name
        if not self.device_address:
            print(f"Scanning for {self.device_name}...")
            devices = await BleakScanner.discover()
            for d in devices:
                print(f"{d.address}: {d.name}")
                if d.name == self.device_name:
                    self.device_address = d.address
                    print(f"Found {self.device_name} at {d.address}")
                    break

            if not self.device_address:
                print(f"{self.device_name} not found!")
                return False

        try:
            # Connect to the device
            print(f"Connecting to {self.device_name} at {self.device_address}...")
            self.client = BleakClient(self.device_address)
            await self.client.connect()

            if self.client.is_connected:
                print(f"Connected to {self.device_name}!")

                # Subscribe to telemetry notifications
                await self.client.start_notify(
                    params.data["TELEMETRY_CHAR_UUID"], self._on_telemetry
                )
                print("Subscribed to telemetry notifications")
                self.start_angle_plot()
                return True
            else:
                print("Failed to connect")
                return False

        except Exception as e:
            print(f"Error connecting: {e}")
            return False

    async def disconnect(self):
        """Disconnect from the device"""
        # Stop the plot thread
        if hasattr(self, "plot_running"):
            self.plot_running = False

        if self.client and self.client.is_connected:
            # Unsubscribe from notifications
            try:
                await self.client.stop_notify(params.data["TELEMETRY_CHAR_UUID"])
            except:
                pass
            # Disconnect
            await self.client.disconnect()
            print("Disconnected")

    async def send_command(self, command):
        """Send a command to the PicoRobot"""
        if not self.client or not self.client.is_connected:
            print("Not connected!")
            return False

        try:
            # Handle command whether it's a string or bytes
            if isinstance(command, str):
                command_bytes = command.encode()
            else:
                command_bytes = command

            await self.client.write_gatt_char(
                params.data["CONTROL_CHAR_UUID"], command_bytes
            )
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def _on_telemetry(self, sender, data):
        """Handle telemetry data"""
        try:
            # Decode the data as a formatted string
            decoded = data.decode()
            if self.verbose_telemetry:
                print(f"Telemetry: {decoded}")

            if ",C" in decoded:
                print(f"\nCurrent config file: {decoded[2:]}")
                return

            # Parse the formatted string into a dictionary
            if "," in decoded:
                telemetry_dict = {}
                parts = decoded.split(",")
                for part in parts:
                    if ":" in part:
                        key, value = part.split(":", 1)
                        telemetry_dict[key] = value

                # Print in a more readable format
                if self.verbose_telemetry:
                    print("Parsed telemetry data:")

                if "M" in telemetry_dict:
                    # Messages
                    message = telemetry_dict["M"]
                    print(f"\nMessage: {message}")

                if "A" in telemetry_dict:
                    angle = float(telemetry_dict["A"])
                    if self.verbose_telemetry:
                        print(f"  Angle: {angle}°")
                    self.angle_buffer.append(angle)

                if "B" in telemetry_dict:
                    balance_angle = float(telemetry_dict["B"])
                    if self.verbose_telemetry:
                        print(f"  Balance target: {balance_angle}°")
                    self.balance_angle_buffer.append(balance_angle)

                if self.verbose_telemetry:
                    if "S" in telemetry_dict:
                        print(f"  Speed: {telemetry_dict['S']}")
                    if "T" in telemetry_dict:
                        print(f"  Turn: {telemetry_dict['T']}")
                    if "R" in telemetry_dict:
                        print(
                            f"  Running: {'Yes' if telemetry_dict['R']=='1' else 'No'}"
                        )
                    if "L" in telemetry_dict:
                        print(f"  Left motor: {telemetry_dict['L']}")
                    if "R" in telemetry_dict:
                        print(f"  Right motor: {telemetry_dict['R']}")
        except:
            # If can't decode, show raw data
            if self.verbose_telemetry:
                print(f"📊 Raw telemetry: {data}")

    def start_angle_plot(self):
        """Start a rolling plot for the angle value in a separate thread."""
        if self.plot_initialized:
            return

        self.plot_initialized = True
        self.plot_running = True

        # Start plotting in a separate thread
        plot_thread = threading.Thread(target=self._run_plot_thread)
        plot_thread.daemon = True  # Thread will exit when main program exits
        plot_thread.start()

    def _run_plot_thread(self):
        """Run the plot in a separate thread."""
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots(figsize=(8, 5))

        # Create two line objects instead of one
        (self.line,) = self.ax.plot([], [], lw=2, color="blue", label="Current Angle")
        (self.balance_line,) = self.ax.plot(
            [], [], lw=2, color="red", label="Target Angle"
        )

        self.ax.set_ylim(-50, 50)
        self.ax.set_xlim(0, 100)
        self.ax.grid(True)
        self.ax.set_title("Real-time Angle Plot")  # Removed emoji to avoid font issues
        self.ax.set_xlabel("Time (ticks)")
        self.ax.set_ylabel("Angle (°)")
        self.ax.legend(loc="upper right")  # Add legend to identify lines
        self.fig.tight_layout()
        plt.show(block=False)

        # Update plot in a loop
        while self.plot_running:
            try:
                # Update the data for the current angle line
                if self.angle_buffer:
                    y_data = [x for x in self.angle_buffer]
                    x_data = list(range(len(y_data)))
                    self.line.set_data(x_data, y_data)

                # Update the data for the balance angle line
                if self.balance_angle_buffer:
                    y_data = [x for x in self.balance_angle_buffer]
                    x_data = list(range(len(y_data)))
                    self.balance_line.set_data(x_data, y_data)

                # Dynamically adjust the x-axis if needed
                max_len = max(
                    len(self.angle_buffer), len(self.balance_angle_buffer), 100
                )
                if max_len > self.ax.get_xlim()[1]:
                    self.ax.set_xlim(0, max_len)

                # Redraw the plot
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

            except Exception as e:
                print(f"Plot error: {e}")

            # Sleep for a short duration to control update rate
            time.sleep(0.1)
        plt.close(self.fig)
