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
from typing import Deque, List


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
        # For plotting angle - add type hint to fix the issue
        self.angle_buffer: Deque[float] = deque(maxlen=100)
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
            print(f"Sending command: {command}")
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
                print(f"📊 Telemetry: {decoded}")

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

                if "A" in telemetry_dict:
                    angle = float(telemetry_dict["A"])
                    if self.verbose_telemetry:
                        print(f"  Angle: {angle}°")
                    self.angle_buffer.append(angle)

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
        (self.line,) = self.ax.plot([], [], lw=2, color="blue")
        self.ax.set_ylim(-50, 50)  # Assuming angles range from -180° to 180°
        self.ax.set_xlim(0, 100)
        self.ax.grid(True)
        self.ax.set_title("📈 Real-time Angle Plot")
        self.ax.set_xlabel("Time (ticks)")
        self.ax.set_ylabel("Angle (°)")
        self.fig.tight_layout()
        plt.show(block=False)

        # Create empty data to initialize plot
        x_data = list(range(100))
        y_data = [0] * 100
        self.line.set_data(x_data, y_data)

        # Update plot in a loop
        while self.plot_running:
            try:
                # Update the data
                if len(self.angle_buffer) > 0:
                    # Convert deque to list with explicit typing to satisfy type checker
                    buffer_list: List[float] = [x for x in self.angle_buffer]
                    x_data = list(range(len(buffer_list)))
                    self.line.set_data(x_data, buffer_list)

                    # Only adjust y limits if needed (avoid constant rescaling)
                    min_val = min(buffer_list) if buffer_list else -180
                    max_val = max(buffer_list) if buffer_list else 180
                    current_ymin, current_ymax = self.ax.get_ylim()

                    if min_val < current_ymin or max_val > current_ymax:
                        # Add some padding to limits
                        padding = (
                            (max_val - min_val) * 0.1 if max_val != min_val else 10
                        )
                        new_min = min(min_val - padding, current_ymin)
                        new_max = max(max_val + padding, current_ymax)
                        self.ax.set_ylim(new_min, new_max)

                    # Adjust x limit to show all data points
                    if len(buffer_list) > self.ax.get_xlim()[1]:
                        self.ax.set_xlim(0, len(buffer_list))

                # Force redraw
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()

                # Sleep briefly to avoid hogging CPU
                time.sleep(0.05)  # Update at 20fps for smoother animation

            except Exception as e:
                print(f"Plot update error: {e}")
                time.sleep(0.5)  # If error occurs, wait a bit before retrying
