from bleak import BleakClient
from bleak import BleakScanner
import parameters.parameters as params
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class BLEEmitter:
    def __init__(
        self,
        device_address=params.data["PICO_ADDRESS"],
        device_name=params.data["PICO_NAME"],
    ):
        self.device_address = device_address
        self.device_name = device_name
        self.client = None
        # For plotting angle
        self.angle_buffer = deque(maxlen=100)
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
                print("Parsed telemetry data:")
                if "A" in telemetry_dict:
                    angle = float(telemetry_dict["A"])
                    print(f"  Angle: {angle}°")
                    self.angle_buffer.append(angle)
                if "S" in telemetry_dict:
                    print(f"  Speed: {telemetry_dict['S']}")
                if "T" in telemetry_dict:
                    print(f"  Turn: {telemetry_dict['T']}")
                if "R" in telemetry_dict:
                    print(f"  Running: {'Yes' if telemetry_dict['R']=='1' else 'No'}")
                if "L" in telemetry_dict:
                    print(f"  Left motor: {telemetry_dict['L']}")
                if "R" in telemetry_dict:
                    print(f"  Right motor: {telemetry_dict['R']}")
        except:
            # If can't decode, show raw data
            print(f"📊 Raw telemetry: {data}")

    def start_angle_plot(self):
        """Start a rolling plot for the angle value."""
        if self.plot_initialized:
            return

        self.plot_initialized = True
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], lw=2)
        self.ax.set_ylim(-180, 180)  # Assuming angles range from -180° to 180°
        self.ax.set_xlim(0, 100)
        self.ax.set_title("📈 Real-time Angle Plot")
        self.ax.set_xlabel("Time (ticks)")
        self.ax.set_ylabel("Angle (°)")

        def update(frame):
            self.line.set_data(range(len(self.angle_buffer)), list(self.angle_buffer))
            return (self.line,)

        self.ani = FuncAnimation(self.fig, update, interval=200, blit=True)
        plt.show()
