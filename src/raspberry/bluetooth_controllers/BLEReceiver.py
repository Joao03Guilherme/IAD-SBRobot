import bluetooth
import json
from machine import Pin
import parameters.parameters as params

"""
THIS CODE SHOULD BE RAN IN THE raspberry PI
"""

# Status LED for visual feedback
led = Pin("LED", Pin.OUT)  # Onboard LED

# BLE flags and constants - using raw values to avoid issues
_IRQ_CENTRAL_CONNECT = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE = 3


class BLEReceiver:
    def __init__(self, name: str = params.data["PICO_NAME"]) -> None:
        """
        Initialize the BLE controller with the given device name.

        Args:
            name (str, optional): The BLE device name. Defaults to value from config.
        """
        self.name = name
        self.ble = bluetooth.BLE()
        self.ble.active(True)

        # Connection status
        self.connected = False
        self.conn_handle = None

        # Command callback
        self.command_callback = None

        # Register services
        self._register_services()

        # Register IRQ handler
        self.ble.irq(self._irq_handler)

        # Start advertising
        self._advertise()

        print(f"BLE initialized as '{name}'")

    def _register_services(self) -> None:
        """
        Register BLE services and characteristics for control and telemetry.

        Returns:
            None
        """
        # Define UUIDs for service and characteristics
        ROBOT_SERVICE_UUID = bluetooth.UUID(params.data["ROBOT_SERVICE_UUID"])
        CONTROL_CHAR_UUID = bluetooth.UUID(params.data["CONTROL_CHAR_UUID"])
        TELEMETRY_CHAR_UUID = bluetooth.UUID(params.data["TELEMETRY_CHAR_UUID"])

        service = (
            ROBOT_SERVICE_UUID,
            (
                (CONTROL_CHAR_UUID, 0x000A),  # FLAG_WRITE | FLAG_READ
                (TELEMETRY_CHAR_UUID, 0x0012),  # FLAG_READ | FLAG_NOTIFY
            ),
        )

        # Register the service
        services = self.ble.gatts_register_services((service,))

        # Save the handles
        self._handle = services[0]
        self._control_handle = services[0][0]
        self._telemetry_handle = services[0][1]

        # Set initial values
        self.ble.gatts_write(self._control_handle, b"READY")
        self.ble.gatts_write(self._telemetry_handle, b'{"status":"ready"}')

        print("Services registered successfully")

    def _advertise(self) -> None:
        """
        Start BLE advertising with the device name.

        Returns:
            None
        """
        # Prepare advertising payload with name
        payload = bytearray()

        # Add flags
        payload += b"\x02\x01\x06"

        # Add name
        name_bytes = bytes(self.name, "utf-8")
        payload += bytes([len(name_bytes) + 1, 0x09]) + name_bytes

        # Start advertising
        self.ble.gap_advertise(100000, adv_data=payload)
        print("Advertising started")

    def _irq_handler(self, event: int, data) -> None:
        """
        Handle BLE events (connect, disconnect, write).

        Args:
            event (int): The BLE event type.
            data: Event-specific data.
        Returns:
            None
        """
        # Connection event
        print(f"BLE event received: {event}")  #  this to track all events

        if event == _IRQ_CENTRAL_CONNECT:
            self.conn_handle, _, _ = data
            self.connected = True
            print("Device connected")
            led.value(1)  # Turn on LED when connected

        # Disconnection event
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self.conn_handle = None
            self.connected = False
            print("Device disconnected")
            led.value(0)  # Turn off LED when disconnected

            # Restart advertising
            self._advertise()

        # Write to characteristic event
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data

            # Check if control characteristic was written to
            if value_handle == self._control_handle:
                # Read the command data
                data_bytes = self.ble.gatts_read(self._control_handle)

                try:
                    # Try to decode as string
                    command = data_bytes.decode().strip()
                    print(f"Command received: {command}")

                    # Process command via callback
                    if self.command_callback:
                        self.command_callback(command)

                except:
                    # Fallback for binary data
                    print(f"Binary data received: {data_bytes}")

    def send_telemetry(self, data: dict | str) -> bool:
        """
        Send telemetry data to the connected device.

        Args:
            data (dict | str): Dictionary or string to send as telemetry.
        Returns:
            bool: True if telemetry was sent, False otherwise.
        """
        if not self.connected:
            return False

        try:
            # Convert to JSON if dictionary
            if isinstance(data, dict):
                json_data = json.dumps(data).encode()
            else:
                # Otherwise encode as string
                json_data = str(data).encode()

            # Write to telemetry characteristic
            self.ble.gatts_write(self._telemetry_handle, json_data)

            # Notify the client if connected
            if self.conn_handle is not None:
                self.ble.gatts_notify(self.conn_handle, self._telemetry_handle)
            return True
        except Exception as e:
            print(f"Error sending telemetry: {e}")
            return False

    def set_command_callback(self, callback) -> None:
        """
        Set the callback function for received commands.

        Args:
            callback: Function that takes a command string as parameter.
        Returns:
            None
        """
        self.command_callback = callback
