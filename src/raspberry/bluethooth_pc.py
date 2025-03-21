import asyncio
from bleak import BleakClient
from bleak import BleakScanner

"""
THIS CODE SHOULD BE RAN IN THE PC
"""

# Device information
PICO_NAME = "PicoRobot"
PICO_ADDRESS = "33844EF1-F428-5A3B-DF48-83DFC12F9890"  # Your Pico's address

# Service and characteristic UUIDs - match those in your bluethooth_receiver.py
ROBOT_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
CONTROL_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"  # For sending commands
TELEMETRY_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"  # For receiving data

class BLEEmitter:
    def __init__(self, device_address=None, device_name=PICO_NAME):
        self.device_address = device_address
        self.device_name = device_name
        self.client = None
        
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
                await self.client.start_notify(TELEMETRY_CHAR_UUID, self._on_telemetry)
                print("Subscribed to telemetry notifications")
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
                await self.client.stop_notify(TELEMETRY_CHAR_UUID)
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
            await self.client.write_gatt_char(CONTROL_CHAR_UUID, command.encode())
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def _on_telemetry(self, sender, data):
        """Handle telemetry data"""
        try:
            # Try to decode as JSON
            decoded = data.decode()
            print(f"📊 Telemetry: {decoded}")
        except:
            # If can't decode, show raw data
            print(f"📊 Raw telemetry: {data}")

# Available commands menu
COMMANDS = {
    "1": "RUN",
    "2": "STOP",
}

async def interactive_mode(controller):
    """Interactive mode to send commands to PicoRobot"""
    while True:
        print("\n=== PicoRobot Control Panel ===")
        print("Basic Commands:")
        for key, cmd in COMMANDS.items():
            print(f"{key}: {cmd}")
        
        choice = input("\nEnter command: ")
        
        if choice.lower() == 'q':
            break
        
        if choice in COMMANDS:
            await controller.send_command(COMMANDS[choice])
        else:
            print("Invalid command. Please try again.")
        
        # Small delay to allow for BLE responses
        await asyncio.sleep(0.5)

async def main():
    """Main function"""
    # Create controller object
    controller = BLEEmitter(PICO_ADDRESS)
    
    try:
        # Try to connect
        if await controller.connect():
            # Run interactive mode
            await interactive_mode(controller)
        
    finally:
        # Always disconnect properly
        await controller.disconnect()

# Run the main function
if __name__ == "__main__":
    asyncio.run(main())