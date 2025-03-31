import asyncio
from bleak import BleakClient
from bleak import BleakScanner

"""
THIS CODE SHOULD BE RAN IN THE PC
"""

# Device information
PICO_NAME = "PicoBot"
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
            # Handle command whether it's a string or bytes
            if isinstance(command, str):
                command_bytes = command.encode()
            else:
                command_bytes = command
                
            await self.client.write_gatt_char(CONTROL_CHAR_UUID, command_bytes)
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
            if ',' in decoded:
                telemetry_dict = {}
                parts = decoded.split(',')
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        telemetry_dict[key] = value
                
                # Print in a more readable format
                print("Parsed telemetry data:")
                if 'A' in telemetry_dict: print(f"  Angle: {telemetry_dict['A']}°")
                if 'S' in telemetry_dict: print(f"  Speed: {telemetry_dict['S']}")
                if 'T' in telemetry_dict: print(f"  Turn: {telemetry_dict['T']}")
                if 'R' in telemetry_dict: print(f"  Running: {'Yes' if telemetry_dict['R']=='1' else 'No'}")
                if 'L' in telemetry_dict: print(f"  Left motor: {telemetry_dict['L']}")
                if 'R' in telemetry_dict: print(f"  Right motor: {telemetry_dict['R']}")
        except:
            # If can't decode, show raw data
            print(f"📊 Raw telemetry: {data}")


# Commands matching those in full_prototype.py
COMMANDS = {
    "1": {"action": "START", "description": "Start balancing"},
    "2": {"action": "STOP", "description": "Stop all motors"},
    "3": {"action": "DRIVE", "description": "Drive with speed & turn (e.g., 3 20 5)"},
    "4": {"action": "TURN", "description": "Turn by angle (e.g., 4 90 1)"},
    "5": {"action": "CONFIG", "description": "Update configuration"},
    "6": {"action": "CALIBRATE", "description": "Start calibration procedure"},
    "7": {"action": "HELP", "description": "Show this help"},
    "8": {"action": "STATUS", "description": "Show current robot status"}
}

async def interactive_mode(controller):
    """Interactive mode to send commands to PicoRobot"""
    while True:
        print("\n=== PicoRobot Control Panel ===")
        print("Available Commands:")
        for cmd_num, cmd_info in COMMANDS.items():
            print(f"{cmd_num}: {cmd_info['action']} - {cmd_info['description']}")
        
        print("\nExamples:")
        print("1 - Start balancing")
        print("2 - Stop motors")
        print("3 20 5 - Drive forward at speed 20, turn bias 5")
        print("4 45 1 - Turn 45 degrees to the right")
        print("7 - Show help")
        print("q - Quit this program")
        
        choice = input("\nEnter command: ")
        
        if choice.lower() == 'q':
            break
        
        # Send the command as-is (the robot will parse it)
        await controller.send_command(choice)
        
        # Small delay to allow for BLE responses
        await asyncio.sleep(0.5)

async def main():
    """Main function"""
    # Create controller object
    controller = BLEEmitter()
    
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




