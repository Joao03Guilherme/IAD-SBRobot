import asyncio
from bluethooth.BLEEmitter import BLEEmitter

"""
THIS CODE SHOULD BE RAN IN THE PC
"""

# Commands matching those in full_prototype.py
COMMANDS = {
    "1": {"action": "START", "description": "Start balancing"},
    "2": {"action": "STOP", "description": "Stop all motors"},
    "3": {"action": "DRIVE", "description": "Drive with speed & turn (e.g., 3 20 5)"},
    "4": {"action": "TURN", "description": "Turn by angle (e.g., 4 90 1)"},
    "5": {"action": "CONFIG", "description": "Update configuration"},
    "6": {"action": "CALIBRATE", "description": "Start calibration procedure"},
    "7": {"action": "HELP", "description": "Show this help"},
    "8": {"action": "STATUS", "description": "Show current robot status"},
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

        if choice.lower() == "q":
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
