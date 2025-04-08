import asyncio
from bluethooth.BLEEmitter import BLEEmitter
import parameters.parameters as params

"""
THIS CODE SHOULD BE RAN IN THE PC
"""

COMMANDS = params.data["COMMANDS"]


async def interactive_mode(controller):
    """Interactive mode to send commands to PicoRobot"""
    while True:
        print("\n=== PicoRobot Control Panel ===")
        print("Available Commands:")
        for cmd_num, cmd_info in COMMANDS.items():
            print(f"{cmd_num}: {cmd_info['action']} - {cmd_info['description']}")
        print("q - Quit this program")

        choice = input("\nEnter command: ")

        if choice.lower() == "q":
            break

        if choice.lower() not in COMMANDS:
            print("Invalid command. Please try again.")
            continue

        # Send the command as-is (the robot will parse it)
        print(f"Sending command: {choice}")
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
