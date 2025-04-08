import asyncio
from bluethooth.BLEEmitter import BLEEmitter
import parameters.parameters as params
import threading
import queue

"""
THIS CODE SHOULD BE RAN IN THE PC
"""

COMMANDS = params.data["COMMANDS"]
input_queue = queue.Queue()

def get_user_input():
    """Function to run in a separate thread to get user input"""
    while True:
        user_input = input("\nEnter command: ")
        input_queue.put(user_input)
        if user_input.lower() == 'q':
            break

async def interactive_mode(controller):
    """Interactive mode to send commands to PicoRobot"""
    # Start a separate thread for user input
    input_thread = threading.Thread(target=get_user_input, daemon=True)
    input_thread.start()
    
    # Print the initial menu
    print("\n=== PicoRobot Control Panel ===")
    print("Available Commands:")
    for cmd_num, cmd_info in COMMANDS.items():
        print(f"{cmd_num}: {cmd_info['action']} - {cmd_info['description']}")
    print("q - Quit this program")
    
    # Process commands from the queue
    running = True
    while running:
        # Check if we have input without blocking
        try:
            # Non-blocking get with a short timeout
            choice = input_queue.get(block=False)
            
            if choice.lower() == "q":
                running = False
                break
                
            if choice not in COMMANDS:
                print("Invalid command. Please try again.")
                # Reprint the menu
                print("\n=== PicoRobot Control Panel ===")
                print("Available Commands:")
                for cmd_num, cmd_info in COMMANDS.items():
                    print(f"{cmd_num}: {cmd_info['action']} - {cmd_info['description']}")
                print("q - Quit this program")
                continue
                
            # Send the command
            print(f"Sending command: {choice}")
            await controller.send_command(choice)
            
        except queue.Empty:
            # No input available, just continue the loop
            pass
            
        # Allow other asyncio tasks to run (like processing BLE notifications)
        await asyncio.sleep(0.1)

async def main():
    """Main function"""
    # Create controller object with telemetry printing disabled
    controller = BLEEmitter(verbose_telemetry=False)

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
