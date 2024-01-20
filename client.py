import asyncio
import websockets
import json

# The URI of the WebSocket server
uri = "ws://localhost:6789"

# Function to get input from the user
def get_input(prompt):
    try:
        integer = int(input(prompt))  # Get and return the input value
        if -1 <= integer <= 1:
            return integer
        else:
            print("\033[91m\n++++++Invalid input, please enter a number between -1 to 1++++++\n\033[0m")
            return get_input(prompt)
    except ValueError:
        print("\033[91m\n++++++Invalid input, please enter a number.++++++\n\033[0m")
        return get_input(prompt)

# Function to display the menu
def display_menu(values):
    print("\nCurrent values:")
    labels = ['X', 'Y', 'Z', 'Yaw', 'Roll', 'Pitch']
    for i, (label, value) in enumerate(zip(labels, values)):
        print(f"{label}: {value}")
    print("Send. Send data")
    print("Exit. Exit")

async def send_keepalive_ping(websocket):
    while True:
        await asyncio.sleep(5)  # Send a ping every 5 seconds
        if websocket.open:
            try:
                await websocket.ping()
            except websockets.exceptions.ConnectionClosed:
                break
async def send_data(websocket, values):
    display_menu(values)
    choice = input("Select an option to edit, send, or exit: ").upper()

    if choice in ['X', 'Y', 'Z', 'YAW', 'ROLL', 'PITCH']:
        index = ['X', 'Y', 'Z', 'YAW', 'ROLL', 'PITCH'].index(choice)
        values[index] = get_input(f"Enter new value for {choice}: ")
    elif choice == 'SEND':
        message = json.dumps({"data": values})
        await websocket.send(message)
        print("Data sent to server")
    elif choice == 'EXIT':
        return False
    else:
        print("Invalid choice, please select a valid option.")
    return True

async def main():
    values = [0, 0, 0, 0, 0, 0]
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                # Start the keepalive task
                keepalive_task = asyncio.create_task(send_keepalive_ping(websocket))

                continue_communication = True
                while continue_communication:
                    continue_communication = await send_data(websocket, values)

                keepalive_task.cancel()
                break  # Exit the loop if 'EXIT' is chosen
        except Exception as e:
            print(f"Connection lost. Attempting to reconnect: {e}")
            await asyncio.sleep(5)  # Wait for 5 seconds before trying to reconnect

asyncio.run(main())
