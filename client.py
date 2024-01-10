import asyncio
import websockets
import json

# Asynchronous function to send data to the server via WebSocket
async def send_data():
    uri = "ws://localhost:6789"  # The URI of the WebSocket server

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
        labels = ['X', 'Y', 'Z', 'A', 'B', 'C']
        for i, (label, value) in enumerate(zip(labels, values)):
            print(f"{label}. {label}: {value}")
        print("Send. Send data")
        print("Exit. Exit")

    try:
        # Establish a connection to the WebSocket server
        async with websockets.connect(uri) as websocket:
            # Initialize values
            values = [0, 0, 0, 0, 0, 0]

            # Loop for input and editing
            while True:
                display_menu(values)
                choice = input("Select an option to edit, send, or exit: ").upper()

                if choice in ['X', 'Y', 'Z', 'A', 'B', 'C']:
                    index = ['X', 'Y', 'Z', 'A', 'B', 'C'].index(choice)
                    values[index] = get_input(f"Enter new value for {choice}: ")
                elif choice == 'SEND':
                    # Convert the data to a JSON formatted string and send it
                    message = json.dumps({"data": values})
                    await websocket.send(message)  # Send the message over WebSocket
                    print("Data sent to server")  # Notify user of successful sending
                elif choice == 'EXIT':
                    print("Exiting program.")
                    break
                else:
                    print("Invalid choice, please select a valid option.")

    except Exception as e:
        # If any exceptions occur during connection or sending, print the error
        print(f"Could not connect to WebSocket server: {e}")

# Execute the send_data function within the asyncio event loop
asyncio.run(send_data())
