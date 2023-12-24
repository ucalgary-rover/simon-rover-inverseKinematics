import asyncio  # Provides for writing single-threaded concurrent code using coroutines
import websockets  # Provides full-featured and easy to use WebSocket implementation
import json  # Provides JSON encoding and decoding functionality

# Asynchronous function to send XYZ data to the server via WebSocket
async def send_xyz():
    uri = "ws://localhost:6789"  # The URI of the WebSocket server to connect to
    
    try:
        # Establish a connection to the WebSocket server
        async with websockets.connect(uri) as websocket:
            # Prompt the user to enter X, Y, and Z values
            x_value = int(input("Enter x value: "))  # Get X value from user
            y_value = int(input("Enter y value: "))  # Get Y value from user
            z_value = int(input("Enter z value: "))  # Get Z value from user
            xyz_data = [x_value, y_value, z_value]  # Combine the values into a list
            
            # Convert the XYZ data to a JSON formatted string and send it
            message = json.dumps({"xyz": xyz_data})
            await websocket.send(message)  # Send the message over WebSocket
            print("Data sent to server")  # Notify user of successful sending
    except Exception as e:
        # If any exceptions occur during connection or sending, print the error
        print(f"Could not connect to WebSocket server: {e}")

# Execute the send_xyz function within the asyncio event loop
asyncio.run(send_xyz())
