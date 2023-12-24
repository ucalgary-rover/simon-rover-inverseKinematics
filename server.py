# Importing necessary libraries
import asyncio  # Provides for writing single-threaded concurrent code using coroutines
import websockets  # Provides full-featured and easy to use WebSocket implementation
import json  # Provides JSON encoding and decoding functionality
import paho.mqtt.client as mqtt  # Provides client functionalities for interacting with MQTT brokers

# Configuration for connecting to an MQTT broker
MQTT_BROKER = 'localhost'  # Address of the MQTT broker
MQTT_PORT = 1883  # Port number the MQTT broker is listening on
MQTT_TOPIC = 'xyz_topic'  # The MQTT topic to publish messages to

# Initialize MQTT Client
mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    # Callback for when the client receives a CONNACK response from the server.
    if rc == 0:
        # Successful connection
        print(f"Connected to MQTT Broker at {MQTT_BROKER}:{MQTT_PORT}")
    else:
        # Unsuccessful connection, rc is the error code
        print(f"Failed to connect to MQTT Broker, return code {rc}")

# Assign the callback function for MQTT connection
mqtt_client.on_connect = on_connect

# Try to connect to the MQTT broker
try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)  # Connect to MQTT broker
except Exception as e:
    print(f"Could not connect to MQTT Broker: {e}")  # Print error if connection fails
    exit()  # Exit the script if connection cannot be established

# Start the MQTT client's network loop
mqtt_client.loop_start()

# Function to apply limits to the XYZ values
def limit_xyz(xyz):
    # Define limits for each of x, y, and z values
    x_limit = (0, 100)
    y_limit = (0, 100)
    z_limit = (0, 100)

    # Apply the limits to each value
    x, y, z = xyz
    x = max(min(x, x_limit[1]), x_limit[0])
    y = max(min(y, y_limit[1]), y_limit[0])
    z = max(min(z, z_limit[1]), z_limit[0])
    
    return x, y, z  # Return the limited x, y, z values

# Asynchronous handler function for WebSocket connections
async def handler(websocket, path):
    print("Server is waiting for client to send data...")
    async for message in websocket:  # Asynchronously listen for messages
        try:
            data = json.loads(message)  # Attempt to parse message as JSON
            xyz = data.get("xyz")  # Extract the 'xyz' data from the message
            
            # Validate and process the 'xyz' data
            if xyz and len(xyz) == 3:
                limited_xyz = limit_xyz(xyz)  # Apply limits to the 'xyz' values
                # Publish the limited xyz data to the MQTT topic
                mqtt_client.publish(MQTT_TOPIC, json.dumps({"limited_xyz": limited_xyz}))
                print(f"Received and published limited XYZ data: {limited_xyz}")
            else:
                print("Invalid data received, expected XYZ array.")
        except json.JSONDecodeError:
            # Handle case where message is not valid JSON
            print("Invalid message format received, expected JSON.")

# Try to start the WebSocket server
try:
    # Prepare the WebSocket server to listen on localhost:6789
    start_server = websockets.serve(handler, "localhost", 6789)
    # Start and run the server indefinitely
    asyncio.get_event_loop().run_until_complete(start_server)
    print("WebSocket Server running on localhost:6789")
    asyncio.get_event_loop().run_forever()
except Exception as e:
    # Print error if the server fails to start
    print(f"Could not start WebSocket server: {e}")
    mqtt_client.loop_stop()  # Stop the MQTT client loop if the server fails to start
