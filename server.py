import asyncio
import websockets
import json
import paho.mqtt.client as mqtt

# MQTT setup
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC = 'xyz_topic'

# Initialize MQTT Client and set up connection
mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT Broker at {MQTT_BROKER}:{MQTT_PORT}")
    else:
        print(f"Failed to connect to MQTT Broker, return code {rc}")

mqtt_client.on_connect = on_connect

# Try to connect to the MQTT broker
try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
except Exception as e:
    print(f"Could not connect to MQTT Broker: {e}")
    exit()

mqtt_client.loop_start()

# Function to limit XYZ values
def limit_xyz(xyz):
    # Define limits for x, y, z
    x_limit = (0, 100)
    y_limit = (0, 100)
    z_limit = (0, 100)

    x, y, z = xyz
    x = max(min(x, x_limit[1]), x_limit[0])
    y = max(min(y, y_limit[1]), y_limit[0])
    z = max(min(z, z_limit[1]), z_limit[0])
    
    return x, y, z

async def handler(websocket, path):
    print("Server is waiting for client to send data...")
    async for message in websocket:
        try:
            data = json.loads(message)
            xyz = data.get("xyz")
            
            if xyz and len(xyz) == 3:
                limited_xyz = limit_xyz(xyz)
                mqtt_client.publish(MQTT_TOPIC, json.dumps({"limited_xyz": limited_xyz}))
                print(f"Received and published limited XYZ data: {limited_xyz}")
            else:
                print("Invalid data received, expected XYZ array.")
        except json.JSONDecodeError:
            print("Invalid message format received, expected JSON.")

try:
    start_server = websockets.serve(handler, "localhost", 6789)
    asyncio.get_event_loop().run_until_complete(start_server)
    print("WebSocket Server running on localhost:6789")
    asyncio.get_event_loop().run_forever()
except Exception as e:
    print(f"Could not start WebSocket server: {e}")
    mqtt_client.loop_stop()
