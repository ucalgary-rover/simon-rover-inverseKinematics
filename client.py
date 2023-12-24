import asyncio
import websockets
import json

async def send_xyz():
    uri = "ws://localhost:6789"
    try:
        async with websockets.connect(uri) as websocket:
            xyz_data = [45, 30, 10]
            message = json.dumps({"xyz": xyz_data})
            await websocket.send(message)
            print("Data sent to server")
    except Exception as e:
        print(f"Could not connect to WebSocket server: {e}")

asyncio.run(send_xyz())
