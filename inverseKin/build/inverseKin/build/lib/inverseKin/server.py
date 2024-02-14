import asyncio
import websockets
import json
import logging
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# Initialize logging
logging.basicConfig(level=logging.INFO)

# Initialize ROS 2
rclpy.init()

class MyROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')
        self.publisher = self.create_publisher(Int32MultiArray, 'goal_position', 10)

    def publish_data(self, data):
        int_data = [int(val * 100) for val in data]  # Example: scale and convert to int
        msg = Int32MultiArray()
        msg.data = int_data
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

# Create a ROS 2 node
ros_node = MyROSNode()

def limit_data(data):
    limits = [(-1, 1)] * 6  # Same limit for x, y, z, a, b, c
    return [max(min(val, lim[1]), lim[0]) for val, lim in zip(data, limits)]

async def send_periodic_pings(websocket):
    while True:
        await asyncio.sleep(10)  # Send a ping every 10 seconds
        try:
            await websocket.ping()
        except websockets.exceptions.ConnectionClosed:
            break

async def handler(websocket, path):
    logging.info("Client connected")
    ping_task = asyncio.create_task(send_periodic_pings(websocket))
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                data_from_socket = data.get("data")
                if isinstance(data_from_socket, list) and len(data_from_socket) == 6 and all(isinstance(num, (int, float)) for num in data_from_socket):
                    limited_data = limit_data(data_from_socket)
                    ros_node.publish_data(limited_data)  # Publish data using ROS 2
                    logging.info(f"Published data: {limited_data}")
                else:
                    logging.warning("Invalid data received.")
            except json.JSONDecodeError:
                logging.error("Invalid JSON message received.")
    except websockets.exceptions.ConnectionClosedError as e:
        logging.warning(f"Connection closed unexpectedly: {e}")
    finally:
        ping_task.cancel()  # Cancel ping task when the connection is closed
        logging.info("Client disconnected")

def run_ros_node():
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

async def main_asyncio():
    async with websockets.serve(handler, "localhost", 6789):
        logging.info("WebSocket Server running on 0.0.0.0")
        await asyncio.Future()  # Runs forever

def main():
    # Run ROS node
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()

    # Run asyncio event loop in a separate thread
    asyncio_thread = threading.Thread(target=lambda: asyncio.run(main_asyncio()), daemon=True)
    asyncio_thread.start()

    ros_thread.join()
    asyncio_thread.join()

if __name__ == '__main__':
    main()
