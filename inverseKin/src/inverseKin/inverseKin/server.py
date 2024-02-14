import asyncio
import websockets
import json
import logging
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

logging.basicConfig(level=logging.INFO)

rclpy.init()

class MyROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')
        self.publisher = self.create_publisher(Int32MultiArray, 'goal_position', 10)

    def publish_data(self, data):
        int_data = [int(val * 100) for val in data]
        msg = Int32MultiArray()
        msg.data = int_data
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

ros_node = MyROSNode()

async def handler(websocket, path):
    logging.info("Client connected")
    try:
        async for message in websocket:
            data = json.loads(message)
            if 'data' in data and len(data['data']) == 6:
                ros_node.publish_data(data['data'])
                logging.info(f"Published data: {data['data']}")
            else:
                logging.warning("Invalid data format received.")
    except Exception as e:
        logging.error(f"Error in WebSocket connection: {e}")
    finally:
        logging.info("Client disconnected")

async def main():
    async with websockets.serve(handler, "localhost", 6789):
        logging.info("WebSocket Server running on localhost:6789")
        await asyncio.Future()

def run_ros_node():
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    asyncio.run(main())
