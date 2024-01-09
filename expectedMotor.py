import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def doIK():
    global ik
    old_position= ik.copy()
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z", initial_position=old_position)
    
def move(x,y,z, x2, y2, z2):
    global target_position
    target_position = [x,y,z]
    global target_orientation
    target_orientation = [x2, y2, z2]
    doIK()

class MyNode(Node):
    def __init__(self):
        super().__init__("Expected_Motor_Angles_publisher")
        self.publisher_ = self.create_publisher(String, 'expected_motor_angles', 10)
        self.timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        # message format: "X X X..." for each motor angle. Is ended with a new line
        msg = String()
        msg.data = str(list(map(lambda r:math.degrees(r), ik.tolist())))
        msg.data += "\n"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    global my_chain
    my_chain = ikpy.chain.Chain.from_urdf_file("simon-v2-0.urdf",active_links_mask=[False, False, True, True, True, True, True, False, False])
    move(0, 0, 0.58, -1, 0, 0)

    pub = MyNode()
    rclpy.spin(pub)

    rclpy.shutdown()

if __name__ == '__main__':
    main() 
