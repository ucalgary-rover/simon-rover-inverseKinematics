#!/usr/bin/env python3
import ikpy.chain
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

ik = None  # Define ik variable at a global level

def doIK():
    global ik
    old_position = ik.copy() if ik is not None else None # Handling the initial case
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z")
    
def move(x, y, z, x2, y2, z2):
    global target_position, target_orientation
    target_position = [x, y, z]
    target_orientation = [x2, y2, z2]
    doIK()

class MyNode(Node):
    def __init__(self):
        super().__init__("Expected_Motor_Angles_publisher")
        self.publisher_ = self.create_publisher(Float32MultiArray, 'expected_motor_angles', 10)
        self.timer = self.create_timer(1, self.timer_callback)

        #subscribe to other node
        # Have to change msg_type 
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'goal_position',
            self.calc_pos,
            10
        )

    def calc_pos(self, msg):
        self.get_logger().info(f" Recieved {msg.data}")
        # msg.data is tuple
        pos = msg.data
        move(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])

    def timer_callback(self):
        global ik
        
        offset = [0.0, 6.746014595031738, -20.746801376342773, -1.0599719285964966, 158.4469451904297, -3.6518921852111816, 0.0, 0.0]
        if ik is not None:
            self.get_logger().info("target pos: %s" % target_position)
            msg = Float32MultiArray()
            
            ik_to_degrees = list(map(lambda r: math.degrees(r), ik.tolist()))
            
            for i in range(len(ik_to_degrees)):
                ik_to_degrees[i] = ik_to_degrees[i] - offset[i]
                ik_to_degrees[i] = float(int(ik_to_degrees[i]))
                
                
            msg.data = ik_to_degrees
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

            #testing forward kinematics
            computed_pos = my_chain.forward_kinematics(ik)
            self.get_logger().info("Computed position : %s" % ['%.2f' % elem for elem in computed_pos[:3, 3]])

def main(args=None):
    rclpy.init(args=args)
    global my_chain
    my_chain = ikpy.chain.Chain.from_urdf_file("/home/symasc/simon-rover-inverseKinematics/simon-v2-0.urdf", active_links_mask=[False, True, True, True, True, True, False, False])

    pub = MyNode()
    rclpy.spin(pub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
