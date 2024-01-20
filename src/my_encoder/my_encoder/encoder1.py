#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
from Phidget22.LogLevel import *
import traceback
import time

# Define global encoder serial number and ports

BASE_SERIAL_NUMBER = 20210910594
BASE_HUB_PORT = 0

ELBOW_SERIAL_NUMBER = 20210910985
ELBOW_HUB_PORT = 2

WRIST_SERIAL_NUMBER = 20210911894
WRIST_HUB_PORT = 1

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        #topic
        self.publisher_ = self.create_publisher(Float32MultiArray, 'actual_motor_angles', 10)

        self.encoders = [
            self.init_encoder(BASE_SERIAL_NUMBER, BASE_HUB_PORT, 0),
            self.init_encoder(ELBOW_SERIAL_NUMBER, ELBOW_HUB_PORT, 1),
            self.init_encoder(WRIST_SERIAL_NUMBER, WRIST_HUB_PORT, 2)
        ]
        self.position_list = []

        # Create a timer
        self.timer = self.create_timer(0.25, self.timer_callback)

    def init_encoder(self, serial_number, hub_port, index):
        encoder = Encoder()
        encoder.setDeviceSerialNumber(serial_number)
        encoder.setHubPort(hub_port)
        encoder.setOnPositionChangeHandler(lambda posChange, timeChange, idx=index: self.onPositionChange(posChange, timeChange, idx))
        encoder.setOnAttachHandler(lambda idx=index: self.onAttach(idx))
        encoder.setOnDetachHandler(lambda idx=index: self.onDetach(idx))
        encoder.setOnErrorHandler(lambda code, desc, idx=index: self.onError(code, desc, idx))
        encoder.openWaitForAttachment(5000)
        return encoder

    def onPositionChange(self, positionChange, timeChange, indexTriggered):
        print("PositionChange: " + str(positionChange))
        print("TimeChange: " + str(timeChange))
        print("IndexTriggered: " + str(indexTriggered))

        current_position = self.encoders[indexTriggered].getPosition()
        self.position_list.append(current_position)
        self.publish_encoder_values(current_position)

        print("getPosition: " + str(current_position))
        print("----------")

    def onAttach(self, indexTriggered):
        print(f"Attached Encoder {indexTriggered}")

    def onDetach(self, indexTriggered):
        print(f"Detached Encoder {indexTriggered}")

    def onError(self, code, description, indexTriggered):
        print(f"Error in Encoder {indexTriggered}:")
        print(f"Code: {ErrorEventCode.getName(code)}")
        print(f"Description: {description}")

    def timer_callback(self):
        # Publish the current position values
        self.publish_encoder_values(self.position_list)
        # Clear the position list for the next iteration
        self.position_list = []

    def publish_encoder_values(self, position):
        msg = Float32MultiArray()
        msg.data = position
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing encoder value: %s' % position)

    # def openAndWaitForAttachment(self):
    #     self.encoder.openWaitForAttachment(5000)

    def close(self):
        self.encoder.close()

def main(args=None):
    rclpy.init(args=args)
    # Create an Encoder instance
    encoder_node = EncoderNode()

    try:
        #Log.enable(LogLevel.PHIDGET_LOG_INFO, "phidgetlog.log")
        #encoder_node.open_and_wait_for_attachment()

        try:
            input("Press Enter to Stop\n")
        except (Exception, KeyboardInterrupt):
            pass

    except PhidgetException as ex:
        traceback.print_exc()
        print("")
        print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)

    finally:
        encoder_node.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
