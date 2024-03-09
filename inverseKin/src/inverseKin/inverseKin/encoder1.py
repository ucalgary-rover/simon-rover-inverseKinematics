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

# Initialize a list for encoder positions
encoder_positions = [0, 0, 0]

# encoder position change, which also publishes it to the topic
def onPositionChange(encoder, positionChange, timeChange, indexTriggered, **kwargs):
    port = encoder.getHubPort()
    pos = encoder.getPosition()
    encoder_positions[port - 1] = pos
    print(f"list = {encoder_positions}")
    publish_encoder_values(encoder_positions)

#handling errors, including checking for not attached properly and printing a message and seeing value to 0
def onError(encoder, code, description, indexTriggered):
    if code == ErrorCode.EPHIDGET_NOTATTACHED:
        print(f"Encoder {indexTriggered} not attached properly. Setting position to 0.")
        encoder_positions[indexTriggered - 1] = 0
    else:
        print(f"Error in Encoder {indexTriggered}:")
        print(f"Code: {ErrorEventCode.getName(code)}")
        print(f"Description: {description}")

#publish encoder values to the 'actual_motor_angles' topic
def publish_encoder_values(position):
    msg = Float32MultiArray()
    msg.data = [float(val) for val in position]
    publisher.publish(msg)
    node.get_logger().info(f'Publishing encoder value: {position}')

def main(args=None):
    global node, publisher
    encoders = []

    try:
        # Initialize ROS 2 node
        rclpy.init(args=args)
        node = rclpy.create_node('encoder_node')
        publisher = node.create_publisher(Float32MultiArray, 'actual_motor_angles', 10)

        # going through the ports 1 2 and 3
        for hub_port in range(1, 4):
            encoder = Encoder()

            try:
                encoder.setHubPort(hub_port)
                encoder.openWaitForAttachment(1000)
                encoder.setEnabled(True)
                encoder.setOnPositionChangeHandler(onPositionChange)
                encoder.setOnErrorHandler(lambda code, desc, idx=hub_port: onError(encoder, code, desc, idx))
                encoders.append(encoder)
                print(f"Encoder {encoder.getHubPort()} Initial Position: {encoder.getPosition()}")

            except PhidgetException as ex:
                # Handle Phidget exceptions for this encoder
                print(f"Error initializing Encoder {hub_port}: {ex.details}")

        try:
            input("Press Enter to Stop\n")
        except (Exception, KeyboardInterrupt):
            pass

    except PhidgetException as ex:
        # Handle Phidget exceptions
        traceback.print_exc()
        print("")
        print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)

    finally:
        # Close Phidget encoders and shut down ROS 2 node
        for encoder in encoders:
            encoder.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
