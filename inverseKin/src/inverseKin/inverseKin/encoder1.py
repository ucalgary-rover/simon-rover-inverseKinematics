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

class SingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if 'index' not in kwargs:
            raise ValueError("A 'index' keyword argument is required.")
        index = kwargs['index']

        if index not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[index] = instance
        return cls._instances[index]
       

class Singleton(metaclass=SingletonMeta):
    #creating the instance of the encoder
    def __init__(self, index):
        self.index = index
        self.encoder = Encoder()
        
#publish encoder values to the 'actual_motor_angles' topic
def publish_encoder_values(position):
    msg = Float32MultiArray()
    msg.data = [float(val) for val in position]
    publisher.publish(msg)
    node.get_logger().info(f'Publishing encoder value: {position}')

def main(args=None):
    global node, publisher
    encoders = []

    #this try catch loop should be able to capture 
    try:
        # Initialize ROS 2 node
        rclpy.init(args=args)
        node = rclpy.create_node('encoder_node')
        publisher = node.create_publisher(Float32MultiArray, 'actual_motor_angles', 10)

        # going through the ports 1 2 and 3
        for hub_port in range(1, 4):
            #keda keda u were overwritting
            enc = Singleton(index = hub_port) #creating an instance, if a hub_port with that value was created it wont reopen it 

            try:
                #each call is based on the single instance of the encoder
                enc.encoder.setHubPort(hub_port)
                enc.encoder.openWaitForAttachment(1000)
                enc.encoder.setEnabled(True)
                enc.encoder.setOnPositionChangeHandler(onPositionChange)
                enc.encoder.setOnErrorHandler(lambda code, desc, idx=hub_port: onError(enc.encoder, code, desc, idx))
                encoders.append(enc.encoder)
                print(f"Encoder {enc.encoder.getHubPort()} Initial Position: {enc.encoder.getPosition()}")

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
