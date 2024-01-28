#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
from Phidget22.LogLevel import *
import traceback
import time
# Base is connected to port 1, Elbow is connected to port 2, Wrist is connected to port 3

encoder_positions = [0,0,1] # pos#0 is base, pos#1 is elbow, pos#2 is wrist

def onPositionChange(encoder, positionChange, timeChange, indexTriggered, **kwargs):
    port = encoder.getHubPort()
    pos = encoder.getPosition()
    encoder_positions[port - 1] = pos
    print(f"list = {encoder_positions}")

def main(args=None):
    encoders = []

    try:
        for hub_port in range(1, 4):
            encoder = Encoder()
            encoder.setHubPort(hub_port)
            encoder.openWaitForAttachment(1000)
            encoder.setOnPositionChangeHandler(onPositionChange) 
            encoders.append(encoder)
            print(f"Encoder {encoder.getHubPort()} Initial Position: {encoder.getPosition()}")

        try:
            input("Press Enter to Stop\n")
        except (Exception, KeyboardInterrupt):
            pass

    except PhidgetException as ex:
        traceback.print_exc()
        print("")   
        print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)

    finally:
        for encoder in encoders:
            encoder.close()

if __name__ == "__main__":
    main()
