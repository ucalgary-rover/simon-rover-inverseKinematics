#!/usr/bin/env python3
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
from Phidget22.LogLevel import *
import traceback
# Base is connected to port 1, Elbow is connected to port 2, Wrist is connected to port 3

global encoder_positions # pos#0 is base, pos#1 is elbow, pos#2 is wrist
encoder_positions = [0, 0, 0] 

def onPositionChange(encoder, positionChange, timeChange, indexTriggered, **kwargs):
    port = encoder.getHubPort()
    pos = encoder.getPosition()
    encoder_positions[port - 1] = pos
    print(f"list = {encoder_positions}")


def get_encoder_positions():
    return encoder_positions

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
        
        
def main(args=None):
    encoders = []
    try:
        for hub_port in range(1, 4):
            enc = Singleton(index = hub_port)
        
            enc.encoder.setHubPort(hub_port)
            enc.encoder.openWaitForAttachment(1000)
            enc.encoder.setOnPositionChangeHandler(onPositionChange) 
            encoders.append(enc.encoder)
            print(f"Encoder {enc.encoder.getHubPort()} Initial Position: {enc.encoder.getPosition()}")

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
