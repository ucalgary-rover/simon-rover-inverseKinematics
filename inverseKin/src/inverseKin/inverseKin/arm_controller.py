from Phidget22.Phidget import *
from Phidget22.PhidgetException import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.RCServo import *
from Phidget22.Devices.Encoder import *
# from encoderPython import get_encoder_positions
import traceback

VHubSerial_motors = 697103
VHubSerial_servo = 697066

encoder_positions = [0, 0, 0]

# TODO: get actual goal value from inverse kinematics
global goal
goal = [0, 0, 0, 0, 0, 0]


def onAttach_motor(self):
    print(" {0} attached!".format(self.getHubPort()))
    motorsAttached[self.getHubPort()] = True


def onAttach_encoder(self):
    print("Encoder {0} attached!".format(self.getHubPort()))


def onDetach(self):
    print("A motor detached!")


def onError(self, code, description):
    print("Code: " + ErrorEventCode.getName(code))
    print("Description: " + str(description))
    print("----------")

def onPositionChange(encoder, positionChange, timeChange, indexTriggered, **kwargs):
    port = encoder.getHubPort()
    pos = encoder.getPosition()
    encoder_positions[port - 1] = pos
    # print(f"list = {encoder_positions}")


def initialize_motors():
    global motors, motorsInfo
    
    for i in range(len(motors)):
        motors[i].setDeviceSerialNumber(VHubSerial_motors)
        motors[i].setHubPort(i)
        motors[i].setOnAttachHandler(onAttach_motor)
        motors[i].setOnDetachHandler(onDetach)
        motors[i].setOnErrorHandler(onError)
        try:
            motors[i].openWaitForAttachment(
                1000
            )  # If having motor connection timout issues, increase this number
        except:
            print(" " + str(i) + " not attached")
        if motors[i].getAttached() == True:
            motors[i].setControlMode(StepperControlMode.CONTROL_MODE_RUN)
            motors[i].setCurrentLimit(motorsInfo[i][0])
            motors[i].setHoldingCurrentLimit(motorsInfo[i][1])
            motors[i].setRescaleFactor(
                (1 / 16) * 1.8 * (1 / motorsInfo[i][2]) * (1 / motorsInfo[i][3])
            )  # (1/16) * Step angle * (1/Gearbox ratio) * (1/Gear ratio) p
            motors[i].setAcceleration(motorsInfo[i][4])
            motors[i].setVelocityLimit(0)
            motors[i].setEngaged(True)
            motors[i].setDataInterval(motors[i].getMinDataInterval())
            
    for hub_port in range(0,4):
        encoder = Encoder()
        encoder.setHubPort(hub_port)
        encoder.openWaitForAttachment(1000)
        encoder.setOnPositionChangeHandler(onPositionChange) 
        encoders.append(encoder)

    

def main(args=None):
    try:
        # Initialize motors
        # Motor and device initialization
        global motors, motorsInfo, claw, motorsAttached
        global baseMotor, shoulderMotor, elbowMotor, wristMotor, wrist1Motor, wrist2Motor
        # Create instances for motors
        baseMotor = Stepper()
        baseInfo = [1.68, 1.68, 100, 1, 10, 10]
        shoulderMotor = Stepper()
        shoulderInfo = [3, 1.68, 100, 1, 5, 5]
        elbowMotor = Stepper()
        ElbowInfo = [1.68, 1.68, 77, 1, 10, 10]
        wristMotor = Stepper()
        WristInfo = [1.68, 1.68, 51, 1, 15, 10]
        wrist1Motor = Stepper()
        Wrist1Info = [0.67, 0.67, 100, 1, 15, 10]
        wrist2Motor = Stepper()
        wrist2Info = [0.67, 0.67, 100, 1, 15, 10]
        # Arrays for motors and their information
        motors = [
            baseMotor,
            shoulderMotor,
            elbowMotor,
            wristMotor,
            wrist1Motor,
            wrist2Motor,
        ]
        motorsInfo = [
            baseInfo,
            shoulderInfo,
            ElbowInfo,
            WristInfo,
            Wrist1Info,
            wrist2Info,
        ]
        motorsAttached = [False, False, False, False, False, False]
        # Initialize RCServo (claw)
        claw = RCServo()
        claw.setChannel(0)
        claw.setHubPort(0)
        claw.setDeviceSerialNumber(VHubSerial_servo)
        claw.openWaitForAttachment(1000)
        claw.setVoltage(RCServoVoltage.RCSERVO_VOLTAGE_7_4V)
        claw.setMinPulseWidth(500)
        claw.setMaxPulseWidth(2500)

        # Initialize motors
        initialize_motors()

        # get encoder value
        global encoders
        encoders = get_encoder_positions()

        # for every motor, while the encoder value is not equal to the goal value, move the motor
        for i in range(len(motors)):
            while encoders[i] != goal[i]:
                motors[i].setVelocityLimit(motorsInfo[i][5])
                encoders = get_encoder_positions()
                print(f"Motor {i} encoder value: {encoders[i]} Goal: {goal[i]}")

        print("Goal reached")

        # Close all motors
        for i in range(len(motors)):
            motors[i].setEngaged(False)

        print("Motors disengaged")

    except PhidgetException as ex:
        # Handle Phidget exceptions
        traceback.print_exc()
        print("")
        print(
            "PhidgetException "
            + str(ex.code)
            + " ("
            + ex.description
            + "): "
            + ex.details
        )


if __name__ == "__main__":
    main()
