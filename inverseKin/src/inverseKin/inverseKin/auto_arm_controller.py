# Authour: Aryan Karadia

# based on the code from Joshua Liu


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from Phidget22.Phidget import *
from Phidget22.PhidgetException import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.RCServo import *
from Phidget22.Devices.Encoder import *
import time
import traceback


VHubSerial_motors = 697103
VHubSerial_servo = 697066


def onAttach_motor(self): 
    print(" {0} attached!".format(self.getHubPort())) 
    motorsAttached[self.getHubPort()] = True

def onAttach_encoder(self): 
    print("Encoder {0} attached!".format(self.getHubPort())) 

def onDetach(self): 
    print("A motor detached!")

def onError(self,code, description): 
    print("Code: " + ErrorEventCode.getName(code)) 
    print("Description: " + str(description)) 
    print("----------")
    
# Initialize a list for encoder positions
encoder_positions = [0, 0, 0]

# encoder position change, which also publishes it to the topic
def onPositionChange(encoder, positionChange, timeChange, indexTriggered, **kwargs):
    port = encoder.getHubPort()
    pos = encoder.getPosition()
    encoder_positions[port - 1] = pos
    # print(f"list = {encoder_positions}")

#handling errors, including checking for not attached properly and printing a message and seeing value to 0
def onError(encoder, code, description, indexTriggered):
    if code == ErrorCode.EPHIDGET_NOTATTACHED:
        print(f"Encoder {indexTriggered} not attached properly. Setting position to 0.")
        encoder_positions[indexTriggered - 1] = 0
    else:
        print(f"Error in Encoder {indexTriggered}:")
        print(f"Code: {ErrorEventCode.getName(code)}")
        print(f"Description: {description}")
    
# init motors
def initialize_motors(): 
    global motors, motorsInfo
    for i in range(len(motors)): 
        motors[i].setDeviceSerialNumber(VHubSerial_motors) 
        motors[i].setHubPort(i) 
        motors[i].setOnAttachHandler(onAttach_motor) 
        motors[i].setOnDetachHandler(onDetach) 
        motors[i].setOnErrorHandler(onError)
        try: 
            motors[i].openWaitForAttachment(1000) # If having motor connection timout issues, increase this number 
        except: 
            print(" " + str(i) + " not attached")
        if (motors[i].getAttached() == True): 
            motors[i].setControlMode(StepperControlMode.CONTROL_MODE_RUN) 
            motors[i].setCurrentLimit(motorsInfo[i][0]) 
            motors[i].setHoldingCurrentLimit(motorsInfo[i][1])
            motors[i].setRescaleFactor((1/16) * 1.8 * (1/motorsInfo[i][2]) * (1/motorsInfo[i][3])) # (1/16) * Step angle * (1/Gearbox ratio) * (1/Gear ratio) p
            motors[i].setAcceleration(motorsInfo[i][4])
            motors[i].setVelocityLimit(0) 
            motors[i].setEngaged(True) 
            motors[i].setDataInterval(motors[i].getMinDataInterval())

class RoverArmController(Node):
    def __init__(self):
        super().__init__('rover_arm_controller')
        self.tolerance = 5.0
        self.expected_motor_angles = None
        self.motors_powered = False
        self.smoothing = 0.005
        self.clawSpeeds = [100, 80]
        self.isClawMoving = False

        # Subscribers for actual and expected motor angles
        self.subscription_expected = self.create_subscription(Float32MultiArray, 'expected_motor_angles', self.expected_callback, 10)
        # index #: Motor, 0: base motor, 1: elbow, 2: wrist main
        self.testing_motor_angles = [self.expected_motor_angles[0], self.expected_motor_angles[2], self.expected_motor_angles[3]]

        print("\nSuccessfully initialized!\n")
        self.adjust_motors()

    def expected_callback(self, data):
        # Callback when expected motor angles are received
        self.expected_motor_angles = data.data
        self.get_logger().info(f"expected: {self.expected_motor_angles}")
        self.adjust_motors()

    def adjust_motors(self):
        # Adjust motors based on actual and expected 
        self.get_logger().info(f"expected: {self.expected_motor_angles}")
        self.get_logger().info(f"actual: {[encoders[i].getPosition() for i in range(len(encoders))]}")
        if self.testing_motor_angles is not None:
            # ZIPPING WITH TESTING ENCODER VALUES CHANGE WHEN ALL MOTORS HAVE ENCODERS
            for encoder, expected_angle, i in zip(encoders, self.testing_motor_angles, range(len(encoders))):
                print(f"diff: {abs(encoder.getPosition() - expected_angle)}")
                print("expected angle: ", expected_angle)
                print("actual angle: ", encoder.getPosition())
                print("i: ", i)
                if abs(encoder.getPosition() - expected_angle) != 0:
                    print("1")
                    self.power_motors(motors[i], i)

    def power_motors(self, motor, motorID):
        try:
            # Set motor properties for movement
            motor.setAcceleration(motorsInfo[motorID+1][4])
            motor.setVelocityLimit(motorsInfo[motorID+1][5])
            # motor.setTargetPosition(self.expected_motor_angles[motorID])
            
            print("TEST", self.testing_motor_angles[motorID])
            motor.setEngaged(True)
            self.motors_powered = True
            
                
            # Wait until the motor reaches the expected angle
            # TODO: WHEN ALL MOTORS HAVE ENCODERS CHANGE "self.testing_motor_angles" TO "self.expected_motor_angles"
            while abs(encoder_positions[motorID] - self.testing_motor_angles[motorID]) > self.tolerance:
                # this only works if encoder is moving in negative direction
                if encoder_positions[motorID] < self.testing_motor_angles[motorID]:
                    break
                print(f"target: {self.testing_motor_angles[motorID]}")
                print(f"cur Pos: {encoder_positions[motorID]}")
                print(f"diff: {abs(encoder_positions[motorID] - self.testing_motor_angles[motorID])}")
                time.sleep(self.smoothing)

            # Disengage the motor
            motor.setEngaged(False)
        except PhidgetException as ex:
            self.get_logger().error(f"PhidgetException 3 {str(ex.code)} ({ex.description}): {ex.details}\n\n MotorID: {motorID}")
        finally:
            self.stop_motors(motorID)
            motor.close()

    '''
            PORT # : MOTOR
            PORT 0 : BASE MOTOR
            PORT 1 : SHOULDER
            PORT 2 : ELBOW
            PORT 3 : WRIST MAIN
            PORT 4 : WRIST 1 (just support)
            PORT 5 : WRIST 2 (just support)
            
            
            ENCODER PORT # : MOTOR
            ENCODER PORT 1 : BASE MOTOR
            ENCODER PORT 2 : ELBOW
            ENCODER PORT 3 : WRIST
        '''
    
    def stop_motors(self, motorID):
        motor = motors[motorID]
        try:
            # Stop the motor gradually
            if self.isClawMoving:
                claw.setTargetPosition(90)
                claw.setEngaged(False)
                self.isClawMoving = False

            lim = motor.getVelocityLimit()
            for i in range(1, 5):
                motor.setVelocityLimit(lim * i / 4)
                time.sleep(self.smoothing / 4)

            motor.setVelocityLimit(0)
        except PhidgetException as ex:
            self.get_logger().error(f"PhidgetException 2 {str(ex.code)} ({ex.description}): {ex.details}")

encoders = []

def main(args=None):
    try:
        rclpy.init(args=args)
        # Motor and device initialization
        global motors, motorsInfo, claw, motorsAttached
        global baseMotor, shoulderMotor, elbowMotor, wristMotor, wrist1Motor, wrist2Motor
        global node, publisher, encoders
        # Create instances for motors
        baseMotor = Stepper()
        baseInfo = [1.68, 1.68, 100, 1, 0.6, 10]
        shoulderMotor = Stepper()
        shoulderInfo = [3, 1.68, 100, 1, 5, 5]
        elbowMotor = Stepper()
        ElbowInfo = [1.68, 1.68, 77, 1, 10, 10]
        wristMotor = Stepper()
        WristInfo = [1.68, 1.68, 51, 1, 15, 10]
        wrist1Motor = Stepper()
        Wrist1Info = [.67, .67, 100, 1, 15, 10]
        wrist2Motor = Stepper()
        wrist2Info = [.67, .67, 100, 1, 15, 10]
        # Arrays for motors and their information
        motors = [baseMotor, shoulderMotor, elbowMotor, wristMotor, wrist1Motor, wrist2Motor]
        motorsInfo = [baseInfo, shoulderInfo, ElbowInfo, WristInfo, Wrist1Info, wrist2Info]
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
        initialize_motors()
        armController = RoverArmController()
        
        # try:
        #     input("Press Enter to Stop\n")
        # except (Exception, KeyboardInterrupt):
        #     pass
        
        rclpy.spin(armController)
    except PhidgetException as ex:
		# Handle Phidget exceptions
        traceback.print_exc()
        print("")
        print("PhidgetException 1 " + str(ex.code) + " (" + ex.description + "): " + ex.details)
    finally:
		# Close Phidget encoders and shut down ROS 2 node
        for encoder in encoders:
            encoder.close()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
    
