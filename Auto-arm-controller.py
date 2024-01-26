import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


"""
TODO: implement motor initialization
TODO: Implement motor power and stop functions using legacy code. MotorID is just index of motor in array. Need to continously power motors until they reach the expected angle (within tolerance) and then stop them. also need to make sure that the motors are only powered when they are not already powered. (use a boolean array to keep track of which motors are powered)
TODO: Implement motor power and stop functions using legacy code. MotorID is just index of motor in array. 
Need to continously power motors until they reach the expected angle (within tolerance) and then stop them. 
also need to make sure that the motors are only powered when they are not already powered. (use a boolean array to keep track of which motors are powered)
"""

global motors, motorsInfo, baseMotor, shoulderMotor, elbowMotor, wristMotor, wrist1Motor, wrist2Motor
global baseInitialPos, shoulderInitialPos, ElbowInitialPos, WristInitialPos, Wrist1InitialPos, wrist2InitialPos
global claw, isClawAttached

baseMotor = Stepper() 
baseInfo = [1.68, 1.68, 100, 1, 10, 10]
baseInitialPos = 0

shoulderMotor = Stepper() 
shoulderInfo = [3, 1.68, 100, 1, 5, 5]
shoulderInitialPos = 0

elbowMotor = Stepper() 
ElbowInfo = [1.68, 1.68, 77, 1, 10, 10]
ElbowInitialPos = 0

wristMotor = Stepper() 
WristInfo = [1.68, 1.68, 51, 1, 15, 10]
WristInitialPos = 0

wrist1Motor = Stepper() 
Wrist1Info = [.67, .67, 100, 1, 15, 10]
Wrist1InitialPos = 0

wrist2Motor = Stepper() 
wrist2Info = [.67, .67, 100, 1, 15, 10]
wrist2InitialPos = 0

motors = [baseMotor, shoulderMotor, elbowMotor, wristMotor, wrist1Motor, wrist2Motor] 
motorsInfo = [baseInfo, shoulderInfo, ElbowInfo, WristInfo, Wrist1Info, wrist2Info 

class RoverArmController(Node):
    def __init__(self):
        # Can Change This Tolerance
        self.tolerance = 0.01

        # Init Node name and Subscribers
        super().__init__('rover_arm_controller')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'actual_motor_angles',
            self.actual_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'expected_motor_angles',
            self.expected_callback,
            10
        )

    def actual_callback(self, data):
        # Sets the actual motor angles to the data from the actual_motor_angles topic (Current Motor Angles)
        self.actual_motor_angles = data.data
        self.adjust_motors()

    def expected_callback(self, data):
        # Sets the expected motor angles to the data from the expected_motor_angles topic (Target Motor Angles)
        self.expected_motor_angles = data.data
        self.adjust_motors()

    def adjust_motors(self):
        # Only adjusts motors if both actual and expected motor angles are set
        if self.actual_motor_angles is not None and self.expected_motor_angles is not None:
            # For every motor, check if the difference between actual and expected angles is greater than the tolerance
            for actual_angle, expected_angle, i in zip(self.actual_motor_angles, self.expected_motor_angles):
                if abs(actual_angle - expected_angle) > self.tolerance:
                    self.power_motors(i)
                    return
            if self.motors_powered:
                self.stop_motors(i)
                self.motors_powered = False

    def power_motors(self, motorID):
        # Using motorID add power to motor to move it towards the expected angle
        pass

    def stop_motors(self, motorID):
        # Implement code to stop supplying power to the motors using the motorID
        pass

if __name__ == "__main__":
    controller = RoverArmController()
    rclpy.spin(controller)