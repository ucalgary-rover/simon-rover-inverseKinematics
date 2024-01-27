import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from Phidget22.Phidget import *
from Phidget22.PhidgetException import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.RCServo import *
import time
import traceback

class RoverArmController(Node):
    def __init__(self):
        super().__init__('rover_arm_controller')
        self.tolerance = 0.01
        self.actual_motor_angles = None
        self.expected_motor_angles = None
        self.motors_powered = False
        self.smoothing = 0.005
        self.clawSpeeds = [100, 80]
        self.isClawMoving = False

        # Subscribers for actual and expected motor angles
        self.subscription_actual = self.create_subscription(Float32MultiArray, 'actual_motor_angles', self.actual_callback, 10)
        self.subscription_expected = self.create_subscription(Float32MultiArray, 'expected_motor_angles', self.expected_callback, 10)

        self.get_logger().info("Initializing Motors...")
        try:
            self.initialize_motors()
        except PhidgetException as ex:
            self.get_logger().error(f"PhidgetException {str(ex.code)} ({ex.description}): {ex.details}")
        print("\nSuccessfully initialized!\n")

    def actual_callback(self, data):
        # Callback when actual motor angles are received
        self.actual_motor_angles = data.data
        self.adjust_motors()

    def expected_callback(self, data):
        # Callback when expected motor angles are received
        self.expected_motor_angles = data.data
        self.adjust_motors()

    def adjust_motors(self):
        # Adjust motors based on actual and expected angles
        if self.actual_motor_angles is not None and self.expected_motor_angles is not None:
            for actual_angle, expected_angle, i in zip(self.actual_motor_angles, self.expected_motor_angles, range(len(self.actual_motor_angles))):
                if abs(actual_angle - expected_angle) > self.tolerance:
                    self.power_motors(motors[i], i)

    def power_motors(self, motor, motorID):
        try:
            # Set motor properties for movement
            motor.setAcceleration(motorsInfo[motorID][4])
            motor.setVelocityLimit(motorsInfo[motorID][5])
            motor.setTargetPosition(self.expected_motor_angles[motorID])
            motor.setEngaged(True)
            self.motors_powered = True

            # Wait until the motor reaches the expected angle
            while abs(motor.getPosition() - self.expected_motor_angles[motorID]) > self.tolerance:
                time.sleep(self.smoothing)

            # Disengage the motor
            motor.setEngaged(False)
        except PhidgetException as ex:
            self.get_logger().error(f"PhidgetException {str(ex.code)} ({ex.description}): {ex.details}")
        finally:
            motor.close()
            self.stop_motors(motorID)

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
            self.get_logger().error(f"PhidgetException {str(ex.code)} ({ex.description}): {ex.details}")

if __name__ == "__main__":
    try:
        # Motor and device initialization
        global motors, motorsInfo, claw, VHubSerial_servo, motorsAttached
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
        Wrist1Info = [.67, .67, 100, 1, 15, 10]

        wrist2Motor = Stepper()
        wrist2Info = [.67, .67, 100, 1, 15, 10]

        # Arrays for motors and their information
        motors = [baseMotor, shoulderMotor, elbowMotor, wristMotor, wrist1Motor, wrist2Motor]
        motorsInfo = [baseInfo, shoulderInfo, ElbowInfo, WristInfo, Wrist1Info, wrist2Info]

        VHubSerial_servo = 697066
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

        rclpy.spin(RoverArmController())

    except PhidgetException as ex:
        traceback.print_exc()
        print(f"\nPhidgetException {str(ex.code)} ({ex.description}): {ex.details}")
        print("Successfully quit program.\n\nGoodbye!\n")