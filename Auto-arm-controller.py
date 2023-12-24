# Joshua Liu
# 
# Possible keys for controls: a-z, 0-9
# 
# VINT Hub 1 wiring (6 Steppers)
#   Port 0: Base
#   Port 1: Shoulder
#   Port 2: Elbow
#   Port 3: Wrist
#   Port 4: Wrist1
#   Port 5: Wrist2
# Wiring the Claw
#   VINT Hub 2 Port 0
#   Servo Controller Port 0
# This version only controls the arm


from Phidget22.Phidget import *
from Phidget22.PhidgetException import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.RCServo import *
from pynput import keyboard
from pynput.keyboard import Key, Listener
import traceback
import time

baseIndex = 0
baseControls = ['d', 'a'] # change this to change the controls of the Base motor

shoulderIndex = 1
shoulderControls = ['w', 's'] # change this to change the controls of the Shoulder motor

elbowIndex = 2
elbowControls = ['r', 'f'] # change this to change the controls of the Elbow motor

wristIndex = 3
wristControls = ['e', 'q'] # change this to change the controls of the Wrist motor

wrist1Index = 4

wrist2Index = 5

pulleyTwistControls = ['u', 'o']
pulleyLiftControls = ['i', 'k']

isClawAttached = False
clawSpeeds = [100, 80]
clawControls = ['j', 'l'] # change this to change the controls of the Claw
isClawMoving = False

stopFlag = False

VHubSerial_motors = 697103
VHubSerial_servo = 697066

smoothing = 0.005 # Time delay (in seconds) for a motor to change from Attached to stopped
motorsAttached = [False, False, False, False, False, False]
encoders = []

def on_press(key):
    global isClawMoving
    try: 
        if key.char == baseControls[0] and motorsAttached[baseIndex] == True: 
            if not baseMotor.getIsMoving(): 
                baseMotor.setVelocityLimit(motorsInfo[baseIndex][5]) 
        elif key.char == baseControls[1] and motorsAttached[baseIndex] == True: 
            if not baseMotor.getIsMoving(): 
                baseMotor.setVelocityLimit(-motorsInfo[baseIndex][5]) 
        if key.char == shoulderControls[0] and motorsAttached[shoulderIndex] == True: 
            if not shoulderMotor.getIsMoving(): 
                shoulderMotor.setVelocityLimit(motorsInfo[shoulderIndex][5]) 
        elif key.char == shoulderControls[1] and motorsAttached[shoulderIndex] == True: 
            if not shoulderMotor.getIsMoving(): 
                shoulderMotor.setVelocityLimit(-motorsInfo[shoulderIndex][5]) 
        if key.char == elbowControls[0] and motorsAttached[elbowIndex] == True: 
            if not elbowMotor.getIsMoving(): 
                elbowMotor.setVelocityLimit(motorsInfo[elbowIndex][5]) 
        elif key.char == elbowControls[1] and motorsAttached[elbowIndex] == True: 
            if not elbowMotor.getIsMoving(): 
                elbowMotor.setVelocityLimit(-motorsInfo[elbowIndex][5]) 
        if key.char == wristControls[0] and motorsAttached[wristIndex] == True: 
            if not wristMotor.getIsMoving(): 
                wristMotor.setVelocityLimit(motorsInfo[wristIndex][5]) 
        elif key.char == wristControls[1] and motorsAttached[wristIndex] == True: 
            if not wristMotor.getIsMoving(): 
                wristMotor.setVelocityLimit(-motorsInfo[wristIndex][5]) 
        if key.char == pulleyTwistControls[0] and motorsAttached[wrist1Index] and motorsAttached[wrist2Index]:
            if not wrist1Motor.getIsMoving() and not wrist2Motor.getIsMoving(): 
                wrist1Motor.setVelocityLimit(motorsInfo[wrist1Index][5]) 
                wrist2Motor.setVelocityLimit(motorsInfo[wrist2Index][5]) 
        elif key.char == pulleyTwistControls[1] and motorsAttached[wrist1Index] and motorsAttached[wrist2Index]:
            if not wrist1Motor.getIsMoving() and not wrist2Motor.getIsMoving(): 
                wrist1Motor.setVelocityLimit(-motorsInfo[wrist1Index][5]) 
                wrist2Motor.setVelocityLimit(-motorsInfo[wrist2Index][5]) 
        if key.char == pulleyLiftControls[0] and motorsAttached[wrist1Index] and motorsAttached[wrist2Index]:
            if not wrist1Motor.getIsMoving() and not wrist2Motor.getIsMoving(): 
                wrist1Motor.setVelocityLimit(motorsInfo[wrist1Index][5]) 
                wrist2Motor.setVelocityLimit(-motorsInfo[wrist2Index][5]) 
        elif key.char == pulleyLiftControls[1] and motorsAttached[wrist1Index] and motorsAttached[wrist2Index]:
            if not wrist1Motor.getIsMoving() and not wrist2Motor.getIsMoving(): 
                wrist1Motor.setVelocityLimit(-motorsInfo[wrist1Index][5]) 
                wrist2Motor.setVelocityLimit(motorsInfo[wrist2Index][5]) 
        if key.char == clawControls[0]: 
            isClawMoving = True
            claw.setTargetPosition(clawSpeeds[0]) 
            claw.setEngaged(True)
        elif key.char == clawControls[1]: 
            isClawMoving = True
            claw.setTargetPosition(clawSpeeds[1]) 
            claw.setEngaged(True)
    except AttributeError: 
        print("Special key {0} pressed".format(key))

def on_release(key): 
    global baseMotor, shoulderMotor, elbowMotor, wristMotor, wrist1Motor, wrist2Motor, stopFlag, isClawMoving
    try:
        if baseControls[0] == key.char or baseControls[1] == key.char: 
            lim = baseMotor.getVelocityLimit() 
            baseMotor.setVelocityLimit(lim * 3 / 4) 
            time.sleep(smoothing / 4) 
            baseMotor.setVelocityLimit(lim / 2) 
            time.sleep(smoothing / 4) 
            baseMotor.setVelocityLimit(lim / 4) 
            time.sleep(smoothing / 4) 
            baseMotor.setVelocityLimit(0)
        if shoulderControls[0] == key.char or shoulderControls[1] == key.char: 
            lim = shoulderMotor.getVelocityLimit() 
            shoulderMotor.setVelocityLimit(lim * 3 / 4) 
            time.sleep(smoothing / 4) 
            shoulderMotor.setVelocityLimit(lim / 2) 
            time.sleep(smoothing / 4) 
            shoulderMotor.setVelocityLimit(lim / 4) 
            time.sleep(smoothing / 4) 
            shoulderMotor.setVelocityLimit(0)
        if elbowControls[0] == key.char or elbowControls[1] == key.char: 
            lim = elbowMotor.getVelocityLimit() 
            elbowMotor.setVelocityLimit(lim * 3 / 4) 
            time.sleep(smoothing / 4) 
            elbowMotor.setVelocityLimit(lim / 2) 
            time.sleep(smoothing / 4) 
            elbowMotor.setVelocityLimit(lim / 4) 
            time.sleep(smoothing / 4) 
            elbowMotor.setVelocityLimit(0)
        if wristControls[0] == key.char or wristControls[1] == key.char: 
            lim = wristMotor.getVelocityLimit() 
            wristMotor.setVelocityLimit(lim * 3 / 4) 
            time.sleep(smoothing / 4) 
            wristMotor.setVelocityLimit(lim / 2) 
            time.sleep(smoothing / 4) 
            wristMotor.setVelocityLimit(lim / 4) 
            time.sleep(smoothing / 4) 
            wristMotor.setVelocityLimit(0)
        if (pulleyTwistControls[0] == key.char or pulleyTwistControls[1] == key.char) or (pulleyLiftControls[0] == key.char or pulleyLiftControls[1] == key.char): 
            lim = wrist1Motor.getVelocityLimit() 
            wrist1Motor.setVelocityLimit(lim * 3 / 4) 
            time.sleep(smoothing / 4) 
            wrist1Motor.setVelocityLimit(lim / 2) 
            time.sleep(smoothing / 4) 
            wrist1Motor.setVelocityLimit(lim / 4) 
            time.sleep(smoothing / 4) 
            wrist1Motor.setVelocityLimit(0)
            lim = wrist2Motor.getVelocityLimit() 
            wrist2Motor.setVelocityLimit(lim * 3 / 4) 
            time.sleep(smoothing / 4) 
            wrist2Motor.setVelocityLimit(lim / 2) 
            time.sleep(smoothing / 4) 
            wrist2Motor.setVelocityLimit(lim / 4) 
            time.sleep(smoothing / 4) 
            wrist2Motor.setVelocityLimit(0)
        if clawControls[0] == key.char or clawControls[1] == key.char:
            if isClawMoving:
                claw.setTargetPosition(90)
                claw.setEngaged(False)
                isClawMoving = False
        if key.char == 'p':
            print("p is pressed")
            print(f"\nQuitting program...")
            for i in range(len(motors)): 
                if(motors[i].getAttached() == True): 
                    motors[i].setEngaged(False) 
                    motors[i].close() 
            stopFlag = True
    except AttributeError: 
        print("Special key {0} released".format(key)) 

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
            
def main(): 
    # All motors, and positions are declared as global variables 
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
    motorsInfo = [baseInfo, shoulderInfo, ElbowInfo, WristInfo, Wrist1Info, wrist2Info] 

    try: 
        # Functions to initialize components 
        print(f"\nInitializing...\n\n") 
        initialize_motors() 
        print(f"\nSuccessfully initialized!\n")
        listener = keyboard.Listener(on_press=on_press, on_release=on_release) 
        listener.start()

        claw = RCServo()
        claw.setChannel(0)
        claw.setHubPort(0)
        claw.setDeviceSerialNumber(VHubSerial_servo)
        claw.openWaitForAttachment(1000)
        claw.setVoltage(RCServoVoltage.RCSERVO_VOLTAGE_7_4V)
        claw.setMinPulseWidth(500)
        claw.setMaxPulseWidth(2500)

        
        
        # Main loop of code, stopFlag becomes True when 'p' is pressed 
        while(stopFlag == False): 
            time.sleep(0.1)
    except PhidgetException as ex: 
        traceback.print_exc() 
        print() 
        print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)
        print(f"Successfully quit program.\n\nGoodbye!\n")


if __name__ == "__main__": 
    main()

