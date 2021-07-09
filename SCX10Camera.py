from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time

i2c = busio.I2C(SCL, SDA)
print("connectig to pca9685")
pca = PCA9685(i2c)
print("connected to pca9685")

pca.frequency = 50

MOVING = False

print("defining servo angles")
CAM0_NEUTRAL = 102
CAM1_NEUTRAL = 110

CAM0_MIN = 10
CAM0_MAX = 170

CAM1_MIN = 60
CAM1_MAX = 150

cam0 = servo.Servo(pca.channels[2])
cam1 = servo.Servo(pca.channels[3])
lightsA = servo.Servo(pca.channels[4])
lightsB = servo.Servo(pca.channels[5])

def checkServoAngle(srv:servo):
    if srv.angle is None or srv.angle > 180 or srv.angle < 0:
        srv.angle = 90

print("checking servo angles...")
checkServoAngle(cam0)
checkServoAngle(cam1)
print("check complete")

print("setting up definitions")
def moveCamServo(srv:servo, ang:float):
    checkServoAngle(srv)
    MOVING = True
    if srv == cam0:
        if ang > CAM0_MAX:
            ang = CAM0_MAX
        elif ang < CAM0_MIN:
            ang = CAM0_MIN
    elif srv == cam1:
        if ang > CAM1_MAX:
            ang = CAM1_MAX
        elif ang < CAM1_MIN:
            ang = CAM1_MIN
    if srv.angle > ang:
        step = -1
    else:
        step = 1
    for i in range(int(srv.angle), int(ang), step):
        srv.angle = i
        time.sleep(0.07)
    MOVING = False

def stopCam():
    moveCamServo(cam0, CAM0_NEUTRAL)
    moveCamServo(cam1, CAM1_NEUTRAL)

def toggleLights():
    lightsA.angle = 50
    time.sleep(0.05)
    lightsA.angle = 150
    lightsB.angle = 50
    time.sleep(0.05)
    lightsB.angle = 150

print("Camera servos active. Please wait.")

while True:
    move = input("Enter move type: ")
    if move == ']':
        moveCamServo(cam0, CAM0_MIN)
    elif move == '[':
        moveCamServo(cam0, CAM0_MAX)
    elif move == '=':
        moveCamServo(cam1, CAM1_MIN)
    elif move == '-':
        moveCamServo(cam1, CAM1_MAX)
    elif move == ';':
        moveCamServo(cam1, 88)
    elif move == 'scan':
        moveCamServo(cam0, CAM0_MIN)
        moveCamServo(cam0, CAM0_MAX)
        moveCamServo(cam0, CAM0_NEUTRAL)
    elif move == 'lights' or move == 'toggle':
        toggleLights()
    elif MOVING == False:
        stopCam()
