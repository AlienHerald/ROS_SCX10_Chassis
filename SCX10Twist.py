#!usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time
import Jetson.GPIO as GPIO

i2c = busio.I2C(SCL, SDA)

print("adding i2c bus")
pca = PCA9685(i2c)
print("i2c bus added")

pca.frequency = 50

NEUTRAL = 90
MAX_STEER = 30

print("defining servos")
motor = servo.Servo(pca.channels[0])
steering = servo.Servo(pca.channels[1])

print("set throttle to neutral")
motor.angle = 90
print("throttle set to neutral")

# RELAY_PIN = 5

# print("set up relay")
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.LOW)

# def onRelay():
#     GPIO.output(RELAY_PIN, GPIO.HIGH)

# def offRelay():
#     GPIO.output(RELAY_PIN, GPIO.LOW)
# print("relay set up")

def checkServoAngle(srv:servo):
    if srv.angle is None or srv.angle > 180 or srv.angle < 0:
        srv.angle = 90

print("checking servo angles")
checkServoAngle(motor)
checkServoAngle(steering)
print("servo angles good")

print("setting up remaining definitions")
def stopMotor():
    if motor.angle == None:
        motor.angle = 90
    checkServoAngle(motor)
    if (motor.angle < 90):
        step = 1
    else:
        step = -1
    for i in range(int(motor.angle), 90, step):
        motor.angle = i
        time.sleep(0.05)

def stopSteering():
    steering.angle = NEUTRAL

def stopAll():
    stopMotor()
    stopSteering()
    # offRelay()

def drive(speed:float):
    # onRelay()
    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    speed *= .5
    motor.angle = 90 + speed

def steer(angle:float):
    # angle *= -1
    toAngle = NEUTRAL + int(angle)
    if toAngle < NEUTRAL - MAX_STEER:
        toAngle = NEUTRAL - MAX_STEER
    elif toAngle > NEUTRAL + MAX_STEER:
        toAngle = NEUTRAL + MAX_STEER
    steering.angle = toAngle
print("definitions set up, wait for node to register")

class SCX10:
    def __init__(self):
        rospy.init_node('scx10')

        self._last_received = rospy.get_time()
        self._timeout = 1.0
        self._rate = 10
        self._max_speed = 0.5
        self._wheel_base = 0.3
        
        self._speed_percent = 0.0
        self._steering_percent = NEUTRAL

        rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)

    def velocity_received_callback(self, message):
        self._last_received = rospy.get_time()

        linear = message.linear.x

        if linear > 1.0:
            linear = 1.0
        elif linear < -1.0:
            linear = -1.0

        angular = message.angular.z

        if angular > 1.0:
            angular = 1.0
        elif angular < -1.0:
            angular = -1.0

        self._speed_percent = linear * 100.0
        self._steering_percent = angular * 100.0

        print("//----------------------//")
        print("COMMAND:")
        print("in angular: " + str(angular))
        print("in linear: " + str(linear))
        print("got speed!")
        print("out speed: " + str(round(self._speed_percent,1)) + "%")
        print("out turn: " + str(round(self._steering_percent,1)) + "%")

    def run(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            delay = rospy.get_time() - self._last_received

            if delay < self._timeout:
                drive(self._speed_percent)
                steer(self._steering_percent)
            else:
                stopAll()
            
            rate.sleep()

def main():
    scx10 = SCX10()

    scx10.run()

if __name__ == '__main__':
    main()


