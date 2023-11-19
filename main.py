from time import sleep
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from enum import Enum
import RPi.GPIO as GPIO
from sshkeyboard import listen_keyboard

doTests = False

class MotorDirection(Enum):
	FORWARD = 1
	BACKWARD = 2

class ServoControl:
	freq = 50
	allSop = False

	i2c = None
	pca = None

	def __init__(self):

		self.i2c = busio.I2C(SCL, SDA)
		self.pca = PCA9685(self.i2c)

	def AllStop(self):
		self.AllStop = True
		print("Servo All Stop")

	def MoveServoTo(self, servo, newAngle):
		print("servo move requested")

	def Test(self):
		print("Servo Test Requested")

class DriveMotorControl:
	driveLeftForwardPin = 27
	driveLeftBackwardsPin = 22
	driveLeftEnablePin = 13
	driveLeftPWM = None

	driveRightForwardPin = 23
	driveRightBackwardsPin = 24
	driveRightEnablePin = 18
	driveRightPWM = None

	pwmFreq = 1000

	def __init__(self):
		print("Drive constructor called")

	def AllStop(self):
		print("AllStop Called")

	def SetPower(self, leftDirection, rightDirection, leftPower, rightPower, ramp=False):
		print("Set Power Called")

	def Test(self):
		print("Drive Test Requested")


#==============
# SSH Keyboard Input
#==============
lastKeyPressed="s"

def keyPressed(key):
	global lastKeyPressed

	firstPress = False

	print("Key Pressed", key)

	if(key == lastKeyPressed):
		firstPress = False
	else:
		firstPress = True

	if (key == ""):
		print("")

	lastKeyPressed = key

#==============
# Execution Ppoint
#=============

ready = False
driveMotorControl = None
servoControl = None


GPIO.setmode(GPIO.BCM)
while (True):
	if (not ready):
		try:
			servoControl = ServoControl()
			driveMotorControl = DriveMotorControl()

			ready = True

			if (doTests):
				servoControl.Test()
				driveMotorControl.Test()

			listen_keyboard(on_press=keyPressed)

		except:
			print("Main Execution Startup Error")
