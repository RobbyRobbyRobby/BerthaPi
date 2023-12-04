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
	driveRightForwardPin = 27
	driveRightBackwardPin = 22
	driveRightEnablePin = 13
	driveRightPWM = None

	driveLeftForwardPin = 23
	driveLeftBackwardPin = 24
	driveLeftEnablePin = 18
	driveLeftPWM = None

	pwmFreq = 1000

	def __init__(self):
		print("Drive constructor called")

		GPIO.setup(self.driveLeftForwardPin, GPIO.OUT)
		GPIO.setup(self.driveLeftBackwardPin, GPIO.OUT)
		GPIO.setup(self.driveLeftEnablePin, GPIO.OUT)
		GPIO.setup(self.driveRightForwardPin, GPIO.OUT)
		GPIO.setup(self.driveRightBackwardPin, GPIO.OUT)
		GPIO.setup(self.driveRightEnablePin, GPIO.OUT)

		GPIO.output(self.driveLeftForwardPin, GPIO.LOW)
		GPIO.output(self.driveLeftBackwardPin, GPIO.LOW)
		GPIO.output(self.driveRightForwardPin, GPIO.LOW)
		GPIO.output(self.driveRightBackwardPin, GPIO.LOW)

		self.driveLeftPWM = GPIO.PWM(self.driveLeftEnablePin, self.pwmFreq)
		self.driveLeftPWM.start(0)
		self.driveRightPWM = GPIO.PWM(self.driveRightEnablePin, self.pwmFreq)
		self.driveRightPWM.start(0)

	def AllStop(self):
		print("AllStop Called")
		self.driveLeftPWM.start(0)
		self.driveRightPWM.start(0)

		GPIO.output(self.driveLeftForwardPin, GPIO.LOW)
		GPIO.output(self.driveLeftBackwardPin, GPIO.LOW)
		GPIO.output(self.driveRightForwardPin, GPIO.LOW)
		GPIO.output(self.driveRightBackwardPin, GPIO.LOW)

	def SetPower(self, leftDirection, rightDirection, leftPower, rightPower, ramp=False):
		GPIO.output(self.driveLeftForwardPin, GPIO.LOW)
		GPIO.output(self.driveLeftBackwardPin, GPIO.LOW)
		GPIO.output(self.driveRightForwardPin, GPIO.LOW)
		GPIO.output(self.driveRightBackwardPin, GPIO.LOW)

		if (leftDirection == MotorDirection.FORWARD):
			GPIO.output(self.driveLeftForwardPin, GPIO.HIGH)
		else:
			GPIO.output(self.driveLeftBackwardPin, GPIO.HIGH)

		if (rightDirection == MotorDirection.FORWARD):
			GPIO.output(self.driveRightForwardPin, GPIO.HIGH)
		else:
			GPIO.output(self.driveRightBackwardPin, GPIO.HIGH)

		if (ramp):
			startingPower = 25
			rampBy = startingPower
			rampLeftPower = startingPower
			rampRightPower = startingPower
			while (rampBy < 100):
				rampBy = rampBy + 1
				if (rampLeftPower < leftPower):
					rampLeftPower = rampLeftPower + 1
				if (rampRightPower < rightPower):
					rampRightPower = rampRightPower + 1

				self.driveLeftPWM.start(rampLeftPower)
				self.driveRightPWM.start(rampRightPower)
				sleep(0.1)
		else:
			self.driveLeftPWM.start(leftPower)
			self.driveRightPWM.start(rightPower)

	def Test(self):
		print("Drive Test Requested")
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.FORWARD, 50, 50)
		sleep(0.5)
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.FORWARD, 100, 100)
		sleep(0.5)
		driveMotorControl.AllStop()
		sleep(1)
		driveMotorControl.SetPower(MotorDirection.BACKWARD, MotorDirection.BACKWARD, 50, 50)
		sleep(1)
		driveMotorControl.SetPower(MotorDirection.BACKWARD, MotorDirection.BACKWARD, 100, 100)
		sleep(1)
		driveMotorControl.AllStop()
		sleep (1)
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.FORWARD, 100, 100, True)
		driveMotorControl.AllStop()

#==============
# SSH Keyboard Input
#==============
lastKeyPressed="s"
pressNumber = 0

def keyPressed(key):
	global lastKeyPressed, pressNumber

	pressPower = 0

	print(key, lastKeyPressed)

	if(key == lastKeyPressed):
		pressNumber = pressNumber + 1
		print("additional")
	else:
		pressNumber = 1

	lastKeyPressed = key

	if (pressNumber == 1):
		pressPower = 50
	if (pressNumber == 2):
		pressPower = 75
	if (pressNumber >= 3):
		pressPower = 100

	print(key, pressNumber, pressPower)

	if (key == "t"):
		driveMotorControl.Test()
	if (key == "w"):
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.FORWARD, pressPower, pressPower)
	if (key == "x"):
		driveMotorControl.SetPower(MotorDirection.BACKWARD, MotorDirection.BACKWARD, pressPower, pressPower)
	if (key == "a"):
		driveMotorControl.SetPower(MotorDirection.BACKWARD, MotorDirection.FORWARD, pressPower, pressPower)
	if (key == "d"):
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.BACKWARD, pressPower, pressPower)
	if (key == "s"):
		driveMotorControl.AllStop()
		pressNumber = 0


#==============
# Execution Point
#=============

ready = False
driveMotorControl = None
servoControl = None
print("Execution point commencing")

GPIO.setmode(GPIO.BCM)

startupErrorCount = 0

while (True):
	if (startupErrorCount > 3):
		print("Main execution failed.  Exiting.")
		break
	if (not ready):
		print("Reports not ready.  Initialising")
		if True: #try:
			servoControl = ServoControl()
			driveMotorControl = DriveMotorControl()

			ready = True
			print("Successful startup, setting ready = true")

			if (doTests):
				servoControl.Test()
				driveMotorControl.Test()

			listen_keyboard(on_press=keyPressed)
			print("Initialization completed")
		#except:
			startupErrorCount = startupErrorCount + 1
			print("Main Execution Startup Error")