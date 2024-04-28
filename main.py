from time import sleep
import busio
import board
import neopixel
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from enum import Enum
import RPi.GPIO as GPIO
from sshkeyboard import listen_keyboard
import sys
import adafruit_vl53l0x
import tm1637

doTests = False
i2c = None

class MotorDirection(Enum):
	FORWARD = 1
	BACKWARD = 2

class ServoControl:
	#freq = 100
	freq = 50
	allStop = False
	servoPanChannel = 0
	servoPan = None
	panPos = 130
	panHomePos = 130
	servoTiltChannel = 1
	servoTilt = None
	tiltPos = 110
	tiltHomePos = 110
	servoClawChannel = 3
	servoClaw = None
	clawPos = 50
	clawHomePos = 50

	i2c = None
	pca = None

	def __init__(self, _i2c):
		print("servo control starting")
		self.i2c = _i2c
		self.pca = PCA9685(self.i2c)
		self.pca.frequency = self.freq

		print("servo i2c and pca ready")
		self.servoPan = servo.Servo(self.pca.channels[self.servoPanChannel])
		self.servoTilt = servo.Servo(self.pca.channels[self.servoTiltChannel])
		print("servo Pan and Tilt ready")

		self.servoClaw = servo.Servo(self.pca.channels[self.servoClawChannel])
		print("claw ready")

		self.PanTiltTo(self.panPos, self.tiltPos)
		self.ClawOpen(0)

		print("servo control ready")

	def AllStop(self):
		self.AllStop = True
		print("Servo All Stop")

	def PanTiltTo(self, newPan = -1, newTilt = -1):
		#tilt higher number = down
		#pan higher number = left

		if (newPan <= 180 and newPan >= 0):
			self.panPos = newPan
		else:
			self.panPos = self.panHomePos

		self.servoPan.angle = self.panPos
		print(self.panPos)

		sleep(0.25)

		if (newTilt <= 180 and newTilt >= 0):
			self.tiltPos = newTilt
		else:
			self.tiltPos = self.tiltHomePos

		self.servoTilt.angle = self.tiltPos
		print(self.tiltPos)

	def PanTiltBy(self, panModifier, tiltModifier):
		#tilt higher number = down
		#pan higher number = left
		tempPan = self.panPos + panModifier
		tempTilt = self.tiltPos + tiltModifier

		self.PanTiltTo(tempPan, tempTilt)

		#if (tempPan <= 180 and tempPan >= 0):
		#	self.panPos = tempPan
		#if (tempTilt <= 180 and tempTilt >= 0):
		#	self.tiltPos = tempTilt

		#self.PanTiltTo(self.panPos, self.tiltPos)

	def ClawOpen(self, modifier):
		#higher number = closed

		self.clawPos = self.clawPos + modifier

		if (self.clawPos <= self.servoClaw.actuation_range and self.clawPos >= 0):
			self.servoClaw.angle = self.clawPos

		print(self.clawPos)

	def Test(self):
		print("Servo Test Requested")

class DriveMotorControl:
	driveLeftForwardPin = 27
	driveLeftBackwardPin = 22
	driveLeftEnablePin = 13
	driveLeftPWM = None

	driveRightForwardPin = 23
	driveRightBackwardPin = 24
	driveRightEnablePin = 18
	driveRightPWM = None

	pwmFreq = 25

	trim = 5

	sensorIRPin = 12
	sensorIRCount = 0
	distanceCountMultiplier = 0.84
	# 10 IRTicks is about 8.5mm.  So 12 is close to 10mm.  That makes a multiplier of 0.83333mm per tick.  I rounded up.

	def sensorIR_callback(self, channel):
		self.sensorIRCount = self.sensorIRCount + 1
		print(self.sensorIRCount)

		if (self.sensorIRCount >= 10):
			self.AllStop()
			self.sensorIRCount = 0

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

		try:
			GPIO.setup(self.sensorIRPin, GPIO.IN)
			GPIO.add_event_detect(self.sensorIRPin, GPIO.RISING, callback=self.sensorIR_callback)
			print("IR Travel Distance Sensor startup success")
		except:
			print("Error starting IR Travel Distance Sensor")

	def ChangePWMFrequency(self, newFreq):
		self.driveLeftPWM.ChangeFrequency(newFreq)

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
			leftPower = leftPower + self.trim
			leftPower = min(leftPower, 100)
			leftPower = max(leftPower, 0)

			rightPower = rightPower - self.trim
			rightPower = min(rightPower, 100)
			rightPower = max(rightPower, 0)

			print(leftPower, rightPower)

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
	global lastKeyPressed , pressNumber #, sensorIRCount

	pressPower = 0

#	print(key, lastKeyPressed)

	if(key == lastKeyPressed):
		pressNumber = pressNumber + 1
		#print("additional")
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
	if (key == "q"):
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.FORWARD, 10, 0)
	if (key == "e"):
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.FORWARD, 0, 10)
	if (key == "x"):
		driveMotorControl.SetPower(MotorDirection.BACKWARD, MotorDirection.BACKWARD, pressPower, pressPower)
	if (key == "a"):
		driveMotorControl.SetPower(MotorDirection.BACKWARD, MotorDirection.FORWARD, pressPower, pressPower)
	if (key == "d"):
		driveMotorControl.SetPower(MotorDirection.FORWARD, MotorDirection.BACKWARD, pressPower, pressPower)
	if (key == "b"):
		servoControl.PanTiltTo()
		#servoControl.MoveServoTo(0, 90)
		#sleep(1)
		#servoControl.MoveServoTo(0, 45)
		#sleep(1)
		#servoControl.MoveServoTo(0, 145)
		#sleep(1)
	if (key == "s"):
		driveMotorControl.AllStop()
		#sensorIRCount = 0
		pressNumber = 0
	if (key == "j"):
		#servoControl.PanTiltTo(140,5)
		servoControl.PanTiltBy(10,0)
	if (key == "l"):
		#servoControl.PanTiltTo(40,5)
		servoControl.PanTiltBy(-10,0)
	if (key == "i"):
		#servoControl.PanTiltTo(90,5)
		servoControl.PanTiltBy(0,10)
	if (key == "k"):
		#servoControl.PanTiltTo(90,175)
		servoControl.PanTiltBy(0,-10)
	if (key == "n"):
		sensorsAndLights.LightsOn()
	if (key == "m"):
		sensorsAndLights.LightsOff()
	if (key == ","):
		servoControl.ClawOpen(-5)
	if (key == "."):
		servoControl.ClawOpen(5)
	if (key == "z"):
		driveMotorControl.AllStop()
		sys.exit()
	if (key == "f"):
		sensorsAndLights.GetToFDistance()


class SensorsAndLights:
	distanceFront = None
	sevenSeg = None
	i2c = None
	neo = None
	lastDistanceFront = 0
#	sevenSegDIO = 19
#	sevenSegCLK = 26

	def __init__(self, _i2c):
		print("Starting ToF")
		self.i2c = _i2c

		try:
			print("Init ToF")
			self.distanceFront = adafruit_vl53l0x.VL53L0X(i2c)
			print("ToF  ready")
			self.GetToFDistance()
			print("ToF startup complete")
		except:
			print("VL53L0 init error")

#		sevenSeg = tm1637.TM1637(clk=self.sevenSegCLK, dio=self.sevenSegDIO)
		print("Starting neopixel")

		try:
			# NeoPixels must be connected to D10, D12, D18 or D21 to work.
			self.neo = neopixel.NeoPixel(board.D21, 8,brightness=0.5,pixel_order=neopixel.GRB,auto_write=True,bpp=3)
			print("neopixel ready")
			self.LightsOff()
		except:
			print("neopixel startup error")


	def LightsOn(self):
		#global neo
		self.neo[0] = (255,255,255)
		self.neo[3] = (255,255,255)
		self.neo[4] = (255,255,255)
		self.neo[7] = (255,255,255)
		print("neopixel on")

	def LightsOff(self):
		#global neo
		self.neo.fill((0,0,0))
		print("neopixel off")

	def GetToFDistance(self):
		self.lastDistanceFront = self.distanceFront.range
		print('Range: {}mm'.format(self.lastDistanceFront))

#==============
# Execution Point
#=============

ready = False
sensorsAndLights = None
driveMotorControl = None
servoControl = None
print("Execution point commencing")
#neo = None

GPIO.setmode(GPIO.BCM)

startupErrorCount = 0

while (True):
	if (startupErrorCount > 3):
		print("Main execution startup failed.  Giving up.")
		#break

	if (not ready and startupErrorCount < 4):
		print("Bootstrap process reports not ready, or error.  Initialising now.")
		#if True:
		try:
			i2c = busio.I2C(board.SCL, board.SDA)
			sensorsAndLights = SensorsAndLights(i2c)
			servoControl = ServoControl(i2c)
			driveMotorControl = DriveMotorControl()

			ready = True
			print("Successful startup, setting ready = true")

			if (doTests):
				servoControl.Test()
				driveMotorControl.Test()

		except:
			startupErrorCount = startupErrorCount + 1
			print("Main Execution Startup Error", startupErrorCount)

	listen_keyboard(on_press=keyPressed)
	print("Initialization completed")
