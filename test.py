import time
import RPi.GPIO as GPIO

sensorPin = 32

#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(sensorPin, GPIO.IN)

print ("Ready")

tickCount = 0
tickBool = False
print(tickBool)
print(tickCount)

def my_callback(channel):
	global tickBool, tickCount, sensorPin
	if (GPIO.input(sensorPin)):
		if (tickBool == False):
			tickCount = tickCount + 1
			tickBool = True
			print(tickCount)
	else:
		tickBool = False

GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensorPin, GPIO.IN)
GPIO.add_event_detect(sensorPin, GPIO.RISING, callback=my_callback, bouncetime=300)

while True:
#	if (GPIO.input(sensor)):
#		if (tickBool == False):
#			tickCount = tickCount + 1
#			tickBool = True
#			print(tickCount)
#	else:
#		tickBool = False
#	#print(tickBool)
	#time.sleep(0.1)
	pass
