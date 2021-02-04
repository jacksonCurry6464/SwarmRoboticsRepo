import RPi.GPIO as GPIO
import time
#initalalizing the gpio
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


#initalalizing the sensor and servo pins
TRIG = 16
ECHO = 20

#setting up the ultrasonic
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

#some useful variables

def pingUltrasonic():
	GPIO.output(TRIG,True)
	time.sleep(0.00001)
	GPIO.output(TRIG,False)

	pulse_start = time.time()
	ender = pulse_start + 1
	while(GPIO.input(ECHO) == 0 and time.time() < ender):
		pulse_start = time.time()
	pulse_end = time.time()
	while(GPIO.input(ECHO) == 1 and time.time() < ender +1):
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start
	distance = pulse_duration * 17150 #distance in cm
	distance = round(distance,2)
	distance = distance/100				#CONVERTING THE VALUE TO METERS
	return distance


test = 0													#MAKE THIS 0 WHEN YOU DONT WANT TO TEST THE SENSOR, AND 1 WHEN YOU DO
if test:
	for x in range(10):
		myVariable = pingUltrasonic()
		print(myVariable)