import pigpio
import RPi.GPIO as GPIO
import time
import csv
import numpy as np
import angleEstimator
import positionCalc
import motorControl
import encoderReader

#this is meant to be be run as the way the robot moves
#it takes a angle amount to move and uses photo diodes and IR emitters to follow a path
right = 1
left = 2
both = 3

encoderRightPin = 21
encoderLeftPin = 4
GPIO.setup(encoderRightPin,GPIO.IN)
GPIO.setup(encoderLeftPin,GPIO.IN)

IRRightPin = 6
IRLeftPin = 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(IRRightPin,GPIO.IN)
GPIO.setup(IRLeftPin,GPIO.IN)

PWMVal = .9

def readIR():
	leftIR = GPIO.input(IRLeftPin)						#JACKSON HERE TRYING SWITCHING THESE(right first left second)
	rightIR = GPIO.input(IRRightPin)
	#print('left IR {:f}'.format(leftIR))
	#print('right IR {:f}'.format(rightIR))
	motorControl.forward(both)
	if(leftIR and rightIR):
		#stop
		#print('stop')
		#motorControl.setSpeed(0,left)
		#motorControl.setSpeed(0,right)
		#time.sleep(.5)
		#go straight
		#print('none picked up')
		motorControl.setSpeed(1,right)				#this was pwm value	corresponds with left wheel spinning
		motorControl.setSpeed(PWMVal,left)
	elif(leftIR and not rightIR):						#switched placement of not in this statement to be before left instead of right
		#turn left
		#print('left on white and right on black so left wheel spins')
		motorControl.setSpeed(PWMVal,left)
		motorControl.setSpeed(0,right)
		#time.sleep(.3)
	elif(not leftIR and rightIR):						#switched placement of not in this statement to be before right instead left
		#turn right
		#print('right')
		motorControl.setSpeed(PWMVal,right)
		motorControl.setSpeed(0,left)
		#time.sleep(.3)
	elif(not leftIR and not rightIR):
		#go straight
		#print('straight')
		motorControl.setSpeed(1,right)
		motorControl.setSpeed(PWMVal,left)

#this moves the robot a given angle along the circle
def moveAngle(angleToMove,circleRad):
	#angleToMove = 2*np.pi
	#print(angleToMove)
	print("ANGLE TO MOVE:")
	print(angleToMove)
	print("GOAL TIC COUNT:")
	side = right
	encoderPin = encoderRightPin
	previousInput = GPIO.input(encoderPin)
	completed = 0
	goalTicCount = np.floor((angleToMove*circleRad)*20/(2.5*.0254*np.pi))			#FIRST PART OF THIS GIVES ARCLENGTH (and divides by 20 for number of tics in a wheel spin), SECOND GiVES CIRCUMFERENCE OF WHEEL IN METERS
	ticCount = 0
	print(goalTicCount)
	delay = 1 + time.time()

	while(not completed):
		(ticCount,previousInput,completed) = encoderReader.moveTicCount(side,ticCount,goalTicCount,previousInput,completed)  #THIS KEEPS TRACK OF HOW MANY TICS HAVE HAPPENED (since in same loop these run while motor is going)
		readIR()				#THIS IS WHERE THE ACTUAL MOVEMENT COMES FROM

	motorControl.setSpeed(0,both)
	while(time.time() < delay):
		pass
	print("DONE MOVING")
