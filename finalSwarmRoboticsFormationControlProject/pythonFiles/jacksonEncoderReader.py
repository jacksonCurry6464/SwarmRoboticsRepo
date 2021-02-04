import time
import RPi.GPIO as GPIO
import numpy as np
import math
import os
import csv

#setting up the encoders in the GPIO
encoderRightPin = 21
encoderLeftPin = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(encoderRightPin,GPIO.IN)
GPIO.setup(encoderLeftPin,GPIO.IN)

right = 1
left = 2
both = 3

encoderPin = 0

def readSpeed(previousLeadingEdge,previousInput,zeroCount,side,speed):
	if(side == right):
		encoderPin = encoderRightPin
	elif(side == left):
		encoderPin = encoderLeftPin

	currentInput = GPIO.input(encoderPin)

	if(currentInput and not previousInput):
		#caluclating the linear speed of the wheel in the form (in to m conversion)*(arclength traveled between encoder ticks)/(change in time since last encoder tick)
		speed = (2.5/2*0.0254)*(2*np.pi/20)/(time.time()-previousLeadingEdge)
		#updating the time of the last encoder tick
		previousLeadingEdge = time.time()
		zeroCount = 0
	else:
		zeroCount += 1

	previousInput = currentInput
	#because we must work in discrete time we assume that the robot continues moving even when we arent getting new data
	#this means that we assume that the robot maintains the seem wheel speed between encoder ticks this can lead to an issue where the wheel stops and we continue that the wheel has not
	#to mitigate this we know that every few cycles th encoder should update and when its doesnt we can assume that the wheel speed is zero
	if(zeroCount > 60):
		speed = 0
		zeroCount = 0

	return (previousLeadingEdge,previousInput,zeroCount,side,speed)


def moveOneTic(side):
	#this function stays in the foreground while the motor is moving
	if(side == left):
		encoderPin = encoderleftPin
		firstInput = GPIO.input(encoderPin)
		while(GPIO.input(encoderPin) == firstInput):
			pass
	elif(side == right):
		encoderPin = encoderRightPin
		firstInput = GPIO.input(encoderPin)
		while(GPIO.input(encoderPin) == firstInput):
			pass
	elif(side == both):
		firstInputRight = GPIO.input(encoderRightPin)
		firstInputLeft = GPIO.input(encoderLeftPin)
		leftDone = 0
		rightDone = 0
		while(not leftDone and not rightDone):
			if(not leftDone):
				if(firstInputLeft != GPIO.input(encoderLeftPin)):
					leftDone = 1
			if(not rightDone):
				if(firstInputRight != GPIO.input(encoderRightPin)):
					rightDone = 1

	return 0


def moveTicCount(side,ticCount,goalTicCount,previousInput,completed):
	if(side == left):
		encoderPin = encoderleftPin
		currentInput = GPIO.input(encoderPin)
	elif(side == right):
		encoderPin = encoderRightPin
		currentInput = GPIO.input(encoderPin)

	if(currentInput != previousInput):
		ticCount += 1

	if(ticCount >= goalTicCount*2):
		completed = 1
	#print(ticCount)
	return (ticCount,currentInput,completed)
