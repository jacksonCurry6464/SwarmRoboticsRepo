import time
import RPi.GPIO as GPIO
import numpy as np
import math
import os
import csv
import motorControl
import encoderReader
import movementAlgorithm
import movementPatterns


correctionPercentage = .7
right = 1
left = 2
both = 3

def goalDirectionCircle(goalX,goalY,currentX,currentY,leftSpeed,rightSpeed,goalSpeed,positions,wheelDistance,goalCircle,previousGoal,side,sideCanChange):
	inside = inCircle.lowerCircle(goalCircle[0],currentX,currentY)
	angleToGoal = np.arctan2(goalY-currentY,goalX-currentX)
	currentHeading = positions[2]
	if(currentHeading):
		currentHeading = currentHeading%(2*np.pi*np.sign(currentHeading))
	movementAngle = (rightSpeed - leftSpeed)/wheelDistance
	movementSpeed = (rightSpeed + leftSpeed)/2

	correctionMagnitude = goalSpeed*(correctionPercentage)

	"""
	print('\n angle is {:f}'.format(angle))
	print('current heading is {:f}'.format(currentHeading))

	print('correction direction is {:f}'.format(correctionDirection))
	"""
	#finding the current speed in vector components
	currentSpeedX = movementSpeed*np.cos(movementAngle + currentHeading)
	currentSpeedY = movementSpeed*np.sin(movementAngle + currentHeading)

	#finding the correction speed in vector components
	correctionSpeedX = correctionMagnitude*np.cos(angleToGoal)
	correctionSpeedY = correctionMagnitude*np.sin(angleToGoal)


	finalX = currentSpeedX + correctionSpeedX
	finalY = currentSpeedY + correctionSpeedY

	theta = np.arctan2(finalY,finalX)
	magnitude = np.sqrt(finalY**2 + finalX**2)

	movementAngleIn = movementAngle

	currentGoal = np.sign(angleToGoal)

	previousGoal = currentGoal

	movementAngleDiff = np.abs(movementAngleIn - (theta - currentHeading)%(2*np.pi))

	movementAngle = movementAngleIn + movementAngleDiff - 1.2*inside*movementAngleDiff

	#print('theta {:f} current heading {:f} difference {:f}'.format(theta,currentHeading,theta - currentHeading))

	Vleft = goalSpeed - movementAngle*wheelDistance/2
	Vright = goalSpeed + movementAngle*wheelDistance/2
	"""
	print('movementAngle is {:f} and movementAngleIn is {:f} '.format(movementAngle,movementAngleIn))
	print('Vmag is {:f} and magnitude is {:f} '.format(movementSpeed,magnitude))
	print('input left is {:f} and output is {:f} '.format(leftSpeed,Vleft))
	print('input right is {:f} and output is {:f} '.format(rightSpeed,Vright))
	"""
	return (Vleft,Vright,movementAngleIn,movementAngle,angleToGoal,correctionSpeedX,correctionSpeedY,side,previousGoal,movementAngleDiff)


def forRobotGoal(goalX,goalY,currentX,currentY,leftSpeed,rightSpeed,goalSpeed,positions,wheelDistance):
	angleToGoal = np.arctan2(goalY-currentY,goalX-currentX)
	currentHeading = positions[2]
	if(currentHeading):
		currentHeading = currentHeading%(2*np.pi*np.sign(currentHeading))
	movementAngle = (rightSpeed - leftSpeed)/wheelDistance
	movementSpeed = (rightSpeed + leftSpeed)/2

	correctionMagnitude = goalSpeed*(correctionPercentage)

	"""
	print('\n angle is {:f}'.format(angle))
	print('current heading is {:f}'.format(currentHeading))

	print('correction direction is {:f}'.format(correctionDirection))
	"""
	#finding the current speed in vector components
	currentSpeedX = movementSpeed*np.cos(movementAngle + currentHeading)
	currentSpeedY = movementSpeed*np.sin(movementAngle + currentHeading)

	#finding the correction speed in vector components
	correctionSpeedX = correctionMagnitude*np.cos(angleToGoal)
	correctionSpeedY = correctionMagnitude*np.sin(angleToGoal)


	finalX = currentSpeedX + correctionSpeedX
	finalY = currentSpeedY + correctionSpeedY

	theta = np.arctan2(finalY,finalX)
	magnitude = np.sqrt(finalY**2 + finalX**2)

	movementAngleIn = movementAngle

	currentGoal = np.sign(angleToGoal)

	previousGoal = currentGoal

	movementAngleDiff = np.abs(movementAngleIn - (theta - currentHeading)%(2*np.pi))

	movementAngle = movementAngleIn + movementAngleDiff

	#print('theta {:f} current heading {:f} difference {:f}'.format(theta,currentHeading,theta - currentHeading))

	Vleft = goalSpeed - movementAngle*wheelDistance/2
	Vright = goalSpeed + movementAngle*wheelDistance/2
	"""
	print('movementAngle is {:f} and movementAngleIn is {:f} '.format(movementAngle,movementAngleIn))
	print('Vmag is {:f} and magnitude is {:f} '.format(movementSpeed,magnitude))
	print('input left is {:f} and output is {:f} '.format(leftSpeed,Vleft))
	print('input right is {:f} and output is {:f} '.format(rightSpeed,Vright))
	"""
	return (Vleft,Vright,movementAngleIn,movementAngle,angleToGoal,correctionSpeedX,correctionSpeedY,previousGoal,movementAngleDiff)

def goalDirectionLine(goalX,goalY,position,goalSpeed,wheelDistance,leftSpeed,rightSpeed):
	currentX = position[0]
	currentY = position[1]
	currentHeading = position[2]
	angleToGoal = np.arctan2(goalY - currentY,goalX - currentX)

	"""
	# as a vector
	xVelocity = currentSpeed*np.cos(currentHeading)
	yVelocity = currentSpeed*np.sin(currentHeading)

	#correction vector
	correctionX = 0
	correctionY = -.2*np.sign(currentY)

	newXVelocity = xVelocity + correctionX
	newYVelocity = yVelocity + correctionY

	nemMovementAngle = np.arctan2(newYVelocity)
	"""

	Vleft = goalSpeed + currentHeading/wheelDistance
	Vright = goalSpeed - currentHeading/wheelDistance

	if(currentY > 0):
		Vleft += .1
	else:
		Vright += .1

	return(Vleft,Vright)


def moveToPointTics(goalX,goalY,wheelDistance,speed):
	distance = np.sqrt(goalX**2 + goalY**2)
	wheelCicumfrence =  2.5*.0254*2*np.pi
	theta = wheelCicumfrence/20/wheelDistance
	xSign = np.sign(goalX)
	side = 0
	if not (xSign):
		#no shift
		pass
	elif( xSign < 0):
		side = right
	else:
		side = left

	ticList = []

	shiftCount = np.floor(np.abs(goalX)/(wheelDistance/2*(1-np.cos(theta)))) #seeing how many jogs left or right we will need

	if(shiftCount != 0):
		forwardCount = int(np.floor(np.sqrt((20*distance/wheelCicumfrence)**2 - shiftCount*wheelDistance/2*np.cos(theta)) - wheelDistance/2*shiftCount*np.sin(theta))/(.0254*2.5*np.pi/20) + shiftCount)

		turnEveryNCount = int(np.floor(forwardCount/shiftCount))
		for ii in range(turnEveryNCount + forwardCount):
			if not (ii%turnEveryNCount):
				if(side == left):
					ticList.append(left)
				elif(side == right):
					ticList.append(right)

			else:
				ticList.append(both)


	else:
		ticList = [both]*speed

	return (ticList)


def moveToAngleTics(currentHeading,goalHeading,wheelDistance):
	deltaTheta = goalHeading - currentHeading
	wheelCircumference = 2.5*.0254*np.pi
	robotPathCircumference = np.pi*wheelDistance
	wheelRotationsPerFullSpin = robotPathCircumference/wheelCircumference
	ticsPerFullSpin = 20*wheelRotationsPerFullSpin
	radiansPerTic = 2*np.pi/ticsPerFullSpin
	ticCount = deltaTheta/(np.pi*2)/radiansPerTic
	return (ticCount)
