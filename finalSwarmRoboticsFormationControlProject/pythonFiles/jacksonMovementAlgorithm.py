import time
import RPi.GPIO as GPIO
import numpy as np
import math
import motorControl
import encoderReader
import pathCorrector
import movementPatterns
import positionCalc
import robotPosition

#methods for finding the goal point
leaderMovement = 0
closestPoint = 0
tangetGoal = 1



def discreteAngle(speed,position,goalCircle,continousPositionX,continousPositionY,continousAngle,continousDistance,averageAngle,averageDistance,positionLogX,positionLogY,positionLogAngle,robotInView,numberOfRobots,wheelDistance):
	#getting some information on the robot position
	(currentAngle,distanceFromEdge) = robotPosition.angleAlong(position[0],position[1],position[2],goalCircle[0])
	robotInCircle = 0
	if(distanceFromEdge >= 0):
		robotInCircle = 1
	print('robot in view {:f}'.format(robotInView))
	print(averageAngle)
	if(robotInView):
		print(averageDistance)
		averageDistance = averageDistance/100
		angleFrom90 = (np.pi - averageAngle) - np.pi/2
		print(angleFrom90)

		leaderPositionX = averageDistance*np.cos(angleFrom90 + np.pi/2 + currentAngle) + position[0]
		leaderPositionY = averageDistance*np.sin(angleFrom90 + np.pi/2 + currentAngle) + position[1]

		print(leaderPositionX,leaderPositionY)
		print(position[0],position[1])
		(leaderAngleAlong,leaderDistanceFromEdge) = robotPosition.angleAlong(leaderPositionX,leaderPositionY,1,goalCircle[0])
		leaderAngleAlong = leaderAngleAlong%(2*np.pi)
		currentAngle = currentAngle%(2*np.pi)
		

		expectedLeaderAngle = leaderAngleAlong + speed
		print('robot current angle {:f}'.format(currentAngle))
		print('leader current angle {:f}'.format(leaderAngleAlong))
		print('expected leader angle{:f}'.format(expectedLeaderAngle))
		print('angle back from leader {:f}'.format((leaderAngleAlong - currentAngle)*180/np.pi))
		#if(np.abs(and))
		angleBackFromLeader = (leaderAngleAlong - currentAngle)%(2*np.pi) + np.pi/180 + 25*np.pi/180
		if(angleBackFromLeader*180/np.pi > 100):
			angleBackFromLeader = float(100*np.pi/180)
		print('angle back from leader {:f}'.format((angleBackFromLeader)*180/np.pi))

		fallBackAngle = 2*np.pi/numberOfRobots
		goalRobotAngle = angleBackFromLeader - fallBackAngle + speed + currentAngle
		newSpeed = (goalRobotAngle - currentAngle)
		print('new speed {:f}'.format(newSpeed))

		if(newSpeed < speed*.7):
			goalRobotAngle = currentAngle + speed*.7
			print('falling back from leader')
		elif(newSpeed > speed*1.2):
			goalRobotAngle = currentAngle + speed*1.2
			print("catching up to leader")

		else:
			print('moving with leader')


	else:
		goalRobotAngle = speed*1.2 + currentAngle
		angleBackFromLeader = 2*np.pi/numberOfRobots

	print('goalRobotangle{:f}'.format(goalRobotAngle))
	#finding the vector to the goal angle position
	distanceToGoal = np.sqrt((goalCircle[0]*np.cos(goalRobotAngle) - position[0])**2 + (goalCircle[0]*np.sin(goalRobotAngle) - position[0] - goalCircle[0])**2)

	angleToGoal = (goalRobotAngle - currentAngle)%(np.pi*2)

	goalX = distanceToGoal*np.cos(np.pi/2 + angleToGoal)
	goalY = distanceToGoal*np.sin(np.pi/2 + angleToGoal)

	ticListMovement = pathCorrector.moveToPointTics(goalX,goalY,wheelDistance,speed)

	goalHeading = currentAngle - np.pi/2

	ticCountAngle = pathCorrector.moveToAngleTics(position[2],goalHeading,wheelDistance)


	return (ticListMovement,ticCountAngle,angleToGoal,angleBackFromLeader)


def leaderTics(speed,goalCircle,position,wheelDistance):		#JACKSON CHECK TO SEE IF THIS IS NEEDED WHEN IT BECOMES THE LEADER!!!
	(currentAngle,distanceFromEdge) = robotPosition.angleAlong(position[0],position[1],position[2],goalCircle[0])
	goalRobotAngle = speed + currentAngle

	#finding the vector to the goal angle position
	distanceToGoal = np.sqrt((goalCircle[0]*np.cos(goalRobotAngle) - position[0])**2 + (goalCircle[0]*np.sin(goalRobotAngle) - position[0] - goalCircle[0])**2)

	angleToGoal = np.arctan2(goalCircle[0]*np.sin(goalRobotAngle) - position[1] - goalCircle[0],goalCircle[0]*np.cos(goalRobotAngle) - position[0]) - position[2]

	goalX = distanceToGoal*np.cos(np.pi/2 + angleToGoal)
	goalY = distanceToGoal*np.sin(np.pi/2 + angleToGoal)

	ticListMovement = 0 #pathCorrector.moveToPointTics(goalX,goalY,wheelDistance,speed)

	goalHeading = currentAngle - np.pi/2

	ticCountAngle = 0 #pathCorrector.moveToAngleTics(position[2],goalHeading,wheelDistance)


	return (ticListMovement,ticCountAngle,speed)


