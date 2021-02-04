import numpy as np
import time
import math

robotRadius = 0.1778/2*100
maximumRadialDistanceDifference = robotRadius*1.5
maxSensorDistance = 80
allowableAngleError = 4*np.pi/180 #5 degrees

def findCenter(p1,p2):
	#This function uses a two point extimation to find the center of the circle
	(x1,y1) = p1
	(x2,y2) = p2

	#making sure that the p1 and p2 are in the correct positions
	if(x1 > x2):
		#switching the angles if they are not in ascending order
		holderX = x1
		x1 = x2
		x2 = holderX

		holderY = y1
		y1 = y2
		y2 = holderY

	theta1 = np.arctan2(y1,x1)
	theta2 = np.arctan2(y2,x2)

	p1p2Distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	#print(x1,y1)
	#print(x2,y2)
	theta3 = np.arctan2(y2 - y1,x2 - x1)
	#print(theta3)

	botCenterX = x1 + p1p2Distance*np.cos(theta3)/2 + robotRadius*np.cos(theta3 - np.pi/2) + 1*.0254/100
	botCenterY = y1 + p1p2Distance*np.sin(theta3)/2 + robotRadius*np.sin(theta3 - np.pi/2) + 5*.0254*100
	print('botCenterX {:f}'.format(botCenterX/.0254/100))
	print('botCenterY {:f}'.format(botCenterY/.0254/100))
	return (botCenterX,botCenterY)


def findRobots(distancesFromData,anglesFromData):
	distances = distancesFromData
	angles = anglesFromData
	distanceHolder = []
	angleHolder = []
	startAngles = []
	finishAngles = []
	startDistance = []
	finishDistance = []
	startAnglesFinal = []
	finishAnglesFinal = []
	startDistanceFinal = []
	finishDistanceFinal = []
	angleCountIn = len(angles)
	angleCount = 0

	#removing all the data points that are farther than sensorRnage cm and removing all datapoints that are more than one robot diameter away

	for ii in range(0,angleCountIn-2):
		if((distances[ii + 1] < maxSensorDistance) and ((distances[ii] < (distances[ii + 1] + robotRadius)) or (distances[ii] > (distances[ii + 1] - robotRadius)))):
			if(distances[ii] < maxSensorDistance):
				distanceHolder.append(distances[ii])
				angleHolder.append(angles[ii])
				angleCount += 1
				#print('data found')
		elif((distances[ii + 2] < maxSensorDistance) and ((distances[ii] < (distances[ii + 2] + robotRadius)) or (distances[ii] > (distances[ii + 2] - robotRadius)))):
			if(distances[ii] < maxSensorDistance):
				distanceHolder.append(distances[ii])
				angleHolder.append(angles[ii])
				angleCount += 1
	#checking if the first angle is in the range because it cannot be handeled by the for loop
	
	#print('angle holder')
	#print(angleHolder)
	#assigning the angles to certain robots and finding the centers or rbots based on this
	if(angleHolder):
		currentAngle = angleHolder[0]
		currentRobot = 1
		robotAssignment = [None] * angleCount
		robotAssignment[0] = currentRobot
		for kk in range(1,angleCount):
			if(angleHolder[kk] > (allowableAngleError + currentAngle)):
				currentRobot += 1

			robotAssignment[kk] = currentRobot
			currentAngle = angleHolder[kk]

		#print(currentRobot)
		#finding the start and finish angles of the robots in viewable
		startAngles = [None]*currentRobot
		finishAngles = [None]*currentRobot
		startDistance = [None]*currentRobot
		finishDistance = [None]*currentRobot
		currentRobot = 1
		startAngles[0] = angleHolder[0]
		startDistance[0] = distanceHolder[0]
		for ll in range(0,angleCount - 1):
			if(robotAssignment[ll] != robotAssignment[ll+1]):
				currentRobot += 1
				finishAngles[currentRobot-2] = angleHolder[ll]
				finishDistance[currentRobot-2] = distanceHolder[ll]
				startAngles[currentRobot-1] = angleHolder[ll+1]
				startDistance[currentRobot-1] = distanceHolder[ll+1]
		finishDistance[currentRobot-1] = distanceHolder[-1]
		finishAngles[currentRobot-1] = angleHolder[-1]
		currentRobot =+ 1
		#checking that there are at least 4 data points for each robot
		dataPoints = 1
		robotDataPointCount = []
		for mm in range(1,angleCount-1):
			if(robotAssignment[mm] == robotAssignment[mm-1]):
				dataPoints += 1
			else:
				robotDataPointCount.append(dataPoints)
				dataPoints = 1
		robotDataPointCount.append(dataPoints)

		robotCount = len(robotDataPointCount)
		for nn in range(0,robotCount):
			if(robotDataPointCount[nn] > 2):
				startAnglesFinal.append(startAngles[nn])
				finishAnglesFinal.append(finishAngles[nn])
				startDistanceFinal.append(startDistance[nn])
				finishDistanceFinal.append(finishDistance[nn])

		#print(len(startAngles))
	return (startAnglesFinal,finishAnglesFinal,startDistanceFinal,finishDistanceFinal)



#this function takes in a list of angles and distances and attempts to find the center of the robot based on this data
#it is meant to find how many robots are in view and their repective positions (the center of the robot not the edge)
def ultrasonic(distances,angles):
	#print(angles)
	(startAngles,finishAngles,distanceStart,distanceFinish) = findRobots(distances,angles)
	#print(angles)
	startAnglesDeg =[i * 180/np.pi for i in startAngles]
	finishAnglesDeg =[j * 180/np.pi for j in finishAngles]
	#print(startAnglesDeg,finishAnglesDeg)
	#print(distanceStart,distanceFinish)
	#print(startAngles)
	robotAngle = []
	robotDistance = []
	botCenterXList = []
	botCenterYList = []
	if(startAngles):
		robotCount = len(startAngles)
		mode = 1
		print('robot count {:f}'.format(robotCount))
		for jj in range(0,robotCount):
			startX = distanceStart[jj]*np.cos(startAngles[jj])
			finishX = distanceFinish[jj]*np.cos(finishAngles[jj])
			startY = distanceStart[jj]*np.sin(startAngles[jj])
			finishY = distanceFinish[jj]*np.sin(finishAngles[jj])
			startCoord = (startX,startY)
			finishCoord = (finishX,finishY)
			(botCenterX,botCenterY) = findCenter(startCoord,finishCoord)
			#print('botCenterX {:f}'.format(botCenterX*.39))
			#print('botCenterY {:f}'.format(botCenterY*.39))
			botCenterXList.append(botCenterX)
			botCenterYList.append(botCenterY)
			robotAngle.append(np.arctan2(botCenterY,botCenterX))
			robotDistance.append(np.sqrt(botCenterX**2 + botCenterY**2))
			#print('robot angle deg {:f}'.format(robotAngle[-1]*180/np.pi))
			#print('robot distance {:f}'.format(robotDistance[-1]))
			#print(angles)
			#print(distances)
			#time.sleep(3)

	return (robotAngle,robotDistance,botCenterXList,botCenterYList)


