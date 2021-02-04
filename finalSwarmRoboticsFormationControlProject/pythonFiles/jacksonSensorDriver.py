import time
import RPi.GPIO as GPIO
import pigpio
import numpy as np
import math
import os
import csv
from jacksonCameraDist import takePicAndFindDistance
from pingUltra import pingUltrasonic
from jacksonlaser import jacksonPingIR

def findMin(myArray):							#helper function that can find the minimum value in an input array 
	minimum = 10000								#setting minimum to be arbitrarily large number
	for x in myArray:
		#print(x)
		if x<minimum:
			minimum = x
	return minimum

def findMedian(myArray):				#CHECK TO MAKE SURE THAT THE ARRAYS THAT ARE GETTING SENT INTO THIS FUNCTION HAVE AT LEAST 1 VALUE
	myArray.sort()
	lengthArray = len(myArray)
	if lengthArray%2 and lengthArray:				#This means it is odd if it goes in here
		targetIndex = (lengthArray/2)-0.5
		targetIndex= int(targetIndex)
		#print(targetIndex)
		return targetIndex
	elif not lengthArray%2:			#This means it is even and can't find the exact center so we will arbitrarily choose the value in the middle more towards the bottom
		targetIndex = (lengthArray/2)-1
		targetIndex = int(targetIndex)
		#print(targetIndex)
		medianNumber = myArray[targetIndex]
		return medianNumber

def pruneList(myArray):				#This will get rid of 0 values and other weird ones we don't want
	lengthArray = len(myArray)
	#print(lengthArray)
	iterator = 0
	for x in range(lengthArray):
		if myArray[iterator] == 0 or myArray[iterator] >1:
			#print("delete")
			del myArray[iterator]
		else:
			#print("iterate")
			iterator = iterator+1
		#print(iterator)
		#print(myArray)
	return myArray

pi = pigpio.pi()
def getData():
	ultrasonic = 1
	picture = 2
	IRLaser = 3
	mode = ultrasonic						#ONLY HAVE 1 of these uncommented and whichever one is uncommented will be the mode that runs
	#mode = picture
	#mode = IRLaser
	#OPENING A NEW FILE HERE
	fmyDistanceWritingFile = open("frontAllDistancesNotJustClosest.txt", "a")
	bmyDistanceWritingFile = open("backAllDistancesNotJustClosest.txt", "a")
	

	if mode == ultrasonic:
		endTime = time.time()+16					#16 IS AN OVERESTIMATE OF HOW LONG IT WILL TAKE TO GET THE DISTANCES THAT SHOULD BE DECREASED IN THE FUTURE
		distRobotFront = np.inf					#here I initialize the two values that will be returned at the end to arbitrarily high values
		distRobotBack = np.inf
		frontMedianList = []
		backMedianList = []
		frontList = []						#this array will contain values where we are looking for the robot in front
		backList = []						#this array will contain the values where we are looking for the robot behind
		loopCounter = 0

		while(time.time()<endTime):
			
				
			if loopCounter == 0:
				servoPin = 19
				initialServoValue = 2450
				time.sleep(1)

				for m in range(8):
					pi.set_servo_pulsewidth(servoPin, initialServoValue)
					time.sleep(0.2)
					for x in range(10):
						myDistance = pingUltrasonic()
						if myDistance<1 and myDistance>0.05:
							print("Front, Distance on this trial is between 0.05 and 1 meters: ", myDistance)
							frontMedianList.append(myDistance)
						else:
							print("No robot seen this measurement")
					if len(frontMedianList)>0:
						myMedian = findMedian(frontMedianList)
						frontList.append(myMedian)

					initialServoValue = initialServoValue-50
					frontMedianList = []							#Reinitializing this array for the next time we loop through here

				initialServoValue = 550					#SETTING THE SERVO VALUE TO NOW BE LOOKING THE OTHER WAY AND THEN THE VALUE WILL GO UP FROM HERE
				time.sleep(1)

				for y in range(8):
					pi.set_servo_pulsewidth(servoPin, initialServoValue)
					time.sleep(0.2)
					for x in range(10):
						myDistance = pingUltrasonic()
						if myDistance<1 and myDistance>0.05:
							print("Back, Distance on this trial is between 0.05 and 1 meters: ", myDistance)
							backMedianList.append(myDistance)
						else:
							print("No robot seen this measurement")

					
					if len(backMedianList)>0:
						myMedian = findMedian(backMedianList)
						backList.append(myMedian)

					initialServoValue = initialServoValue+50
					backMedianList = []
				#print("FRONT LIST BELOW")
				#print(frontList)
				#print("BACK LIST BELOW")
				#print(backList)

				#BELOW IS WHERE I AM WRITING TO THE TEXT FILE WHERE I AM SAVING ALL OF THE 				
				for x in range(len(frontList)):
					stringforFile = "{}".format(frontList[x])
					fmyDistanceWritingFile.write(stringforFile+"\n")

				for x in range(len(backList)):
					stringforFile = "{}".format(backList[x])
					bmyDistanceWritingFile.write(stringforFile+"\n")	




				
				backList = pruneList(backList)

				if frontList == []:
					distRobotFront = np.inf
				else:
					frontList = pruneList(frontList)
					if frontList == []:
						distRobotFront = np.inf
					else:
						distRobotFront = findMin(frontList)

				if backList == []:
					distRobotBack = np.inf
				else:
					backList = pruneList(backList)
					if backList == []:
						distRobotBack = np.inf
					else:
						distRobotBack = findMin(backList)

				


			loopCounter = loopCounter+1
			time.sleep(0.003)
		loopCounter = 0
		
		print("robot in front is this far away:")
		print(distRobotFront)
		print("robot behind is this far away:")
		print(distRobotBack)
		return (distRobotFront, distRobotBack)



	elif mode == picture:
		endTime = time.time()+16					#16 IS AN OVERESTIMATE OF HOW LONG IT WILL TAKE TO GET THE DISTANCES THAT SHOULD BE DECREASED IN THE FUTURE
		distRobotFront = 10000					#here I initialize the two values that will be returned at the end to arbitrarily high values
		distRobotBack = 10000
		firstList = []						#this array will contain values where we are looking for the robot in front
		secondList = []						#this array will contain the values where we are looking for the robot behind
		loopCounter = 0
		while(time.time()<endTime):
			if loopCounter == 0:
				servoPin = 19
				pi.set_servo_pulsewidth(servoPin,2475)			#Here I check the first angle which is straight ahead
				time.sleep(0.5)
				if mode == ultrasonic:
					distance1 = pingUltrasonic()
				elif mode == picture:
					distance1 = takePicAndFindDistance()
				elif mode == IRLaser:
					distance1 = jacksonPingIR()
				firstList.append(distance1)

				pi.set_servo_pulsewidth(servoPin, 2350)			#move servo to next angle to take measurement
				time.sleep(0.5)
				if mode == ultrasonic:
					distance2 = pingUltrasonic()
				elif mode == picture:
					distance2 = takePicAndFindDistance()
				elif mode == IRLaser:
					distance2 = jacksonPingIR()
				firstList.append(distance2)

				pi.set_servo_pulsewidth(servoPin,2225)			#move servo to third angle in front to check
				time.sleep(0.5)
				if mode == ultrasonic:
					distance3 = pingUltrasonic()
				elif mode == picture:
					distance3 = takePicAndFindDistance()
				elif mode == IRLaser:
					distance3 = jacksonPingIR()
				firstList.append(distance3)

				pi.set_servo_pulsewidth(servoPin,525)			#move servo to looking directly behind to check
				time.sleep(0.5)
				if mode == ultrasonic:
					distance4 = pingUltrasonic()
				elif mode == picture:
					distance4 = takePicAndFindDistance()
				elif mode == IRLaser:
					distance4 = jacksonPingIR()
				secondList.append(distance4)

				pi.set_servo_pulsewidth(servoPin,650)			#move servo to looking directly behind to check
				time.sleep(0.5)
				if mode == ultrasonic:
					distance5 = pingUltrasonic()
				elif mode == picture:
					distance5 = takePicAndFindDistance()
				elif mode == IRLaser:
					distance5 = jacksonPingIR()
				secondList.append(distance5)

				pi.set_servo_pulsewidth(servoPin,775)			#move servo to looking directly behind to check
				time.sleep(0.5)
				if mode == ultrasonic:
					distance6 = pingUltrasonic()
				elif mode == picture:
					distance6 = takePicAndFindDistance()
				elif mode == IRLaser:
					distance6 = jacksonPingIR()
				secondList.append(distance6)

				distRobotFront = findMin(firstList)
				distRobotBack = findMin(secondList)



			loopCounter = loopCounter+1
			time.sleep(0.003)
		loopCounter = 0
		
		print("robot in front is this far away:")
		print(distRobotFront)
		print("robot behind is this far away:")
		print(distRobotBack)
		return (distRobotFront, distRobotBack)

	


#getData()