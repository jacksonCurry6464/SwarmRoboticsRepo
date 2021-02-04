#from IPython import get_ipython
#get_ipython().magic('reset -sf')

#
import time
import RPi.GPIO as GPIO
import numpy as np
import math
import os
import csv
import motorControl
import encoderReader
import movementAlgorithm
import positionCalc
import sensorDriver
import robotPosition
import IRLineFollower
import checkDistance

# Some user controlled parameters and robot geometry
numberOfRobots = 3
wheelDistance = 0.174625 # meters
circleRadius = 0.7
sensorRange = .8


#BELOW ARE THE FILES WHERE I AM WRITING THE DATA TO
TrialDataFile = open("Trial1Data.txt", "a")
#fmyDistanceWritingFile = open("frontAllDistancesNotJustClosest.txt", "a")              #THESE 2 FILES ARE WRITTEN TO INSIDE OF SENSOR DRIVER
#bmyDistanceWritingFile = open("backAllDistancesNotJustClosest.txt", "a")




#initializing some of the necessary variables for making sure timing works
currentTime = 0
previousTime = 0
startTime = time.time()
endTime = 60*60 #run for the given time in seconds


# setting up the encoder and its variables
previousLeadingEdgeRight = currentTime
previousLeadingEdgeLeft = currentTime

#initializing the wheel speed = 0 counters
zeroCountRight = 0
zeroCountLeft = 0

#this is the way that functins can identify if they are moving the left or right wheel when called
right = 1
left = 2
both = 3

#initializing the tuple that holds the information from the encoder
#(time of the previous leading, previous encoder input, number of times the encoder has read since changing, side value, speed)
#encoderRight = (previousLeadingEdgeRight,0,zeroCountRight,right,0)				#JACKSON HERE DOING THIS
#encoderLeft = (previousLeadingEdgeLeft,0,zeroCountLeft,left,0)

#calculating the speed relationship between the outside wheel (right) and inside wheel (left)
#leftRightRatio = (circleRadius*2 - wheelDistance)/(circleRadius*2 + wheelDistance)		#JACKSON HERE

#initializing the position tracker and estimated angle along the circle
position = [0,0,np.pi] #x,y,headingangle
timeLog = []
totalAngle = 0

#starting both of the wheels with a kick
motorControl.forward(both)
#leftIn = goalSpeed*2/(1 + 1/leftRightRatio)
#rightIn = goalSpeed*2/(1 + leftRightRatio)
leftIn = 0
rightIn = 0
Vright = 0
Vleft = 0
goalX = 0
goalY = 0
#setting up the for the algorithm
mode = -1
"""
This function relies heavily ont the use of modes each being:
	- mode 0: the robot is the leader it attempts to go in circles forever
	- mode 1: the robot is looking for another robot to be its leader
	- mode 2: the robot has found a leader and is moving into position

Unlike algorithm 4 the robots never fall back or become leaders. This is because the robots do not have high accuracy or percison in their speed distance or direction measurements, if we attempted to fallback and leave the sensor range the robots would quickly drift from eachother
"""


# there if there is a jumper on pin 2(reads HIGH) then it is the leader
#this will be used when we need to set leader before the start of the trial instead of using code
#currently this is not in use and line 92 is commented out before uploading to the leader
leaderPin = 2
#GPIO.setup(leaderPin,GPIO.IN)
isLeader = 1
#isLeader = GPIO.input(leaderPin)
#isLeader = 0
print(isLeader)
if(isLeader):
	mode = 0 #simple circle
else:
	mode = 1 #finding a leader

# some more variables for the algorithm
previousGoal = 1
side = -1
sideCanChange = 1

#this is the leader posiiotn in relative space
robotCenterX = [0]
robotCenterY = [0]
#this is the leader posiiotn in absolute space
leaderPositionX = [0]
leaderPositionY = [0]
#this is the time that the data was taken
robotPingTime = [0]

#continuos data holder
continousPositionX = [0]
continousPositionY = [0]
continousAngle = [0]
continousDistance = [0]

#initalalizing the variable that indicaates if there was a successful data reading/ a robot in range
robotInView = 0

positionLogX = [0]
positionLogY = [0]
positionLogAngle = [np.pi]

#describing which way the robot will move with
lineFollowing = 1
moveTics = 0

#recording convergence time
convergedAtTime = 0
outOfConvergedAtTime = 0
checkingAfterConverged = 1
angleErrorRange = 8

# every delaytime in seconds the robots move then take sensor data
delayTime = 20				#JACKSON HERE
#making sure that they all move at the same time
#this helps to prevent the robots starting at odd times when started from the commandline
while(np.floor(time.time()%delayTime)):
	pass
time.sleep(2)

################################################################################################################################
#Begin Initialize the algorithm
#Violet Mwaffo
#fixed parameters
K_gain, phi_gain, N, w_ref, T = 1, 3, 3 , 0.1, 2 #0.005, 2500 0.25/np.pi*180			#N here is Number of robots from what I understand

psi = 2*np.pi / numberOfRobots #target spacing
theta_max = 3*psi/4

K = K_gain*w_ref
phi = phi_gain*K

#first of all let us define the set of parameters
#so that the assumptions are verified
w_0 = np.zeros(N) #((N, 1))								#VECTOR CONTAINING THE ANGLES THE ROBOT WILL MOVE AT NEXT TURN
#omega_0 in the paper
w_0[0] = w_ref #the leader starts at w_ref
        
#T = 2500 # simulation time
w = np.zeros((N,T))                                        #IMPORTANT MATRIX 3x2
w_max = w_ref + K # max speed of agent
# what don I need this for?
###
w_ij = w_max - w_ref
# declare the state vector (it is a matrix because it evolves in time)
th = np.zeros((N,T))
diff_th = np.zeros((N,T)) 
        
alfa = np.zeros((N, N-1,T)) #
        
# computes the noise realizations for the entire simulation
noise = (2*np.random.rand(N, N-1,T)-1)*phi
        
temp = np.zeros((N, N-1,T)) 
        
#preallocation of the output matrix, inf denotes no measurement availabilty
y = np.zeros((N, N-1,T)) + np.inf 

###############################################################################
#this initialization is useless for robotic application
#make sure that the leader is placed at the end of the queue        
#initialize angular position
#th[:,0] = (np.random.rand(1,N) * 2 - 1) * np.pi
th[:,0] = np.array([-3.0515, 0.0904, 3.1124])
th[:,0] = np.sort(th[:,0]) # this sorting is unnecessary. However, it facilitates any data analysis 
diff_th = np.min(np.diff(th[:,0]))                                                                          #IN OUR CASE THIS SHOULD BE AN ARRAY FILLED WITH VALUES OF np.pi/6 (30 degree differences)

while diff_th <= 4*phi + 2*w_max:
    #th[:,0] = (np.random.rand(1,N) * 2 - 1) * np.pi
    th[:,0] = np.sort(th[:,0]) 
    diff_th = np.min(np.diff(th[:,0]))
#################################################################################
            
# stores the relative phases. Could be generalized through a for loop.
for kk in range(N-1): #for kk = 1:N-1
    temp[:,kk,0] = np.mod(th[:,0] - np.roll(th[:,0], -kk-1), 2*np.pi)
            
#computes the angular distances
alfa[:,:,0] = np.minimum.reduce([temp[:,:,0], 2*np.pi-temp[:,:,0]]) #% angular distance
        
#    #preallocation of the output matrix, inf denotes no measurement availabilty
y = np.zeros((N, N-1,T)) + np.inf
y_new = np.zeros((N,N-1)) + np.inf   								#HERE HE IS INITIALIZING A 3X2 ARRAY OF ZEROS
        
alfa_temp = alfa[:,:,0]
noise_temp = noise[:,:,0]
y_new[alfa[:,:,0] < theta_max] = alfa_temp[alfa_temp < theta_max] + noise_temp[alfa_temp < theta_max]
y[:,:,0] = y_new
del y_new, alfa_temp, noise_temp
     
# preallocation of the upsilon intervals. I do so as if I never measure and
#then insert the information from the available measurements
LB_ups = np.zeros((N, N-1,T)) + theta_max
UB_ups = np.zeros((N, N-1,T)) + np.pi
# this is the interval of positive values
Gamma_1_LB = np.zeros((N, N-1,T))
Gamma_1_UB = np.pi*np.ones((N, N-1,T))
# this is the interval of negative values
Gamma_2_LB = -np.pi*np.ones((N, N-1,T))
Gamma_2_UB = np.zeros((N, N-1,T))

LB_temp = np.amax([np.zeros((N , np.size(y[:,:,0]- phi,1))),y[:,:,0]- phi],0)
UB_temp = np.min([theta_max * np.ones((N,np.size(y[:,:,0]- phi,1))),y[:,:,0] + phi],0)
LB_new = np.zeros((N, N-1)) + theta_max
UB_new = np.zeros((N, N-1)) + np.pi
LB_new[alfa[:,:,0] < theta_max] = LB_temp[alfa[:,:,0] < theta_max]
UB_new[alfa[:,:,0] < theta_max] = UB_temp[alfa[:,:,0] < theta_max]
LB_ups[:,:,0] = LB_new
UB_ups[:,:,0] = UB_new
del LB_temp, UB_temp, LB_new, UB_new
        
# Compute the UB e LB and their hull for t = 1
# this is the interval of positive values
Gamma_1_LB[:,:,0] = LB_ups[:,:,0]
Gamma_1_UB[:,:,0] = UB_ups[:,:,0]
# this is the interval of negative values
Gamma_2_LB[:,:,0] = -UB_ups[:,:,0]
Gamma_2_UB[:,:,0] = -LB_ups[:,:,0]
    
# computes the Hull.
Hull_LB = 0 * Gamma_1_LB
Hull_UB = 0 * Gamma_1_UB

for ii in range(N):            
    Hull_LB[ii,:,0] = np.minimum(Gamma_1_LB[ii,:,0],Gamma_2_LB[ii,:,0])            
    Hull_UB[ii,:,0] = np.maximum(Gamma_1_UB[ii,:,0],Gamma_2_UB[ii,:,0])
            
## end of uncertainty
# at t=1 I cannot control as there is ambiguity on whether any agent
# follows or precedes agent i
        
w[:,0] = w_0                                                    #SETTING FIRST COLUMN OF THE ROBOT MATRIX TO BE EQUAL TO VALUE THEY SHOULD MOVE
# the leader AlWAYS travels at the reference speed
w[0,:] = w_ref                                                  #SETTING FIRST ROW CORRESPONDING TO THE LEADER ROBOT TO ALWAYS BE THE REFERENCE SPEED
        
# here I record whether an agent has or has not identified his follower
follower = np.zeros((N, 1), dtype=int)
ind_fun = np.zeros((N, 1))
        
stima = psi*np.ones((N, T)) + 0.0001
        
cond_10_a = np.zeros((N, N-1, T))
cond_10_b = np.zeros((N, N-1, T))
        
# preallocate the follower's hull
Hull_fol_LB = np.zeros((N, T))
Hull_fol_UB = np.zeros((N, T))
        
temp_3 = np.zeros((N, N-1, N-1))
########End Initialize the algorithm
#############################################################################################################

#############################################################################################################
#convert distance to phase angle based on radius
def convertDist(dist):
    val = np.mod(2*np.arcsin(0.5*dist/circleRadius), 2*np.pi) #estimated phase value
    if val > theta_max:
        return np.inf
    else:
        return val

#initial
angleBackFromLeader = 2*np.pi/numberOfRobots
inRangeCount = 0
timeAsleader = 0
convergeCount = 0
check = np.array([0,0,0]) 
iniTime = time.time()
# running the algorithm
iterationCounter = 0
while(currentTime < endTime):
    
    print("iterationCounter:")
    print(iterationCounter)
    iterationCounter = iterationCounter+1
    previousTime = currentTime
    currentTime = time.time() - startTime
    timeLog.append(currentTime)
    deltaT = currentTime - previousTime
    
################################################################################################################
###Violet Mwaffo
########################begin balancing formation pursuit ###################
########################sensor provide the value of y #####################
    
    if(isLeader):
        (frontRobotDistance, backRobotDistance) = sensorDriver.getData()                   #JACKSON HERE THIS NOW JUST RETURNS THE FRONT AND BACK ROBOT DISTANCES    
        print("Below is FrontRobotDistance Before Conversion:")
        print(frontRobotDistance)
        nonConvertedFrontDistance = frontRobotDistance
        nonConvertedBackDistance = backRobotDistance
        backRobotDistance = convertDist(backRobotDistance) #convert the sensor distance in phase angle based on circle radius 
        frontRobotDistance = convertDist(frontRobotDistance) #convert the sensor distance in phase angle based on circle radius  
        print("Below is frontRobotDistance after Conversion:")
        print(frontRobotDistance)
        w[0,1] = w_ref 
        th[:,1] = th[:,0] + w[:,0] 

    else:#finding the distance to leader
        (frontRobotDistance, backRobotDistance) = sensorDriver.getData()					#JACKSON HERE THIS NOW JUST RETURNS THE FRONT AND BACK ROBOT DISTANCES
        nonConvertedFrontDistance = frontRobotDistance
        nonConvertedBackDistance = backRobotDistance
        backRobotDistance = convertDist(backRobotDistance) #convert the sensor distance in phase angle based on circle radius 
        frontRobotDistance = convertDist(frontRobotDistance) #convert the sensor distance in phase angle based on circle radius  
        
        # system dynamcs and measurement generation 
        deltaTH = np.zeros(N)
        deltaTH[0] = w_ref
    #    time_elpse = currentTime - iniTime
        if backRobotDistance < np.inf:
            deltaTH[1] = backRobotDistance #phase between robot and its leader (robot pushing it)
        else:
            if follower[1] == 0: #robot has not identified its leader
                deltaTH[1] = 0 #can occur only at begining when the robot has not idetified its follower
            else:
                deltaTH[1] = w_ref #robot is moving at constant speed
            
        if frontRobotDistance < np.inf:
            deltaTH[2] = frontRobotDistance #phase between robot and its follower (robot that it is pushing)
        else:
            if follower[1] == 0: #robot has not identified its leader
                deltaTH[2] = 0 #can occur only at begining when the robot has not idetified its follower
            else:
                deltaTH[2] = w_ref #robot is moving at constant speed
        
        #th[:,1] = th[:,0] + w[:,0] #theoretical value from simulations
        th[:,1] = th[:,0] + deltaTH #true value using feedback from sensor 
        #print("TH[] is BELOW:")
        #print(th)    
        for kk in np.arange(0,N-1,1):        
            temp[:,kk,1] = np.mod(th[:,1] - np.roll(th[:,1], -kk-1), 2*np.pi)	# Updating times:    
        
        
        alfa[:,:,1] = np.minimum(temp[:,:,1],2*np.pi-temp[:,:,1])
        y_new = np.zeros((N,N-1))+np.inf
        alfa_temp = alfa[:,:,1]
        noise_temp = noise[:,:,1] * 0 #no noise for now
        y_new[alfa[:,:,1] < theta_max] = alfa_temp[alfa_temp < theta_max] + noise_temp[alfa_temp < theta_max]
        y[:,:,1] = y_new
        del y_new, alfa_temp, noise_temp
        ###########################################################################
        print("ALfA BELOW")
        print(alfa)
        print("Y Below")
        print(y[:,:,1])
        LB_temp = np.amax([np.zeros((N , np.size(y[:,:,1]- phi,1))),y[:,:,1]- phi],0)
        UB_temp = np.amin([theta_max * np.ones((N,np.size(y[:,:,1]- phi,1))),y[:,:,1] + phi],0)
        LB_new = np.zeros((N, N-1)) + theta_max
        UB_new = np.zeros((N, N-1)) + np.pi
        LB_new[alfa[:,:,1] < theta_max] = LB_temp[alfa[:,:,1] < theta_max]
        UB_new[alfa[:,:,1] < theta_max] = UB_temp[alfa[:,:,1] < theta_max]
        LB_ups[:,:,1] = LB_new
        UB_ups[:,:,1] = UB_new
        del LB_temp, UB_temp, LB_new, UB_new
        
        for ii in np.arange(1,N,1):#range(1,N,1):  
            print("PRINTING VALUE OF FOLLOWER:")
            print(follower)
            if follower[ii] == 0: # if an agent hasn't identified its follower,
                # it cannot exclude any alternative for the remainder of the agents' speeds
                Gamma_1_LB[ii, alfa[ii,:,1] > theta_max, 1] = theta_max
                Gamma_1_UB[ii, alfa[ii,:,1] > theta_max, 1] = np.pi
                Gamma_2_LB[ii, alfa[ii,:,1] > theta_max, 1] = - np.pi
                Gamma_2_UB[ii, alfa[ii,:,1] > theta_max, 1] = - theta_max
                
                # First I perform the projection.
                # if ii stands still, ii-1 other agents can get closer, none
                # can get farther
                Gamma_1_LB[ii, alfa[ii,:,1] <= theta_max, 1] = np.maximum(Gamma_1_LB[ii, alfa[ii,:,1] <= theta_max, 0] - w_max, 0)
                Gamma_1_UB[ii, alfa[ii,:,1] <= theta_max, 1] = Gamma_1_UB[ii, alfa[ii,:,1] <= theta_max, 0]
                Gamma_2_LB[ii, alfa[ii,:,1] <= theta_max, 1] = np.maximum(Gamma_2_LB[ii, alfa[ii,:,1] <= theta_max, 0]-w_max,-theta_max)
                Gamma_2_UB[ii, alfa[ii,:,1] <= theta_max, 1] = Gamma_2_UB[ii, alfa[ii,:,1] <= theta_max, 0]
                # Now I perform the intersection
                Gamma_1_LB[ii, alfa[ii,:,1] <= theta_max, 1] = np.maximum(Gamma_1_LB[ii, alfa[ii,:,1] <= theta_max, 1],LB_ups[ii, alfa[ii,:,1] <= theta_max, 1])
                Gamma_1_UB[ii, alfa[ii,:,1] <= theta_max, 1] = np.minimum(Gamma_1_UB[ii, alfa[ii,:,1] <= theta_max, 1],UB_ups[ii, alfa[ii,:,1] <= theta_max, 1])
                Gamma_2_LB[ii, alfa[ii,:,1] <= theta_max, 1] = np.maximum(Gamma_2_LB[ii, alfa[ii,:,1] <= theta_max, 1],-UB_ups[ii, alfa[ii,:,1] <= theta_max, 1])
                Gamma_2_UB[ii, alfa[ii,:,1] <= theta_max, 1] = np.minimum(Gamma_2_UB[ii, alfa[ii,:,1] <= theta_max, 1],-LB_ups[ii, alfa[ii,:,1] <= theta_max, 1])            
                          
                # computes the Hull.
                temp_2 = Gamma_2_LB[ii,:,1] < Gamma_2_UB[ii,:,1]
                Hull_LB[ii,:,1] = Gamma_1_LB[ii,:,1]
                Hull_LB[ii,temp_2,1] = Gamma_2_LB[ii,temp_2,1]
                temp_2 = Gamma_1_UB[ii,:,1] > Gamma_1_LB[ii,:,1]
                Hull_UB[ii,:,1] = Gamma_2_UB[ii,:,1]
                Hull_UB[ii,temp_2,1] = Gamma_1_UB[ii,temp_2,1]
                del temp_2            
                
                ## I compute condition 10_a
                # this basically checks if the negative interval is empty
                cond_10_a[ii,:,1] = (Gamma_2_LB[ii,:,1] >= Gamma_2_UB[ii,:,1]).astype(np.int)
                # if the condition has been verified at a previous instant it is still verified
                cond_10_a[ii, cond_10_a[ii,:,0] == 1, 1] = 1
                # I use this variable to check if agent ii can discern if jj is ahead of any other agent
                temp_11 = np.zeros((N-1,N-1))            
                
                for jj in np.arange(0,N-1,1):                 
                    temp_11[jj,:] = Hull_UB[ii,jj,1] - Gamma_1_LB[ii,:,1]
                    temp_3[ii, jj, temp_11[jj,:] < 0]=1
                
                # here a one means that condition (10b) is fulfilled for agent jj
                cond_10_b[ii,:,1] =  np.sum(temp_3[ii,:,:], axis=1) == N-2            
                
                ## Violet beware of this caveat
                # remember that in the second dimension, the last element
                # is ii-1 then ii-2 and so on...
                temp_13 = cond_10_a[ii,:,1] * cond_10_b[ii,:,1]
                # note that this should always be 3
                #print("Condition 10 A:")
                #print(cond_10_a)
                #print("Condition 10 B:")
                #print(cond_10_b)
                #print("TEMP 13 BELOW!")
                #print(temp_13)
                temp_13 = np.nonzero(temp_13)
                
                if np.sum(temp_13) > 0:
    #                print( "Empty" )
    #            else:
                    #
                    follower[ii] = 1         #temp_13       CHANGING THIS OUT WITH 1!!!
                    # at this time instant agent i must apply the control law,
                    # moreover, the variable Hull_fol_LB[ii,1], must be created
                    Hull_fol_LB[ii,1] = Hull_LB[ii,follower[ii],1]
                    Hull_fol_UB[ii,1] = Hull_UB[ii,follower[ii],1]
                    stima[ii,1] = Hull_fol_UB[ii,1]
                    w[ii,1] = w_ref + K*(stima[ii,1]<=psi)
                    #print("THE VALUES OF W ARE BELOW:")
                    #print(w)       
                
            else: ## if ii has already identified its follower
                
                if y[ii,follower[ii],1] < np.inf: # and ii-1 is still in the visual cone of i
                    
                    # first I perform the projection
                    # my follower cannot be faster
                    Hull_fol_LB[ii,1] = Hull_fol_LB[ii,0]
                    # but I cannot exclude he be slower
                    Hull_fol_UB[ii,1] = Hull_fol_UB[ii,0] + w[ii,0] - w_ref
                    
                    # then I perform the correction
                    Hull_fol_LB[ii,1] = np.maximum(Hull_fol_LB[ii,1], LB_ups[ii,follower[ii],1])
                    Hull_fol_UB[ii,1] = np.minimum(Hull_fol_UB[ii,1],UB_ups[ii,follower[ii],1])                
                    
                else: ## and ii-1 is outside the visual cone of ii                
                    # then only the projection is performed
                    # I know my follower's speed is w_ref
                    Hull_fol_LB[ii,1] = Hull_fol_LB[ii,0] + w[ii,0] - w_ref
                    Hull_fol_UB[ii,1] = Hull_fol_UB[ii,0] + w[ii,0] - w_ref
                    
                
                stima[ii,1] = Hull_fol_UB[ii,1]        
            
            # now the control action is applied
            if (follower[ii] == 0):
                
                w[ii,1] = w_0[ii]
                
            else:
                
                w[ii,1] = w_ref + K*(stima[ii,1] <= psi)  
        
           
    #reinitialize var
    th[:,0] = th[:,1] 
    w[:,0] = w[:,1]
    #reinitialize variables            
    Gamma_1_LB[:, :, 0] = Gamma_1_LB[:, :, 1]
    Gamma_1_UB[:, :, 0] = Gamma_1_UB[:, :, 1]
    Gamma_2_LB[:, :, 0] = Gamma_2_LB[:, :, 1]
    Gamma_2_UB[:, :, 0] = Gamma_2_UB[:, :, 1]
    cond_10_a[:,:,0] = cond_10_a[:,:,1]
    Hull_fol_LB[:,0] = Hull_fol_LB[:,1]
    Hull_fol_UB[:,0] = Hull_fol_UB[:,1]    

    #check convergence
    check = np.abs(np.diff(th[:,-1]) - psi) < K
    check = np.append(check,np.abs((2*np.pi-(th[-1,-1] - th[0,-1])) - psi) < (N-1)*K)  
    '''
    if np.sum(check) == 3 and isLeader == 0:
        #print("conv",check, "time", time.time() - iniTime) 
    '''
        
########End balancing formation pursuit algorithm 
################################################################################################################
    
    
	#print(isLeader)
    if(not isLeader):
        if nonConvertedFrontDistance<0.4:
            print("ROBOT IN FRONT TOO CLOSE, NO MOVEMENT")
            angleToMove = 0
        else:
            angleToMove = w[1,1]
        print("angle for the robot to move is below:")
        print(angleToMove)
    elif(isLeader):
        if nonConvertedFrontDistance<0.4:
            print("ROBOT IN FRONT TOO CLOSE, NO MOVEMENT")
            angleToMove = 0
        else:
            angleToMove = w[0,1]
        print("leaders angle to move is below:")
        print(angleToMove)
    leftOverTime = time.time()
    while(np.floor(time.time()%delayTime)):
        pass
    print('leftover time {:f}'.format(time.time() - leftOverTime))
    
	#moving the robot
    if(lineFollowing):
        IRLineFollower.moveAngle(angleToMove,circleRadius)
    if(lineFollowing):
        (changeX,changeY,totalAngle) = positionCalc.changeInPositionLineFollowing(angleToMove,wheelDistance,circleRadius,totalAngle,position)

    position[0] += changeX
    position[1] += changeY
    position[2] = totalAngle
    positionLogX.append(position[0])
    positionLogY.append(position[1])
    positionLogAngle.append(position[2])

	#more data saving just with a different purpose
	#this just saves the time it takes to enter and leave convergence
    """
    with open('convergeTime.csv','a') as file:
		data = csv.writer(file,delimiter = ',')
		dataOut = [str(convergedAtTime),str(outOfConvergedAtTime)]#,str(timeLog[-1]),str(leaderPositionX),str(leaderPositionY)]
		data.writerow(dataOut)
	"""
    with open('movementTracker.csv','a') as file:
        data = csv.writer(file,delimiter = ',')
        dataOut = [str(angleToMove)]#,str(timeLog[-1]),str(leaderPositionX),str(leaderPositionY)]
        data.writerow(dataOut)
	#print('time elapsed {:f} to finish'.format(time.time() - startTime - currentTime))

print('End due to time')
