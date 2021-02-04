import cv2
import numpy as np
import imutils
import picamera
import time

start = time.time()

#print("hello")

def findSquare(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lowerOrange = np.array([170,130,130])				#OLD numbers: 0,180,0
    upperOrange = np.array([179,255,255])			#OLD numbers: 40,255,255

    mask = cv2.inRange(hsv, lowerOrange, upperOrange)
    #below I will eliminate false negatives to make the image of the orange square clearer
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,kernel)
    mask = cv2.erode(mask,kernel,iterations = 1)
    mask = cv2.dilate(mask,kernel,iterations = 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)    

    #now I will create an edged version of the final mask
    edged = cv2.Canny(mask, 35, 125)

    finalEdge = cv2.findContours(edged.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    finalEdge = imutils.grab_contours(finalEdge)
    if finalEdge:										#checking to make sure there is some orange square on the screen
        c = max(finalEdge,key = cv2.contourArea)
        return cv2.minAreaRect(c)
    else:
        return -1
    #cv2.imshow('finalEdge',finalEdge)
    #cv2.imshow('mask', mask)
    #cv2.imshow('edged', edged)
    #cv2.imshow("image",image)

#returns distance in centimeters
def DistanceHelper(knownHeight, focalLength, perHeight):
    distance = (knownHeight*focalLength)/perHeight
    return distance

def takePicAndFindDistance():
    #below I do the calibration picture which should stay the same every time (YOU NEED THIS IMAGE TO BE IN THE FOLDER YOU ARE USING!!)
    knownDistance = 60.96		#this is 2 ft converted to cm. 
    knownHeight = 10.16			#height of marker converted to cm.
    calibrationPic = cv2.imread("2ft.jpg")
    marker = findSquare(calibrationPic)
    focalLength = (marker[1][1]*knownDistance)/knownHeight #made this 1-1 to access parameters of minAreaRect
							   #parameters are (center(x,y), (width,height), angle of rotation)


   #now taking a picture and finding its distance
    myCamera = picamera.PiCamera()
    myCamera.vflip=True			#this is flipping the image which may or may not be necessary
    myCamera.capture("newPic.jpg")
    myCamera.close()			#add in the close() function to free up the picamera for the next picture

    #picture is now taken (maybe add in a sleep time before imreading??)
    newPicture = cv2.imread("newPic.jpg")
    marker = findSquare(newPicture)
    if marker == -1:
        print("no square in sight")
        return 100000
    centimeters = DistanceHelper(knownHeight, focalLength, marker[1][1])


    box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
    box = np.int0(box)
    cv2.drawContours(newPicture, [box], -1, (0, 255, 0), 2)
    #cv2.putText(newPicture, "%.2fft" % (inches / 12),(newPicture.shape[1]-200, newPicture.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX,2.0,(0,255,0),3)
    #cv2.imshow("newPicture", newPicture)		#PUT BACK IN FOR TESTING
    #cv2.waitKey(0)
    
    #print("centimeters:")
    #print (centimeters)
    time.sleep(0.1)
    print("ORANGE SQUARE!!!")
    meters = centimeters/100
    print(meters)
    return meters

#takePicAndFindDistance()

