import time
import RPi.GPIO as GPIO
import numpy as np
import math
import os
import csv
import encoderReader
GPIO.setwarnings(False)          # do not show any warnings
GPIO.setmode (GPIO.BCM)

#setting up the motor
motor1Pin = 17
in1 = 27
in2 = 22
in3 = 18
in4 = 23
motor2Pin = 24

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(motor1Pin,GPIO.OUT)
GPIO.setup(motor2Pin,GPIO.OUT)

rightMotor = GPIO.PWM(motor1Pin,1000)
leftMotor = GPIO.PWM(motor2Pin,1000)

leftMotor.start(0)
rightMotor.start(0)

right = 1
left = 2
both = 3

def forward(motor):
	if motor == left:
		GPIO.output(in1,GPIO.HIGH)						#HIGH,LOW,HIGH,LOW
		GPIO.output(in2,GPIO.LOW)
	elif motor == right:
		GPIO.output(in3,GPIO.HIGH)
		GPIO.output(in4,GPIO.LOW)
	elif motor == both:
		GPIO.output(in1,GPIO.HIGH)
		GPIO.output(in2,GPIO.LOW)
		GPIO.output(in3,GPIO.HIGH)
		GPIO.output(in4,GPIO.LOW)
	return 0

#setting a motor to a given duty cycle based on the PWM signal
def setSpeed(speed,motor):
	speed = speed * 100
	if(speed > 100):
		speed = 100
	if motor == left:
		leftMotor.ChangeDutyCycle(speed)

	elif motor == right:
		rightMotor.ChangeDutyCycle(speed)

	elif motor == both:
		leftMotor.ChangeDutyCycle(speed)
		rightMotor.ChangeDutyCycle(speed)
	return speed





