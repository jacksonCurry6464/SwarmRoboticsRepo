import pigpio

pi = pigpio.pi()
servoPin = 19
pi.set_servo_pulsewidth(servoPin, 1500)
#1500,
#1200,
#1000
#700
