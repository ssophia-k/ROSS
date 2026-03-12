import RPi.GPIO as GPIO 
import time

IN1 = 3
IN2 = 2
EEP = 4


GPIO.setmode(GPIO.BCM)
GPIO.setup(IN2, GPIO.OUT) # GPIO BCM 2, IN2
GPIO.setup(IN1, GPIO.OUT) #GPIO BCM 3, IN1
#GPIO.setup(EEP, GPIO.OUT) # GPIO BCM 4, EEP


for i in range(2):
	GPIO.output(IN2, False)
	GPIO.output(IN1, True)
	time.sleep(3)
	
	GPIO.output(IN2, True)
	GPIO.output(IN1, False)
	time.sleep(3)

GPIO.cleanup()
