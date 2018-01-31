import RPi.GPIO as GPIO
import time
import sys

GPIO.setmode(GPIO.BOARD)
pwmPin1 = 7
pwmPin2 = 12

param = sys.argv

GPIO.setup(pwmPin1, GPIO.OUT)
GPIO.setup(pwmPin2, GPIO.OUT)

pwm1 = GPIO.PWM(pwmPin1, 100)
pwm2 = GPIO.PWM(pwmPin2, 100)

pwm1.start(float(param[1]))
pwm2.start(float(param[2]))

time.sleep(1)

pwm1.stop()
pwm2.stop()
GPIO.cleanup()
