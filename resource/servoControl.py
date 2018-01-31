import serial
import time

ser0 = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(1)
ser1 = serial.Serial('/dev/ttyACM1', 9600)
time.sleep(1)

var0 = raw_input('motor0: ')
var1 = raw_input('motor1: ')

if(var0 == ''):
	var0 = '90'
if(var1 == ''):
	var1 = '90'

str0 = var0 + chr(ord('\r')) + chr(ord('\n'))
str1 = var1 + chr(ord('\r')) + chr(ord('\n'))

ser0.write(str0)
time.sleep(1)
ser1.write(str1)
time.sleep(1)

ser0.close()
ser1.close()
