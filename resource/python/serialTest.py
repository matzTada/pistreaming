import serial
ser = serial.Serial('/dev/ttyAMA0', 9600)

try:
	while True:
		instr = raw_input('input command >>> ')
		instr += "\n"
		ser.write(instr)
finally:
	ser.close()
	exit()
