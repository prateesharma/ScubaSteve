import serial
f= open("liquidSensor.txt","wb")
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
while True:
    data = arduino.readline()[:-2] #the last bit gets rid of the new-line chars
    if data:
        f.write(data)
