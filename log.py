import serial

serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)

f1 = open('log.txt', 'w')
line = s.readline().decode()
print(line)
while line != 'finished\r\n':
    f1.write(line)
    line = s.readline().decode()
    print(line)
f1.write(line)
f1.close()