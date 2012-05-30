from serial import *
c=Serial(0)
c.write('com port support from Python')
if c.inWaiting():
print c.read( c.inWaiting() ),'characters read'
else:
print 'no reply'
c.close()

//The code opened COM1 @ default setting 9600,8,N,0

from serial import *
c=Serial(,19200,8,'E',1)

# com port opened at 19200 baud, 8 databits, even parity, 1 stopbit
c.close()i
