import math
import serial
import time


#Setup GPS
#UART.setup("UART2")
ser = serial.Serial('/dev/ttyAMA0',9600)
ser.close()
ser.open()
ser.write(b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n")
#ser.write(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
time.sleep(0.5)
ser.write(b"$PMTK300,100,0,0,0,0*2C\r\n")
time.sleep(0.5)
ser.write(b"$PMTK251,57600*2C\r\n")
time.sleep(0.5)
ser.baudrate=57600
ser.flushInput()
ser.flushInput()
print('GPS Initialized!')


def get_GPS():
	ser.flushInput()
	ser.flushInput()
	line=ser.readline()
	#print(line)
	#print(len(line))
#	while line[3]+line[4]+line[5]<>'GGA':
#		line=ser.readline()
	if len(line)>=43:
		if line[43]!=0:#line[43]=='1' or line[43]=='2':
			try:
				line = line.decode("utf-8")
				commas=list(range(0,14))
				k=0
				for i in range(0,len(line)-1):
					if line[i]==',':
						commas[k]=i
						k=k+1
				Time=float(line[commas[0]+1:commas[1]])
				alt=float(line[commas[8]+1:commas[9]])
				#alt=0.1*float(line[commas[9]-1])
				#altstr=line[commas[8]+1]
				#for i in range(commas[8]+2,commas[9]-1):
				#	altstr=altstr+line[i]
				#alt=float(altstr)+alt
			#print alt
			#print line
				deg_lat=float(line[18]+line[19])
				min_lat=float(line[20]+line[21])+0.0001*float(line[23]+line[24]+line[25]+line[26])  #23 is a "."
				deg_long=float(line[30]+line[31]+line[32])
				min_long=float(line[33]+line[34])+0.0001*float(line[36]+line[37]+line[38]+line[39]) #35 is a "."
				if line[28]=='N':
					lat_sign=1.0
				if line[28]=='S':
					lat_sign=-1.0
				if line[41]=='E':
					long_sign=1.0
				if line[41]=='W':
					long_sign=-1.0
				lat=lat_sign*(deg_lat*math.pi/180.0 + min_lat*math.pi/10800.0)
				longi=long_sign*(deg_long*math.pi/180.0 + min_long*math.pi/10800.0)
				return [lat*180/math.pi,longi*180/math.pi,alt,Time]
			except:
				return [0,0,0,0]
	return [0,0,0,0]  #null


def reinit():
	ser.write(b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n")
	#ser.write(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
	time.sleep(0.01)
	ser.write(b"$PMTK300,100,0,0,0,0*2C\r\n")
	time.sleep(0.01)
	ser.write(b"$PMTK251,57600*2C\r\n")
	time.sleep(0.01)
	ser.flushInput()
	ser.flushInput()
	print('Searching for GPS Lock...')