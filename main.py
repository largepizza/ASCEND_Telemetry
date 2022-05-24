##ASCEND SPRING 2022 RASPBERRY PI 4 CODE

#MAVLINK
from __future__ import print_function
from builtins import object
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
#System
import time
import atexit
import serial
from subprocess import Popen, PIPE #for safe sudo shutdown
import numpy as np
import math
##Sensors
#IMU
import getopt, sys
#import rcpy 
#import rcpy.mpu9250 as mpu9250
#IMU
import imu
#GPS
import gps
#LED (and GPIO)
import RPi.GPIO as GPIO

#file writing
import uuid
import os
import shutil

print('All packages successfully initialized.')
GPIO.setup(26, GPIO.OUT)



class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)
        
        
#constants
Ts = 0.5
temp_high = 80
temp_low = 50
#globals
t0_mono = 0
last_t = 0

#shutsdown the board
def shutdown_board():
    sudo_password = 'temppwd'
    command = 'shutdown -h now'.split()
    
    p = Popen(['sudo', '-S'] + command, stdin=PIPE, stderr=PIPE,
              universal_newlines=True)
    sudo_prompt = p.communicate(sudo_password + '\n')[1]


#Ensures GPIO pins don't get stuck on shutdown
def exit_handler():
    GPIO.output(26, False)
    print('Shutting Down')
    
    
def writeToFile(path, time_mono, data):
    time_mono = time_mono - t0_mono
    f = open(path,'a')
    f.write('%s' % time_mono)
    for i in data:
        f.write(', %s' % i)
    f.write('\n')
    f.close()
    
def writeLineToFile(path, data):
    time_mono = time.monotonic() - t0_mono
    f = open(path,'a')
    f.write('%s: ' % time_mono)
    f.write('%s' % data)
    f.write('\n')
    f.close()

atexit.register(exit_handler)
print('Exit Register Set')


def main():
    #gpio toggles
    #function locals

    

    
    #get current device time
    global t0_mono
    t0_mono = time.monotonic()
    global last_t
    last_t = t0_mono
    gps_data_last = [0, 0, 0, 0]
    final_t = time.monotonic()
    
    #create directory
    
    
    #directory = uuid.uuid4().hex
    #directory = 'dir' + directory[0:9]
    
    directory = 'test'
    
    parent_dir = "/home/pi/Desktop/ASCEND/data"
    # Path
    main_path = os.path.join(parent_dir, directory)
    try:
        shutil.rmtree(main_path)
        print('Path %s removed' % main_path)
    except OSError as e:
        print("Error: %s : %s" % (main_path, e.strerror))
    os.mkdir(main_path)
    print("Directory '% s' created" % directory)

    ##Files
    #Log
    log_path = os.path.join(main_path,'log.txt')
    writeLineToFile(log_path, 'Startup Successful')
    writeLineToFile(log_path, 't0_mono = %s' % t0_mono)
    print('-> Log Data Written')
    
    #IMU
    accel_path = os.path.join(main_path,'accel.txt')
    mag_path = os.path.join(main_path,'mag.txt')
    gyro_path = os.path.join(main_path,'gyro.txt')
    quat_path = os.path.join(main_path,'quat.txt')
    euler_path = os.path.join(main_path,'euler.txt')
    lin_accel_path = os.path.join(main_path,'lin_accel_path.txt')
    gravity_path = os.path.join(main_path,'gravity.txt')
    imu_temp_path = os.path.join(main_path,'imu_temp.txt')
    
    
    #GPS
    gps_path = os.path.join(main_path,'gps.txt')

    ##MAVLINK INIT
    
    mav_tele = mavutil.mavlink_connection(device = '/dev/ttyUSB0', baud = 57600, autoreconnect = True)
    
    ##
    

    print('Entering While Loop!')
    
    ## Main Loop ---------------------------------------------------
    while(True):
        current_t = time.monotonic()
        if(current_t - last_t >= Ts):
            last_t = current_t
            time_elapsed = time.monotonic() - t0_mono
            print('T = ' + str(time_elapsed))
            
            
            ## Sensor Reading ------------------------------------
            #IMU
            imu_t = time.monotonic()
            imu_temp = imu.imu_temp()
            accel = imu.imu_accel()
            mag = imu.imu_mag()
            gyro = imu.imu_gyro()
            quat = imu.imu_quat()
            euler = imu.imu_euler()
            lin_accel = imu.imu_lin_accel()
            gravity = imu.imu_gravity()

            #GPS
            gps_t = time.monotonic()
            gps_data = gps.get_GPS()
            
            
            ## GPIO --------------------------------------------
            #System Startup
            
            
            #GPS Reinitialization
            if (gps_data[0] + gps_data[1] + gps_data[2] + gps_data[3] == 0):
                gps_data = gps_data_last
                if (int(math.floor(time_elapsed)) % 5 == 0):
                    gps.reinit()
                    writeLineToFile(log_path, 'GPS Lost! Reconnecting...')
            else:
                gps_data_last = gps_data

            ## File Writing ------------------------------------
            #IMU
            writeToFile(accel_path, imu_t, accel)
            writeToFile(mag_path, imu_t, mag)
            writeToFile(gyro_path, imu_t, gyro)
            writeToFile(quat_path, imu_t, quat)
            writeToFile(euler_path, imu_t, euler)
            writeToFile(lin_accel_path, imu_t, lin_accel)
            writeToFile(gravity_path, imu_t, gravity)
            writeToFile(imu_temp_path, imu_t, [imu_temp])

            #GPS
            writeToFile(gps_path, gps_t, gps_data)
            
            ## MAVLINK Transmission ---------------------------
            mav_t = int(time.monotonic()*1000)
            #Generic heartbeat (REQUIRED FOR MISSION PLANNER)
            mav_tele.mav.heartbeat_send(8, 0, 128, 0, 4, 1) 
            #Attitude euler angles
            mav_tele.mav.sys_status_send(int('100111',2)+0x2000000,
                                         int('100111',2)+0x2000000,
                                         int('100111',2)+0x2000000,
                                         np.uint16((final_t - current_t)/Ts*1000),
                                         65535,
                                         -1,
                                         -1,
                                         0,
                                         0,
                                         0,
                                         0,
                                         0,
                                         0)
                                         
            mav_tele.mav.attitude_send(mav_t, 
                                       float(euler[0]*math.pi/180), #roll
                                       float(euler[2]*math.pi/180), #pitch
                                       float(euler[1]*math.pi/180), #yaw
                                       float(gyro[0]),  #roll angular speed
                                       float(gyro[2]),  #pitch angular speed
                                       float(gyro[1]))  #yaw angular speed
            #GPS Data
            mav_tele.mav.global_position_int_send(mav_t,
                                                  int(gps_data[0] * 10000000), #lat (deg*10^7)
                                                  int(gps_data[1] * 10000000), #long (deg*10^7)
                                                  int(gps_data[2]*1000), #alt (mm)
                                                  int(gps_data[2]*1000 - 331012), #alt above sea level (mm) APPROX IN PHOENIX
                                                  0, #ground x speed (unknown)
                                                  0, #ground x speed (unknown)
                                                  0, #ground x speed (unknown)
                                                  np.uint16(int(euler[2]*100))) #roll
            #Accel Data
            mav_tele.mav.scaled_imu_send(mav_t,
                                         np.int16(int(accel[0]*1000/9.807)),
                                         np.int16(int(accel[1]*1000/9.807)),
                                         np.int16(int(accel[2]*1000/9.807)),
                                         np.int16(int(gyro[0]*1000/360*2*math.pi)),
                                         np.int16(int(gyro[1]*1000/360*2*math.pi)),
                                         np.int16(int(gyro[2]*1000/360*2*math.pi)),
                                         np.int16(int(mag[0]*10)),
                                         np.int16(int(mag[1]*10)),
                                         np.int16(int(mag[2]*10)))
            #Battery Data
            #mav_tele.mav.battery_status_send(0,
            #                                 1,
            #                                 1,
            #                                 int(ext_temps[4]*1000),
            #                                 [int(voltages[4]*1000), 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535],
            #                                 -1,
            #                                 -1,
            #                                 -1,
            #                                 -1)
             
          
                                                  
            #LED BLINK
            if (math.floor(time_elapsed) % 2 == 0):
                GPIO.output(26, True)
            else:
                GPIO.output(26, False)

            
            
            # Final Time
            final_t = time.monotonic()
            print('-> t_delta = %s' % str(final_t - current_t))
        
            #Two Hour Ground Timeout

                
            #Four Hour Timeout
            if (time_elapsed > 14400):
                writeLineToFile(log_path, '4 Hour Auto Board Shutdown - Goodbye!')
                exit_handler()
                shutdown_board()
                
    

main()
    

    
