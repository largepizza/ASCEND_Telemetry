import math
import serial
import time
import busio
import board
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
imu_sensor = adafruit_bno055.BNO055_I2C(i2c)
print('IMU Initialized!')

def imu_temp():
    temp = imu_sensor.temperature
    return temp
    
def imu_accel():
    accel = imu_sensor.acceleration
    return  accel
    
def imu_mag():
    mag = imu_sensor.magnetic
    return mag
    
def imu_gyro():
    gyro = imu_sensor.gyro
    return gyro
    
def imu_euler():
    euler = imu_sensor.euler
    return euler

def imu_quat():
    quat = imu_sensor.quaternion
    return quat
    
def imu_lin_accel():
    lin_accel = imu_sensor.linear_acceleration
    return lin_accel
    
def imu_gravity():
    gravity = imu_sensor.gravity
    return gravity
