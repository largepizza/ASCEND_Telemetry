import math
import serial
import time
import busio
import board
import adafruit_bno055

#1, I2C1_SCL, I2C1_SDA
i2c = busio.I2C(board.SCL, board.SDA)
##ADDRESSES
#TCA9534A - I2C GPIO Expander
GPIO_ADDRESS = 0x38

#ADS7828 - I2C 12-Bit 8 Channel ADC
POW_ADC_ADDRESS = 0x48
TEMP_ADC_ADDRESS = 0x49

#LM75AD118 - I2C Digital Temperature Sensor
PI_TEMP_ADDRESS = 0x4C
TELE_TEMP_ADDRESS = 0x4D
CAM_TEMP_ADDRESS = 0x4E
ET_TEMP_ADDRESS = 0x4F

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


def read_temp(address):
    if (i2c.try_lock()):
        buff = bytearray(2)
        i2c.readfrom_into(address, buff)
        i2c.unlock()

        if buff[1] == 0x01:
            return ~buff[0] * -0.5
        else:
            return buff[0] * 0.5
    else:
        return -273


def read_fet_temps():
    #[PI, TELE, CAM, ET]
    return [read_temp(PI_TEMP_ADDRESS), read_temp(TELE_TEMP_ADDRESS), read_temp(CAM_TEMP_ADDRESS), read_temp(ET_TEMP_ADDRESS)]

def read_adc(address, ch):
    if (ch % 2 == 0):
        ch = int(8 + ch/2)
    else:
        ch = int(12 + (ch-1)/2)
    if (i2c.try_lock()):
        msg = bytearray(1)
        buff = bytearray(2)
        msg[0] = (ch & 0xF) << 4
        i2c.writeto(address,msg)
        i2c.readfrom_into(address, buff)
        i2c.unlock()

        val = ((buff[0] << 8) | (buff[1]))
        return 5.105*val/4095
    else:
        return -1

def read_voltages():
    #[PI, TELE, CAM, ET, BATT]
    return [read_adc(POW_ADC_ADDRESS,0x00), read_adc(POW_ADC_ADDRESS,0x02), read_adc(POW_ADC_ADDRESS,0x04), (151*read_adc(POW_ADC_ADDRESS,0x06))/51, (151*read_adc(TEMP_ADC_ADDRESS,0x00))/51]
    
def read_currents():
    #[PI, TELE, CAM, ET, BATT]
    return [read_adc(POW_ADC_ADDRESS,0x01)/2, read_adc(POW_ADC_ADDRESS,0x03)/2, read_adc(POW_ADC_ADDRESS,0x05)/2, read_adc(POW_ADC_ADDRESS,0x07)/2, read_adc(TEMP_ADC_ADDRESS,0x01)/2]

def read_ext_temps():
    #Locations [BBB, PI, CAM, ET, FAN, EXT]
    #Thermistor # [23, 21, 14, 12, 22 , 24]
    return [read_adc(TEMP_ADC_ADDRESS,0x02), read_adc(TEMP_ADC_ADDRESS,0x03), read_adc(TEMP_ADC_ADDRESS,0x04), read_adc(TEMP_ADC_ADDRESS,0x05), read_adc(TEMP_ADC_ADDRESS,0x06), read_adc(TEMP_ADC_ADDRESS,0x07)]
    
def convert_ext_temps(ext_temps):
    #Locations [BBB, PI, CAM, ET, FAN, EXT]
    #Thermistor # [23, 21, 14, 12, 22 , 24]
    A = [0.001466284845671, 0.001424496067374, 0.001618999741276, 0.001164695575098, 0.001415189476664, 0.001805517389675]
    B = [1.461108512220047e-04, 1.511635517653385e-04, 1.293962168691637e-04, 1.831854156496012e-04, 1.516074965218780e-04, 1.048608461803633e-04]
    C = [2.594940528980064e-07, 2.438885519442357e-07, 2.838965513394133e-07, 1.747990273626044e-07, 2.445546167968906e-07, 3.444477299123012e-07]
    
    ext_temp_conv = [0, 0, 0, 0, 0, 0]
    for n in range(6):
        Rs = (100e3 * ext_temps[n])/(5.105 - ext_temps[n])
        logRs = math.log(Rs)
        ext_temp_conv[n] = (1.0/(A[n] + B[n] * logRs + C[n]*logRs**3)) - 273.15
    return ext_temp_conv
    

def gpio_init():
    if (i2c.try_lock()):
        msg = bytearray(2)
        msg[0] = 0x03
        msg[1] = 0x00
        i2c.writeto(GPIO_ADDRESS, msg)
        i2c.unlock()
        return 1
    else:
        return -1


def toggle_gpio(val):
    if (i2c.try_lock()):
        msg = bytearray(2)
        msg[0] = 0x01
        msg[1] = val
        i2c.writeto(GPIO_ADDRESS, msg)
        i2c.unlock()
        return 1
    else:
        return -1


def gpio_switch(pi, tele, cam, et, LED):
    #switches the MOSFETS based on a simple 0,1 imput and constructs the byte
    val = (1*(-pi + 1) + 2*(-tele + 1) + 4*(-cam + 1) + 8*et + 0x10*LED) & 0xFF
    toggle_gpio(val)
    return 1
    


# Testing code, comment before running main.py
#gpio_init()
#toggle_gpio(0x17)

#while(1):
#    fet_temps = read_fet_temps();
#    print("PI = " + str(fet_temps[0]))
#    print("TELE = " + str(fet_temps[1]))
#    print("CAM = " + str(fet_temps[2]))
#    print("ET = " + str(fet_temps[3]))
#    gpio_switch(0,0,0,0,0)
#    print(convert_ext_temps(read_ext_temps()))
#    print(read_voltages())
#    print(read_currents())
#    time.sleep(0.5)