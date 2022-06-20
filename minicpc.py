###########################################################
############################################################
# yusheng, 2019-04-11
# email: yusheng.wu@helsinki.fi
# tel: +358 41 4722 694
#
# hardware: 3*PT1000, 1*cooler, 2*heater, 1*fan, 1*air_pump, 1*liquild_pump

import os, glob, time, configparser, math
from time import sleep
import numpy as np
import RPi.GPIO as GPIO
from datetime import datetime, timedelta
from simple_pid import PID
from pymongo import MongoClient
from smbus2 import SMBus
from subprocess import call
import json

# from pubsub import publish

from sensirion.sensirion import read_sensor

############################################################
############################################################

for i in range(0):
    print(f"starting {i} ...")
    sleep(1)

# load configuration
dir_path = os.path.dirname(os.path.realpath(__file__))
config_file = dir_path + os.sep + 'conf.ini'
config = configparser.ConfigParser()
def getConfig(config_file):
    config.read(config_file)
    settings = (config['DEFAULT'])
    working = config['DEFAULT']['working'] == 'T'
    save_data = config['DEFAULT']['save_data'] == 'T'
    sleep_time = float(settings['sleep_time'])
    Ts_set = float(settings['Ts_set'])
    Tc_set = float(settings['Tc_set'])
    To_set = float(settings['To_set'])
    avg_coef = float(settings['avg_coef'])
    P_1 = float(settings['P_1'])
    I_1 = float(settings['I_1'])
    D_1 = float(settings['D_1'])
    scale_1 = float(settings['scale_1'])
    P_2 = float(settings['P_2'])
    I_2 = float(settings['I_2'])
    D_2 = float(settings['D_2'])
    scale_2 = float(settings['scale_2'])
    P_3 = float(settings['P_3'])
    I_3 = float(settings['I_3'])
    D_3 = float(settings['D_3'])
    scale_3 = float(settings['scale_3'])
    P_4 = float(settings['P_4'])
    I_4 = float(settings['I_4'])
    D_4 = float(settings['D_4'])
    scale_4 = float(settings['scale_4'])
    GAIN = float(settings['GAIN'])
    flow_CH = int(settings['flow_CH'])
    flow_coef = float(settings['flow_coef'])
    flow_set = float(settings['flow_set'])
    db_host = settings['db_host']
    db_port = int(settings['db_port'])
    db_name = settings['db_name']
    col_name = settings['col_name']
    return working, save_data, sleep_time, Ts_set, Tc_set, To_set, avg_coef, P_1, I_1, D_1, scale_1, P_2, I_2, D_2, scale_2, P_3, I_3, D_3, scale_3, P_4, I_4, D_4, scale_4, GAIN, flow_CH, flow_coef, flow_set, db_host, db_port, db_name, col_name


working, save_data, sleep_time, Ts_set, Tc_set, To_set, avg_coef, P_1, I_1, D_1, scale_1, P_2, I_2, D_2, scale_2, P_3, I_3, D_3, scale_3, P_4, I_4, D_4, scale_4, GAIN, flow_CH, flow_coef, flow_set, db_host, db_port, db_name, col_name  = getConfig(config_file)

#TODO
#flow_set_1, flow_set_2

# list of GPIO
PIN_3 = 2
PIN_5 = 3
PIN_7  = 4
PIN_11 = 17
PIN_12 = 18
PIN_13 = 27
PIN_15 = 22
PIN_16 = 23
PIN_18 = 24
PIN_19 = 10
PIN_21 = 9
PIN_22 = 25
PIN_23 = 11
PIN_24 = 8
PIN_26 = 7
PIN_27 = 0
PIN_28 = 1
PIN_29 = 5
PIN_31 = 6
PIN_32 = 12
PIN_33 = 13
PIN_35 = 19
PIN_36 = 16
PIN_37 = 26
PIN_38 = 20
PIN_40 = 21

# wiring input:
GPIO_OPC = PIN_36
GPIO_miso = PIN_21
GPIO_poweroff = PIN_32

# wiring output:
GPIO_heater_sat = PIN_15
GPIO_cooler = PIN_22
GPIO_heater_OPC = PIN_16
GPIO_air_pump = PIN_13
GPIO_csSat = PIN_31
GPIO_csCon = PIN_11
GPIO_csOPC = PIN_37
GPIO_mosi = PIN_19
GPIO_clk = PIN_23
GPIO_onled = PIN_40

# initialize measured temperature
Ts = Ts_set
Tc = Tc_set
To = To_set

# initialize GPIO
#GPIO.cleanup() # Reset ports
GPIO.setwarnings(False) # do not show any warnings
GPIO.setmode(GPIO.BCM) # programming the GPIO by BCM pin numbers.
GPIO.setup(GPIO_OPC, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # input, counter
GPIO.setup(GPIO_heater_sat,GPIO.OUT) # output, saturator heater
GPIO.setup(GPIO_cooler,GPIO.OUT) # output, cooler
GPIO.setup(GPIO_heater_OPC,GPIO.OUT) # output, saturator heater
GPIO.setup(GPIO_air_pump,GPIO.OUT) # output
GPIO.setup(GPIO_csSat, GPIO.OUT) # saturator temperature ADC Chip Select pin
GPIO.setup(GPIO_csCon, GPIO.OUT) # condenser temperature ADC Chip Select pin
GPIO.setup(GPIO_csOPC, GPIO.OUT) # OPC temperature ADC Chip Select pin
GPIO.setup(GPIO_miso, GPIO.IN) # define MISO-pin for SPI communication
GPIO.setup(GPIO_mosi, GPIO.OUT) # define MOSI-pin for SPI communication
GPIO.setup(GPIO_clk, GPIO.OUT) # define CLK-pin for SPI communication
GPIO.setup(GPIO_onled, GPIO.OUT) #define raspberry pi running led
GPIO.setup(GPIO_poweroff, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #define power off pin as input

GPIO.output(GPIO_csSat, GPIO.HIGH) #set chip select pin to high according to datasheet
GPIO.output(GPIO_csCon, GPIO.HIGH) #set chip select pin to high according to datasheet
GPIO.output(GPIO_csOPC, GPIO.HIGH) #set chip select pin to high according to datasheet
GPIO.output(GPIO_mosi, GPIO.LOW) #set set mosi low
GPIO.output(GPIO_clk, GPIO.LOW) #set serial clock low
GPIO.output(GPIO_onled, GPIO.HIGH) #turn on power on led
GPIO.output(GPIO_air_pump, GPIO.HIGH)

#init averaging
avg_conc = 0
cumul_conc = 0
cumul_volt = 0
avg = 0

#initialize power off button
def powerOff(channel):
    call("sudo shutdown -h now", shell=True)
#    print('shutdown')

GPIO.add_event_detect(GPIO_poweroff, GPIO.RISING, callback=powerOff, bouncetime=200)

# initialize pulse counter
def conterPlus(channel):
    global counter
    counter = counter + 1

GPIO.add_event_detect(GPIO_OPC, GPIO.RISING, callback=conterPlus)
counter = 0
concdd = 0
# initialize temperature sensor ADC

def readTemp(csPin):
    out = readRegisters(0, 8, csPin) #read all registers
    [rtd_msb, rtd_lsb] = [out[1], out[2]]
    rtd_ADC_Code = ((rtd_msb << 8) | rtd_lsb) >> 1 #acquire ADC_code
    temp_C = (rtd_ADC_Code / 32.0) - 256.0 #calculate temperature in celsius 
    return temp_C

def writeRegister(regNum, dataByte, csPin):
    GPIO.output(csPin, GPIO.LOW)   
    addressByte = 0x80 | regNum; # 0x8x to specify 'write register value'
    sendByte(addressByte) # first byte is address byte
    sendByte(dataByte) # the rest are data bytes
    GPIO.output(csPin, GPIO.HIGH)

def readRegisters(regNumStart, numRegisters, csPin):
    out = []
    GPIO.output(csPin, GPIO.LOW)
    sendByte(regNumStart)

    for byte in range(numRegisters):    
        data = recvByte()
        out.append(data)

    GPIO.output(csPin, GPIO.HIGH)
    return out

def sendByte(byte):
    for bit in range(8):
        GPIO.output(GPIO_clk, GPIO.HIGH)
        if (byte & 0x80):
            GPIO.output(GPIO_mosi, GPIO.HIGH)
        else:
            GPIO.output(GPIO_mosi, GPIO.LOW)
        byte <<= 1
        GPIO.output(GPIO_clk, GPIO.LOW)

def recvByte():
    byte = 0x00
    for bit in range(8):
        GPIO.output(GPIO_clk, GPIO.HIGH)
        byte <<= 1
        if GPIO.input(GPIO_miso):
            byte |= 0x1
        GPIO.output(GPIO_clk, GPIO.LOW)
    return byte

writeRegister(0, 0xC3, GPIO_csSat)      #select settings: vbias=1, conversion_mode=auto, continous_mode=1, 2-wire_mode, fault_auto_clear, 50Hz_filter. From datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
time.sleep(.1)                          # give it 100ms for conversion
writeRegister(0, 0xC3, GPIO_csCon)      #set same settings for second ADC
time.sleep(.1)                          # give it 100ms for conversion
writeRegister(0, 0xC3, GPIO_csOPC)      #set same settings for third ADC
time.sleep(.1)                          # give it 100ms for conversion

pid_1 = PID(P_1, I_1, D_1, setpoint=Ts_set) # heater sat
pwm_1 = GPIO.PWM(GPIO_heater_sat, 100) # PWM output, with 
pwm_1.start(0) # generate PWM signal with 0% duty cycle
pid_2 = PID(P_2, I_2, D_2, setpoint=Tc_set)
pwm_2 = GPIO.PWM(GPIO_cooler, 100)
pwm_2.start(0)
pid_3 = PID(P_3, I_3, D_3, setpoint=To_set)
pwm_3 = GPIO.PWM(GPIO_heater_OPC,10)
pwm_3.start(0)


# initialize flow rate, air pump
bus = SMBus(1)

def get_flow():
    sensirion_1 = read_sensor(1)
    sensirion_2 = read_sensor(2)
    return sensirion_1, sensirion_2

pid_4 = PID(P_4, I_4, D_4, setpoint=flow_set) # air pump
pwm_4 = GPIO.PWM(GPIO_air_pump,150)
pwm_4.start(0)

#TODO
#pwm_5

# initialize ADC converter
def ReadADC():
    LSBsize = 3.3/pow(2,8) #reference voltage divided by bit size (8bit ADC bit size = 2^8)
    MSB, LSB = bus.read_i2c_block_data(0x51, 0, 2) #Read two bytes from register 0 device address 0x51
    ADCCode = MSB<<4 | LSB>>4 #perform bit arithmetics according to datasheet
    voltage = ADCCode*LSBsize
    return voltage

# init database
db = MongoClient(db_host, db_port)[db_name]
collection = db[col_name]

#initialize sample time counter
start = datetime.now()
#initialize data save
filename = datetime.now().strftime("%Y%m%d-%H%M%S")
with open("/home/pi/data/"+filename, 'a') as f:
    f.write('date, time, concentration, scatter voltage, flow, Toptics, Tsaturator, Tcondenser')
    f.write('\n')

############################################################
############################################################

print('************************')
print('************************')
print('**** mini_CPC start ****')
print('************************')
print('************************')
while working:
    
    # load settings
    working, save_data, sleep_time, Ts_set, Tc_set, To_set, avg_coef, P_1, I_1, D_1, scale_1, P_2, I_2, D_2, scale_2, P_3, I_3, D_3, scale_3, P_4, I_4, D_4, scale_4, GAIN, flow_CH, flow_coef, flow_set, db_host, db_port, db_name, col_name = getConfig(config_file)

    # get datettime
    date_time = datetime.utcnow() + timedelta(hours=2) # get system date-time in UTC+2
    date_time = date_time.strftime("%Y-%m-%d %H:%M:%S")
    ## OPC counter
    counter  = 0
    start = datetime.utcnow()
    time.sleep(sleep_time)
    counts = counter
    time_delta = datetime.utcnow() - start
    
    ## get temperatures
    Ts = readTemp(GPIO_csSat) # saturator temperature
    Tc = readTemp(GPIO_csCon) # condensor temperature
    To = readTemp(GPIO_csOPC) # optics temperature
    
    ## change PWM
    pid_1 = PID(P_1, I_1, D_1, setpoint=Ts_set)
    pid_2 = PID(P_2, I_2, D_2, setpoint=Tc_set)
    pid_3 = PID(P_3, I_3, D_3, setpoint=To_set)
    dc_1 = pid_1(Ts) # heater duty cycle
    dc_2 = pid_2(2*Tc_set - Tc) # cooler duty cycle
    dc_3 = pid_3(To) # heater_2 duty cycle
    
    if dc_1 > 75: # duty cycle should between 0-100
        dc_1 = 75
    elif dc_1 < 0:
        dc_1 = 0
    dc_1 = dc_1*scale_1
    if dc_2 > 85: # duty cycle should between 0-100
        dc_2 = 85
    elif dc_2 < 0:
        dc_2 = 0
    dc_2 = dc_2*scale_2
    if dc_3 > 75: # duty cycle should between 0-100
        dc_3 = 75
    elif dc_3 < 0:
        dc_3 = 0
    dc_3 = dc_3*scale_3
    pwm_1.ChangeDutyCycle(dc_1)
    pwm_2.ChangeDutyCycle(dc_2)
    pwm_3.ChangeDutyCycle(dc_3)

    ## flow rate
    flow_1, flow_2 = get_flow()
    print(f"flow_1: {flow_1}, flow_2: {flow_2}")

    dc_4 = pid_4(flow_1)
    #dc_4 = 20
    if dc_4 > 100: # duty cycle should between 0-100
        dc_4 = 100
    elif dc_4 < 0:
        dc_4 = 0
    dc_4 = dc_4*scale_4
    pwm_4.ChangeDutyCycle(dc_4)

    #TODO
    #pwm_5

    #TODO
    #ADC
    # Read scatter mode voltage
    # voltage = ReadADC()

    sample_time = time_delta.seconds + time_delta.microseconds*10**(-6)

    flow = flow_1
    if flow > 0.01:
        concdd = counts/(flow*1000/60)/sample_time
    else:
        conc = 0
    #averaging
    cumul_conc += concdd
    # cumul_volt += voltage
    avg += 1

    if avg == avg_coef:
        avg_conc = cumul_conc / avg_coef
        # avg_voltage = cumul_volt / avg_coef
        with open("/home/pi/data/"+filename, 'a') as f:
            f.write(str(date_time))
            f.write(',')
            f.write(str(round(avg_conc)))
            f.write(',')
            # f.write(str(round(avg_voltage, 3)))
            # f.write(',')
            f.write(str(flow))
            f.write(',')
            f.write(str(round(To)))
            f.write(',')
            f.write(str(round(Ts)))
            f.write(',')
            f.write(str(round(Tc)))
            f.write('\n')
        
        dttm = datetime.utcnow() + timedelta(hours=2)
        dttm_str = dttm.strftime("%Y-%m-%d %H:%M:%S")

        data_dict = {
            'dttm': dttm_str,
            'conc': avg_conc,
            # 'volt': avg_voltage,
            'Ts': Ts,
            'Tc': Tc,
            'To': To,
            'flow': flow
        }
        # publish(data_dict)

        data_dict['dttm'] = dttm
        # collection.insert_one(data_dict)

        avg = 0
        cumul_conc = 0
        # cumul_volt = 0

    #print all
    print('************************************************************************')
    print(avg)
    print(date_time)
    print('Ts = ',round(Ts, 2), end = ' ,')
    print('Tc = ',round(Tc, 2), end = ' ,')
    print('To = ',round(To, 2))
    print(round(dc_1, 1), end = ' ,')
    print(round(dc_2, 1), end = ' ,')
    print(round(dc_3, 1))
    #print(round(dc_4, 1))
    print('Flow = ',round(flow, 3),' lpm')
    print(round(dc_4, 2))
    # print('Scatter voltage = ', round(voltage, 4), 'V')
    print('corrected concentration = ', round(concdd))
    print('average concentration = ', round(avg_conc))
#    print('noncorected concentration = ', round(conc))
    print('counts = ', counts)
    print('sample time = ', sample_time)
############################################################
############################################################

# exit

# ending database
#if save_data:
#    date_time = datetime.utcnow() # get system date-time in UTC
#    dataDict ={
#        'date_time': date_time,
#        'concentration': -999,
#        'counts': -999,
#        'Ts': -999,
#        'Tc': -999,
#        'To': -999,
#        'Td': -999,
#        'flow': -999,
#        'log': 'end'
#        }
#    collection.insert_one(dataDict)

GPIO.cleanup() # Reset ports

print('************************')
print('************************')
print('*********  bye *********')
print('************************')
print('************************')

