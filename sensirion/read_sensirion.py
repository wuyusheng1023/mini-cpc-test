import smbus
import time

bus = smbus.SMBus(1)
address=0x25
bus.write_i2c_block_data(address, 0x3F, [0xF9]) #Stop any cont measurement of the sensor
time.sleep(.5)
bus.write_i2c_block_data(address, 0x36, [0X1E]) #Set sensor in differential pressure mode, update rate 0.5ms
time.sleep(.5)

while True:
    reading=bus.read_i2c_block_data(address,0,9)
    pressure_value=reading[0]+float(reading[1])/255
    if pressure_value>=0 and pressure_value<128:
        diffirential_pressure=pressure_value*240/256 #scale factor adjustment
    elif pressure_value>128 and pressure_value<=256:
        diffirential_pressure=-(256-pressure_value)*240/256 #scale factor adjustment
    elif pressure_value==128:
        diffirential_pressure=99999999 #Out of range
    print("Diffirential Pressure: "+str(diffirential_pressure)+" PA")
    time.sleep(1)
