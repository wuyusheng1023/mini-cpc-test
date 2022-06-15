import smbus
import time

bus = smbus.SMBus(1)

address_1 = 0x25
address_2 = 0x26

def init_sensor(address):
    bus.write_i2c_block_data(address, 0x3F, [0xF9]) #Stop any cont measurement of the sensor
    time.sleep(.1)
    bus.write_i2c_block_data(address, 0x36, [0X1E]) #Set sensor in differential pressure mode, update rate 0.5ms
    time.sleep(.1)

init_sensor(address_1)
init_sensor(address_2)

def read_sensor(n):
    if n == 1:
        address = address_1
    elif n == 2:
        address = address_2

    reading=bus.read_i2c_block_data(address, 0, 9)
    pressure_value=reading[0]+float(reading[1])/255
    if pressure_value>=0 and pressure_value<128:
        diffirential_pressure=pressure_value*240/256 #scale factor adjustment
    elif pressure_value>128 and pressure_value<=256:
        diffirential_pressure=-(256-pressure_value)*240/256 #scale factor adjustment
    elif pressure_value==128:
        diffirential_pressure=99999999 #Out of range
    time.sleep(0.1)
    return pressure_value


if __name__ == "__main__":
    print(read_sensor(1))
    time.sleep(0.5)
    print(read_sensor(2))
