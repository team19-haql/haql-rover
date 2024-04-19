import board
import busio
import time

ADDR = 0x22

def setup(alternate):
    global i2c
    if alternate:
        i2c = busio.I2C(board.SCL_1, board.SDA_1)
    else:
        i2c = busio.I2C(board.SCL, board.SDA)

def read_motor_value(reg):
    buffer = bytearray(4)

    i2c.try_lock()
    i2c.writeto_then_readfrom(ADDR,
        bytearray([reg]),
        buffer,
        out_start=0,
        out_end=4,
        )
    i2c.unlock()

    value = int.from_bytes(buffer, byteorder='little', signed=True)
    value = value / (1 << 15)
    return value

def write_motor_value(reg, value):
    value = int(value * (1 << 15))
    value = value.to_bytes(4, byteorder='little', signed=True)

    i2c.try_lock()
    i2c.writeto(ADDR, bytearray([reg]) + value)
    i2c.unlock()

def scan_all():
    i2c.try_lock()
    print([hex(i) for i in i2c.scan()])
    i2c.unlock()

if __name__ == '__main__':
    print('Bus 0')
    setup(False)
    scan_all()
    i2c.deinit()

    # print('Bus 1')
    # setup(True)
else:
    setup(False)
