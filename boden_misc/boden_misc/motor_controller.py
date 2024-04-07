import board
import busio

ADDR = 0x42

i2c = busio.I2C(board.SCL_1, board.SDA_1)


def read_motor_value(reg):
    buffer = bytearray(4)

    i2c.try_lock()
    i2c.writeto_then_readfrom(
        ADDR,
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

