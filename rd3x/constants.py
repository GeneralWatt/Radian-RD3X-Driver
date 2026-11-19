from enum import IntEnum

BAUDRATE = 57600
START_BYTE = 0xA6


class Ack(IntEnum):
    NO_DATA = 0x03
    DATA = 0x06
    ERROR = 0x09
    DELAY = 0x0C


class Command(IntEnum):
    IDENTITY = 0x02
    READ_INSTANTANEOUS = 0x2E
    PULSE_CONSTANT = 0x32