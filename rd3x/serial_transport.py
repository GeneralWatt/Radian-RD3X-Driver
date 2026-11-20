import serial
import threading
import time

from .constants import BAUDRATE
from .exceptions import RD3XTimeout


class SerialTransport:

    def __init__(self, port):

        self.lock = threading.Lock()

        self.serial = serial.Serial(
            port=port,
            baudrate=BAUDRATE,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.1
        )

        self.serial.dtr = True

        time.sleep(0.3)

    def close(self):

        self.serial.close()

    def read_exact(
        self,
        count,
        timeout=10.0
    ):

        deadline = time.monotonic() + timeout

        data = bytearray()

        while len(data) < count:

            data.extend(
                self.serial.read(
                    count - len(data)
                )
            )

            if time.monotonic() > deadline:
                raise RD3XTimeout()

        return bytes(data)