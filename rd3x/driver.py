import struct
import time

from .constants import *
from .datatypes import *
from .exceptions import *
from .protocol import *
from .serial_transport import *
from .ti_float import decode_ti_float

class Metric:

    VOLTAGE = 0
    CURRENT = 1
    POWER = 2
    PHASE = 6

class RD33:

    def __init__(self, port):

        self.transport = SerialTransport(port)

    def close(self):
        self.transport.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


    def transact(
        self,
        command,
        payload=b""
    ):

        packet = build_packet(
            command,
            payload
        )

        with self.transport.lock:

            self.transport.serial.reset_input_buffer()

            self.transport.serial.write(packet)

            self.transport.serial.flush()

            head = self.transport.read_exact(2)

            ack = head[0] & 0x0F

            if ack == Ack.DELAY:

                time.sleep(0.25)

                head = self.transport.read_exact(2)

                ack = head[0] & 0x0F

            if ack == Ack.NO_DATA:

                frame = head + self.transport.read_exact(2)

                return b""

            length_bytes = self.transport.read_exact(2)

            length = struct.unpack(
                ">H",
                length_bytes
            )[0]

            payload = self.transport.read_exact(
                length
            )

            crc = self.transport.read_exact(2)

            frame = (
                head
                + length_bytes
                + payload
                + crc
            )

            if (
                struct.unpack(">H", crc)[0]
                != checksum(frame[:-2])
            ):
                raise RD33ChecksumError()

            if ack == Ack.ERROR:

                code = struct.unpack(
                    ">H",
                    payload[:2]
                )[0]

                raise RD33CommandError(
                    command,
                    code
                )

            return payload


    def identity(self):

        payload = self.transact(
            Command.IDENTITY,
            b""
        )

        fields = [

            x.replace("\x00", "").strip()

            for x in
            payload.decode(
                "latin-1"
            ).split(",")
        ]

        while len(fields) < 4:
            fields.append("")

        return Identity(
            model=fields[0],
            serial=fields[1],
            name=fields[2],
            firmware=fields[3]
        )


    def read_metric(
        self,
        metric
    ):

        payload = self.transact(

            Command.READ_INSTANTANEOUS,

            struct.pack(
                ">HH",
                metric,
                0xFFFD
            )
        )

        values = [

            decode_ti_float(
                payload[i:i+4]
            )

            for i in range(
                0,
                len(payload),
                4
            )
        ]

        return ThreePhaseValue(
            A=values[0],
            B=values[1],
            C=values[2],
            total=values[3]
        )

    def pulse_constant(
        self,
        accumulator=0
    ):

        payload = self.transact(

            Command.PULSE_CONSTANT,

            struct.pack(
                ">HB",
                accumulator,
                1
            ) + bytes(4)
        )

        return decode_ti_float(
            payload[:4]
        )