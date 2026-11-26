import struct
import time

from .constants import *
from .datatypes import *
from .exceptions import *
from .protocol import *
from .serial_transport import *
from .ti_float import decode_ti_float

class Metric(IntEnum):
    VOLTAGE = 0
    CURRENT = 1
    POWER = 2
    VA = 3
    VAR = 4
    FREQUENCY = 5
    PHASE_ANGLE = 6
    PF = 7
    ANALOG_SENSE = 8
    DELTA_PHASE = 9
    DELTA_VOLTS = 10
    DELTA_WATTS = 11
    DELTA_VA = 12
    DELTA_VAR = 13
    X_DELTA_VAR = 14
    X_WYE_VAR = 15

class RD3X:

    def __init__(self, port):

        self.transport = SerialTransport(port)

    def close(self):
        self.transport.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


    def transact(self,command,payload=b""):

        packet = build_packet(command,payload)

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

            length = struct.unpack(">H",length_bytes)[0]

            payload = self.transport.read_exact(length)

            crc = self.transport.read_exact(2)

            frame = (head + length_bytes + payload + crc)

            if (struct.unpack(">H", crc)[0] != checksum(frame[:-2])):
                raise RD3XChecksumError()

            if ack == Ack.ERROR:

                code = struct.unpack(">H",payload[:2])[0]

                raise RD3XCommandError(command,code)

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


# ------------------------------------------------------------------
# Harmonics
# ------------------------------------------------------------------

    def start_voltage_harmonics(self):

        # payload = self.transact(CMD_TRIGGER_HARMONIC,bytes([0x02, 1]))
        payload = self.transact(CMD_TRIGGER_HARMONIC,bytes([0x02, 1,1,0]))

        while payload[0] != 2:
            
            time.sleep(0.05)

            payload = self.transact(CMD_TRIGGER_HARMONIC,bytes([0x00, 0x00]))

        print("Voltage Harmonics Ready")

    def start_current_harmonics(self):
        payload = self.transact(CMD_TRIGGER_HARMONIC,bytes([0x03, 1]))

        while payload[0] != 3:

            time.sleep(0.05)

            payload = self.transact(CMD_TRIGGER_HARMONIC,bytes([0x00, 0x00]))

        print("Current Harmonics Ready")

    def read_harmonic(
        self,
        harmonic: int,
        voltage: bool = True,
        phases=("A",),
        timeout: float = 5.0
    ):
        """
        Read a single harmonic.

        Parameters
        ----------
        harmonic : int
            Harmonic number 1..50

        voltage : bool
            True  = voltage harmonic
            False = current harmonic

        phases : tuple
            Example:
                ("A",)
                ("A","B")
                ("A","B","C")

        timeout : float
            Timeout in seconds

        Returns
        -------
        dict
        """

        if not 1 <= harmonic <= 50:
            raise ValueError(
                "harmonic must be 1..50"
            )


        phases = tuple(phase.upper() for phase in phases)
        if len(phases) != 1 or phases[0] not in ("A", "B", "C"):
            raise ValueError("Read one phase per RD-3X DCA: phases must be ('A',), ('B',), or ('C',)")

        # Command 0F reads the harmonic table for the DCA connected to the
        # instrument. The DSP offset is in 4-byte words; each harmonic has
        # three TI floats (amplitude, phase, distortion).
        offset = (harmonic - 1) * VALUES_PER_HARMONIC
        payload = struct.pack(">HI", BYTES_PER_HARMONIC, offset)
        data = self.transact(CMD_READ_HARMONIC, payload)

        if len(data) != BYTES_PER_HARMONIC:
            raise RD3XProtocolError(
                f"Expected {BYTES_PER_HARMONIC} bytes of harmonic data, received {len(data)}"
            )

        return {
            phases[0]: {
                "harmonic": harmonic,
                "amplitude": decode_ti_float(data[0:4]),
                "phase": decode_ti_float(data[4:8]),
                "distortion": decode_ti_float(data[8:12]),
            }
        }
