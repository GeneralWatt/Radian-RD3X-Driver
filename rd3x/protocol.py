import struct

from .constants import START_BYTE


def checksum(data: bytes) -> int:

    return sum(data) & 0xFFFF


def build_packet(
    command,
    payload=b"",
    include_length=True
):

    frame = bytearray()

    frame.append(START_BYTE)
    frame.append(command)

    if include_length:
        frame.extend(
            struct.pack(">H", len(payload))
        )

    frame.extend(payload)

    frame.extend(
        struct.pack(">H", checksum(frame))
    )

    return bytes(frame)