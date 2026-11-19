def decode_ti_float(data: bytes) -> float:

    if len(data) != 4:
        raise ValueError()

    exp_byte = data[0]

    if exp_byte in (0x80, 0x81):
        return 0.0

    exponent = (
        exp_byte - 256
        if exp_byte >= 128
        else exp_byte
    )

    fraction = (
        ((data[1] & 0x7F) << 16)
        |
        (data[2] << 8)
        |
        data[3]
    )

    if data[1] & 0x80:
        return -(2.0 - fraction / (1 << 23)) * (2.0 ** exponent)

    return (1.0 + fraction / (1 << 23)) * (2.0 ** exponent)