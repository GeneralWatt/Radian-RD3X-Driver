class RD3XError(Exception):
    pass


class RD3XTimeout(RD3XError):
    pass


class RD3XChecksumError(RD3XError):
    pass


class RD3XProtocolError(RD3XError):
    pass


class RD3XCommandError(RD3XError):

    def __init__(self, command, code):
        super().__init__(
            f"Command 0x{command:02X} rejected (0x{code:04X})"
        )