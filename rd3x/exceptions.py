class RD33Error(Exception):
    pass


class RD33Timeout(RD33Error):
    pass


class RD33ChecksumError(RD33Error):
    pass


class RD33ProtocolError(RD33Error):
    pass


class RD33CommandError(RD33Error):

    def __init__(self, command, code):
        super().__init__(
            f"Command 0x{command:02X} rejected (0x{code:04X})"
        )