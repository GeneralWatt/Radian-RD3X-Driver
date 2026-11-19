from dataclasses import dataclass


@dataclass
class Identity:

    model: str
    serial: str
    name: str
    firmware: str


@dataclass
class ThreePhaseValue:

    A: float
    B: float
    C: float
    total: float