from rd33.driver import RD33
from rd33.driver import Metric

with RD33("COM3") as rd:

    print(rd.identity())

    volts = rd.read_metric(
        Metric.VOLTAGE
    )

    print(volts)

    amps = rd.read_metric(
        Metric.CURRENT
    )

    print(amps)

    watts = rd.read_metric(
        Metric.POWER
    )

    print(watts)