import time


def continuous_snapshot(
    rd,
    interval=1.0
):

    while True:

        yield {

            "timestamp": time.time(),

            "voltage":
                rd.read_metric(0),

            "current":
                rd.read_metric(1),

            "power":
                rd.read_metric(2)
        }

        time.sleep(interval)