# RD3x Python Driver
Python driver for the Radian RD-3X reference standards.

Current implementation supports: (Based on reverse-engineered communications and Radian protocol documentation)

- Identity command
- Instantaneous metric reads
- Pulse constant reads
- Packet framing
- Checksum verification
- TI TMS320C3x float conversion



## Requirements:
pip install pyserial


## Serial Settings
Parameter	Value
Interface	RS232
Baud	57600
Data Bits	8
Parity	None
Stop Bits	1
DTR	Enabled
Mode	Half Duplex


## Quick Start
from rd3x import RD3x

rd = RD3x("COM3")

print(rd.identity())
Example:

{
    'model': 'RD-33-337',
    'serial': '300007',
    'version': '07.05.30'
}
Reading Measurements
Voltage:

volts = rd.read_instantaneous("V")
Current:

amps = rd.read_instantaneous("A")
Power:

watts = rd.read_instantaneous("W")
Phase Angle:

angle = rd.read_instantaneous("deg")



## Returned structure:

{
    "A": 230.1,
    "B": 229.8,
    "C": 230.4,
    "net": 230.1
}

## Packet Structure
START
COMMAND
LENGTH
PAYLOAD
CHECKSUM


## Example Identity:

A6 02 00 00 00 A8
Implemented Commands
Command	Function
0x02	Identity
0x2E	Instantaneous Metric
0x32	Pulse Constant
Known Metric IDs
ID	Metric
0	Volts
1	Amps
2	Watts
6	Phase Angle
Additional metrics are believed to exist but require the RD-Kit Customer Access Manual.

Limitations
The following functions exist on the RD-33 but are not yet implemented because command definitions are not available:

Harmonics
THD
Vector Data
Trend Logging
Power Quality
Flicker
Min / Max Metrics
Accumulators
Configuration Writes
References
Radian RD-33 Operations Manual

Radian PCSuite User Manual

Radian RD3x Communications Notes