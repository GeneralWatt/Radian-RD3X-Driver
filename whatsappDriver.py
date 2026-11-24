"""
Radian RD-xx / RD-33 Python Driver - Single File
Based on: Customer Access Commands Manual for RD-xx Family, Doc 944011.D Rev D
Supports: RD-20/21 v3.05+, RD-22 v3.04+, RD-3x v6.00+, RD-33 v7.02.90+

Protocol:
  RS-232 57600 baud, 1 start, 1 stop, no parity, DTR high
  Packet: [Start 1B][Type 1B][Length 2B BE][Data 0-256B][Checksum 2B BE]
  Checksum = sum(Start + Type + Length bytes + Data bytes) & 0xFFFF
  Start upper nibble = DCA, lower nibble = ACK status
    Master DCA = 0xA (0xA6 = Master, ACK with data)
    RD-3x Phase A = 0xB (0xB6), B = 0xC (0xC6), C = 0xD (0xD6)
  ACKs:
    0xA3 = ACK no data, 0xA6 = ACK with data,
    0xAC = ACK with delay (data = delay ms), 0xA9 = ACK with error (data = error code)

TI TMS320VC33 float:
  Bits 31-24 = exponent (2's complement), Bit 23 = sign, Bits 22-0 = mantissa
  Exponent -128 => 0.0. Mantissa is 2's complement, 24-bit effective.
  Unsupported metrics return FLT_MAX = 3.4028235e38

Harmonics (RD-33):
  28h Trigger -> poll status -> 0Fh Read Table 3
  0Fh Offset is DSP WORDS = byte_offset / 4, max 252 bytes per call
"""

import struct
import time
import math
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # pip install pyserial

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BAUDRATE = 57600
TIMEOUT = 1.0  # RD replies typically <50ms, worst 225ms

class DCA(IntEnum):
    MASTER = 0xA
    PHASE_A = 0xB  # RD-3x
    PHASE_B = 0xC  # RD-3x
    PHASE_C = 0xD  # RD-3x

class Ack(IntEnum):
    NODATA = 0x3
    DATA = 0x6
    DELAY = 0xC
    ERROR = 0x9

class PacketType(IntEnum):
    NOP = 0x00
    IDENTIFICATION = 0x02
    RESET_RD = 0x03
    RESET_METRICS = 0x07
    START_ACCUM = 0x08
    STOP_ACCUM = 0x09
    START_TIMED_ACCUM = 0x0A
    LOCK_RELAY = 0x0B
    TRIGGER_WAVEFORM = 0x0C
    READ_INSTANT_RD2X = 0x0D
    READ_WAVEFORM = 0x0E
    READ_HARMONIC = 0x0F
    READ_ACCUM_RD2X = 0x16
    AUTO_CALIBRATE = 0x1B
    BNC_CONTROL = 0x1D
    SYSTEM_STATUS = 0x20
    READ_MIN_RD2X = 0x21
    READ_MAX_RD2X = 0x23
    TRIGGER_HARMONIC = 0x28
    MODE_CHANGE = 0x2C
    READ_INSTANT_RD3X = 0x2E
    READ_ACCUM_RD3X = 0x2F
    READ_MIN_RD3X = 0x30
    READ_MAX_RD3X = 0x31
    SET_PULSE_CONST = 0x32
    STANDARD_TEST = 0x34
    METER_TEST = 0x39
    VECTOR_VOLT_AMP = 0x7C

ERROR_CODES = {
    0x01: "Line Error (Framing/Break)",
    0x02: "Checksum Error",
    0x03: "Timeout Error",
    0x04: "Invalid Start Byte (not A6h)",
    0x05: "Length Error",
    0x06: "Invalid Packet Type",
    0x07: "Invalid Start Address",
    0x08: "Invalid Range",
    0x09: "Invalid Data Length",
    0x0A: "Invalid Boundary Byte",
    0x0B: "Invalid Device",
    0x0C: "Device Error",
    0x0D: "Receive Register Overflow",
    0x0E: "Invalid Start Delimiter",
    0x0F: "Invalid Data Error",
    0x10: "Internal Communications Error",
}

# Table 1 - Instantaneous Metrics (Section 4.1)
INSTANTANEOUS_METRICS = [
    (0, 0x14, "Volts (V)"),
    (1, 0x18, "Amps (A)"),
    (2, 0x1C, "Watt (W)"),
    (3, 0x20, "Volt-Amps (VA)"),
    (4, 0x24, "VAR"),
    (5, 0x28, "Frequency (Hz)"),
    (6, 0x2C, "Phase (deg)"),
    (7, 0x30, "Power Factor (PF)"),
    (8, 0x34, "Analog Sense"),
    (9, 0x38, "Delta Phase"),
    (10, 0x3C, "Delta Volts (dV)"),
    (11, 0x40, "Delta Watts (dW)"),
    (12, 0x44, "Delta VA (dVA)"),
    (13, 0x48, "Delta VAR (dVAR)"),
    (14, 0x4C, "Cross-connected Delta VAR (xdVAR)"),
    (15, 0x50, "Cross-connected Wye VAR (xyVAR)"),
]

# Table 2 - Accumulated Metrics (Section 4.2)
ACCUMULATED_METRICS = [
    (0, 0x04, "Watt Hours (Wh)"),
    (1, 0x08, "VAR Hours (VARh)"),
    (2, 0x0C, "Q Hours (Qh)"),
    (3, 0x10, "Volt-Amp Hours (VAh)"),
    (4, 0x14, "Volt Hours (Vh)"),
    (5, 0x18, "Amp Hours (Ah)"),
    (6, 0x1C, "Volt Squared Hours (V2h)"),
    (7, 0x20, "Amp Squared Hours (A2h)"),
    (8, 0x24, "Watt Hours Plus (Wh+)"),
    (9, 0x28, "Watt Hours Minus (Wh-)"),
    (10, 0x2C, "VAR Hours Plus (VARh+)"),
    (11, 0x30, "VAR Hours Minus (VARh-)"),
    (12, 0x34, "Delta Watt Hours (dWh)"),
    (13, 0x38, "Accumulated Time (t)"),
    (14, 0x3C, "Delta VA Hours (dVAh)"),
    (15, 0x40, "Delta VAR Hours (dVARh)"),
    (16, 0x44, "Cross Delta VAR Hours (xdVARh)"),
    (17, 0x48, "Cross Wye VAR Hours (xyVARh)"),
    (18, 0x4C, "Delta Wh+"),
    (19, 0x50, "Delta Wh-"),
    (20, 0x54, "Delta VARh+"),
    (21, 0x58, "Delta VARh-"),
    (22, 0x5C, "Cross Delta VARh+"),
    (23, 0x60, "Cross Delta VARh-"),
    (24, 0x64, "Cross Wye VARh+"),
    (25, 0x68, "Cross Wye VARh-"),
]

FLT_MAX = 3.4028235e38

# ---------------------------------------------------------------------------
# Harmonic Constants
# ---------------------------------------------------------------------------

class HarmonicStatus(IntEnum):
    NOT_RUNNING = 0
    IN_PROGRESS = 1
    VOLTAGE_READY = 2
    CURRENT_READY = 3

HARMONIC_STATUS_NAMES = {
    0: "Not running (data = zeros)",
    1: "In progress (data = zeros)",
    2: "Voltage analysis ready",
    3: "Current analysis ready",
}

class HarmonicControl(IntEnum):
    STATUS_FIXED = 0x00
    STATUS_RESTART_FIXED = 0x01
    TRIGGER_VOLTAGE_FIXED = 0x02
    TRIGGER_CURRENT_FIXED = 0x03
    STATUS_TRACKING = 0x10
    STATUS_RESTART_TRACKING = 0x11
    TRIGGER_VOLTAGE_TRACKING = 0x12
    TRIGGER_CURRENT_TRACKING = 0x13

@dataclass
class HarmonicResult:
    harmonic: int
    amplitude: float
    phase_deg: float
    distortion: float
    status: int
    phase: str

@dataclass
class HarmonicTableEntry:
    order: int
    amplitude: float
    phase_deg: float
    distortion_thd: float

# ---------------------------------------------------------------------------
# TI Float Conversion - Sec 3.2 / 3.3, TMS320C3x
# ---------------------------------------------------------------------------

def ti_to_float(data: bytes) -> float:
    """Convert 4-byte TI TMS320VC33 float (big-endian as sent) to Python float."""
    if len(data) != 4:
        raise ValueError("TI float needs 4 bytes")
    b0, b1, b2, b3 = data
    if b0 in (0x80, 0x81):
        return 0.0
    if (b0, b1, b2, b3) == (0x7F, 0x7F, 0xFF, 0xFF):
        return FLT_MAX
    mantissa = ((b1 & 0x7F) << 16) | (b2 << 8) | b3
    neg = bool(b1 & 0x80)
    if b0 == 0x7F and neg and mantissa == 0:
        return -FLT_MAX
    exp = b0 if b0 < 0x80 else b0 - 0x100
    if exp == -128:
        return 0.0
    if neg:
        full = mantissa | 0x800000
        signed = full - 0x1000000
        return (signed / (1 << 23)) * (2.0 ** exp)
    else:
        full = mantissa | 0x800000
        return (full / (1 << 23)) * (2.0 ** exp)

def float_to_ti(value: float) -> bytes:
    """Convert Python float to 4-byte TI format. Used for Set Pulse Constant etc."""
    if value == 0.0:
        return bytes([0x80, 0x00, 0x00, 0x00])
    if value >= FLT_MAX:
        return bytes([0x7F, 0x7F, 0xFF, 0xFF])
    if value <= -FLT_MAX:
        return bytes([0x7F, 0xFF, 0xFF, 0xFF])
    neg = value < 0
    abs_val = abs(value)
    exp = math.floor(math.log2(abs_val)) + 1
    mant = int(round((abs_val / (2.0 ** exp)) * (1 << 23)))
    if mant >= (1 << 24):
        mant >>= 1
        exp += 1
    mant_23 = mant & 0x7FFFFF
    b0 = exp & 0xFF
    b1 = ((0x80 if neg else 0x00) | ((mant_23 >> 16) & 0x7F))
    b2 = (mant_23 >> 8) & 0xFF
    b3 = mant_23 & 0xFF
    if neg:
        signed = (-mant) & 0xFFFFFF
        mant_23 = signed & 0x7FFFFF
        b1 = 0x80 | ((mant_23 >> 16) & 0x7F)
        b2 = (mant_23 >> 8) & 0xFF
        b3 = mant_23 & 0xFF
    return bytes([b0, b1, b2, b3])

def calc_checksum(packet_without_checksum: bytes) -> int:
    """Sec 3.1: sum of all bytes & 0xFFFF"""
    return sum(packet_without_checksum) & 0xFFFF

# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

@dataclass
class RDResponse:
    start: int
    packet_type: int
    data: bytes
    ack: int
    dca: int
    delay_ms: Optional[int] = None
    error_code: Optional[int] = None

    @property
    def is_error(self) -> bool:
        return self.ack == Ack.ERROR

class RDError(Exception):
    pass

class RDxx:
    def __init__(self, port: str, dca: DCA = DCA.MASTER, timeout: float = TIMEOUT):
        if serial is None:
            raise ImportError("pip install pyserial")
        self.ser = serial.Serial(
            port=port, baudrate=BAUDRATE, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=timeout,
            rtscts=False, dsrdtr=False, xonxoff=False,
        )
        self.ser.dtr = True  # DTR line high per Sec 1.3
        self.dca = dca
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()
    def __enter__(self):
        return self
    def __exit__(self, *a):
        self.close()

    # -- low level --
    def _build_packet(self, pkt_type: int, data: bytes = b"", start: Optional[int] = None) -> bytes:
        if start is None:
            start = (self.dca << 4) | 0x06
        header = bytes([start, pkt_type]) + struct.pack(">H", len(data)) + data
        return header + struct.pack(">H", calc_checksum(header))

    def _read_response(self) -> RDResponse:
        hdr = self.ser.read(2)
        if len(hdr) < 2:
            raise RDError("Timeout waiting for response")
        start, pkt_type = hdr[0], hdr[1]
        ack = start & 0x0F
        dca = (start >> 4) & 0x0F
        if ack == Ack.NODATA:
            raw = self.ser.read(2)
            if len(raw) < 2:
                raise RDError("Truncated ACK_NODATA checksum")
            return RDResponse(start, pkt_type, b"", ack, dca)
        len_bytes = self.ser.read(2)
        if len(len_bytes) < 2:
            raise RDError("Truncated length")
        length = struct.unpack(">H", len_bytes)[0]
        data = self.ser.read(length) if length else b""
        cksum_bytes = self.ser.read(2)
        if len(cksum_bytes) < 2:
            raise RDError("Truncated checksum")
        calc = calc_checksum(bytes([start, pkt_type]) + len_bytes + data)
        recv = struct.unpack(">H", cksum_bytes)[0]
        if calc != recv:
            raise RDError(f"Checksum mismatch: calc {calc:04X} != recv {recv:04X}")
        resp = RDResponse(start, pkt_type, data, ack, dca)
        if ack == Ack.DELAY and len(data) >= 2:
            resp.delay_ms = struct.unpack(">H", data[:2])[0]
        if ack == Ack.ERROR and len(data) >= 2:
            code = struct.unpack(">H", data[:2])[0]
            resp.error_code = code
            raise RDError(f"RD Error {code:02X}h: {ERROR_CODES.get(code, 'Unknown')}")
        return resp

    def transact(self, pkt_type: int, data: bytes = b"", start: Optional[int] = None) -> RDResponse:
        self.ser.reset_input_buffer()
        self.ser.write(self._build_packet(pkt_type, data, start))
        resp = self._read_response()
        if resp.delay_ms:
            time.sleep(resp.delay_ms / 1000.0)
        return resp

    # -- 00h NOP --
    def nop(self) -> bool:
        resp = self.transact(PacketType.NOP)
        return not resp.is_error

    # -- 02h Identification --
    def identification(self) -> dict:
        resp = self.transact(PacketType.IDENTIFICATION)
        text = resp.data.split(b"\x00")[0].decode("ascii", errors="ignore")
        parts = [p.strip() for p in text.split(",")]
        return {
            "raw": text,
            "model": parts[0] if len(parts) > 0 else "",
            "serial": parts[1] if len(parts) > 1 else "",
            "name": parts[2] if len(parts) > 2 else "",
            "version": parts[3] if len(parts) > 3 else "",
        }

    # -- 03h Reset RD --
    def reset_rd(self) -> RDResponse:
        return self.transact(PacketType.RESET_RD)

    # -- 07h Reset Metrics --
    # Bits: 01=Waveform, 02=Instant, 04=Min, 08=Max, 10=Accum
    def reset_metrics(self, mask: int = 0x1F) -> bool:
        resp = self.transact(PacketType.RESET_METRICS, bytes([mask]))
        return not resp.is_error

    # -- 08h Start Accumulating --
    # control: 00=status query, 01=normal gate, 06=meter gate, 0E=sensor mode
    def start_accumulating(self, control: int = 0x01, pulse_data: int = 0) -> RDResponse:
        data = bytes([control]) + struct.pack(">H", pulse_data)
        return self.transact(PacketType.START_ACCUM, data)

    # -- 09h Stop Accumulating --
    def stop_accumulating(self) -> RDResponse:
        return self.transact(PacketType.STOP_ACCUM)

    # -- 0Ah Start Timed Accumulating Test --
    def start_timed_test(self, samples: int) -> RDResponse:
        return self.transact(PacketType.START_TIMED_ACCUM, struct.pack(">I", samples))

    # -- 0Bh Lock/Unlock Relay Ranges --
    def lock_relay(self, control_byte: int, relay_bits: int) -> RDResponse:
        data = bytes([control_byte]) + struct.pack(">H", relay_bits)
        return self.transact(PacketType.LOCK_RELAY, data)

    # -- 0Ch Trigger Waveform --
    def trigger_waveform(self) -> RDResponse:
        return self.transact(PacketType.TRIGGER_WAVEFORM)

    # -- 0Dh Read Instantaneous RD-2x --
    def read_instantaneous_rd2x(self, offset: int = 0x40, count: int = 0x14, update_flag: int = 0xFFFD) -> dict:
        data = struct.pack(">H", offset) + struct.pack(">I", count) + struct.pack(">H", update_flag)
        resp = self.transact(PacketType.READ_INSTANT_RD2X, data)
        return self._parse_ti_block(resp.data, INSTANTANEOUS_METRICS)

    # -- 16h Read Accumulating RD-2x --
    def read_accumulating_rd2x(self) -> dict:
        data = struct.pack(">H", 0x20) + struct.pack(">I", 0x04)
        resp = self.transact(PacketType.READ_ACCUM_RD2X, data)
        return self._parse_ti_block(resp.data, ACCUMULATED_METRICS[:8])

    # -- 21h / 23h Min / Max RD-2x --
    def read_min_rd2x(self) -> dict:
        data = struct.pack(">H", 0x40) + struct.pack(">I", 0x14)
        resp = self.transact(PacketType.READ_MIN_RD2X, data)
        return self._parse_ti_block(resp.data, INSTANTANEOUS_METRICS)

    def read_max_rd2x(self) -> dict:
        data = struct.pack(">H", 0x40) + struct.pack(">I", 0x14)
        resp = self.transact(PacketType.READ_MAX_RD2X, data)
        return self._parse_ti_block(resp.data, INSTANTANEOUS_METRICS)

    # -- 2Eh / 2Fh / 30h / 31h RD-3x single-metric --
    def _read_rd3x_metric(self, pkt_type: int, metric_index: int, extra: bytes = b"") -> dict:
        data = struct.pack(">H", metric_index) + extra
        resp = self.transact(pkt_type, data)
        if len(resp.data) < 20:
            raise RDError(f"Short RD-3x response: {len(resp.data)} bytes")
        vals = [ti_to_float(resp.data[i*4:(i+1)*4]) for i in range(5)]
        return {"A": vals[0], "B": vals[1], "C": vals[2], "Net": vals[3], "Reserved": vals[4]}

    def read_instantaneous_rd3x(self, metric_index: int) -> dict:
        return self._read_rd3x_metric(PacketType.READ_INSTANT_RD3X, metric_index, struct.pack(">H", 0xFFFD))

    def read_accumulating_rd3x(self, metric_index: int) -> dict:
        return self._read_rd3x_metric(PacketType.READ_ACCUM_RD3X, metric_index)

    def read_min_rd3x(self, metric_index: int) -> dict:
        return self._read_rd3x_metric(PacketType.READ_MIN_RD3X, metric_index)

    def read_max_rd3x(self, metric_index: int) -> dict:
        return self._read_rd3x_metric(PacketType.READ_MAX_RD3X, metric_index)

    # -- 20h System Status --
    def system_status(self) -> bytes:
        resp = self.transact(PacketType.SYSTEM_STATUS)
        return resp.data

    # -- 1Dh BNC Control --
    def bnc_control_rd2x(self, bnc_code: int, bnc_data: int) -> RDResponse:
        return self.transact(PacketType.BNC_CONTROL, bytes([bnc_code, bnc_data]))

    def bnc_control_rd3x(self, bnc_code: int, bnc_data: int, phase_code: int) -> RDResponse:
        return self.transact(PacketType.BNC_CONTROL, bytes([bnc_code, bnc_data, phase_code]))

    # -- 32h Set Pulse Output Constant --
    def set_pulse_constant(self, metric_index: int, status: int, constant: float) -> RDResponse:
        data = struct.pack(">H", metric_index) + bytes([status]) + float_to_ti(constant)
        return self.transact(PacketType.SET_PULSE_CONST, data)

    # -- 1Bh Auto-Calibrate --
    def auto_calibrate(self, mode: int = 0) -> RDResponse:
        return self.transact(PacketType.AUTO_CALIBRATE, bytes([mode]))

    # -- helpers --
    @staticmethod
    def _parse_ti_block(data: bytes, table) -> dict:
        out = {}
        for idx, offset, name in table:
            pos = idx * 4
            if pos + 4 <= len(data):
                val = ti_to_float(data[pos:pos+4])
                out[name] = None if val >= FLT_MAX else val
        return out

    # =======================================================================
    # HARMONICS - Sec 2.12 [0Fh], Sec 2.19 [28h], Table 3 Sec 4.3
    # =======================================================================

    def trigger_harmonic(self, control: int, harmonic_number: int,
                          subharmonic: int = 1, base_harmonic: int = 0) -> dict:
        """
        28h Trigger Harmonic Analysis
        Request: [Control 1B][Harmonic 1B][Subharmonic 1B][Base 1B] (N=4)
        Response Master (A6h): 42 bytes = 3 x [Status 1B + HarmonicData 13B] + Subharmonic Status
        Response Per-phase: 14 bytes = [Status 1B + HarmonicData 13B]
        For RD-33 >=07.02.90, HarmonicData 13B = [Harmonic 1B][Amplitude 4B TI][Phase 4B TI][Distortion 4B TI]
        """
        data = bytes([control, harmonic_number, subharmonic, base_harmonic])
        resp = self.transact(PacketType.TRIGGER_HARMONIC, data)
        return self._parse_trigger_response(resp.data, self.dca)

    @staticmethod
    def _parse_trigger_response(data: bytes, dca) -> dict:
        results = {}
        if dca == DCA.MASTER:
            phases = ['A', 'B', 'C']
            for i, ph in enumerate(phases):
                off = i * 14
                if off + 14 > len(data):
                    break
                status = data[off]
                harm_data = data[off+1:off+14]
                results[ph] = RDxx._parse_single_harmonic_block(status, harm_data, ph)
            if len(data) > 42:
                results['subharmonic_status'] = list(data[42:])
        else:
            ph = {DCA.PHASE_A: 'A', DCA.PHASE_B: 'B', DCA.PHASE_C: 'C'}.get(dca, '?')
            if len(data) >= 14:
                results[ph] = RDxx._parse_single_harmonic_block(data[0], data[1:14], ph)
            if len(data) > 14:
                results['subharmonic_status'] = list(data[14:])
        return results

    @staticmethod
    def _parse_single_harmonic_block(status: int, block: bytes, phase: str) -> HarmonicResult:
        harm_num = block[0]
        amp = ti_to_float(block[1:5])
        phase_deg = ti_to_float(block[5:9])
        distortion = ti_to_float(block[9:13])
        return HarmonicResult(
            harmonic=harm_num, amplitude=amp, phase_deg=phase_deg,
            distortion=distortion, status=status, phase=phase
        )

    def read_harmonic_table(self, num_bytes: int = 252, offset_dsp_words: int = 0) -> list:
        """
        0Fh Read Harmonic Data - Table 3
        Request: [N 2B BE][Offset 4B BE]
          N = bytes to read, multiple of 4, max 252
          Offset = DSP address = byte_offset / 4  !!! CAUTION in manual
        Table 3 layout (07.03.40+):
          00h: Amp H1, 04h: Phase H1, 08h: THD
          0Ch: Amp H2, 10h: Phase H2, 14h: Distortion H2
          ...
          24Ch: Amp H50, 250h: Phase H50, 254h: Distortion H50
          Total 600 bytes, needs 3 calls (252+252+96)
        NOTE: This table is per-DCA. Use per-phase DCA for RD-33.
        """
        entries = []
        remaining = 600
        byte_offset = 0
        while remaining > 0:
            chunk = min(252, remaining)
            chunk = (chunk // 4) * 4
            dsp_offset = byte_offset // 4
            data = struct.pack(">H", chunk) + struct.pack(">I", dsp_offset)
            resp = self.transact(PacketType.READ_HARMONIC, data)
            raw = resp.data
            for i in range(0, len(raw), 12):
                if i + 12 > len(raw):
                    break
                order = (byte_offset + i) // 12 + 1
                amp = ti_to_float(raw[i:i+4])
                ph = ti_to_float(raw[i+4:i+8])
                dist = ti_to_float(raw[i+8:i+12])
                entries.append(HarmonicTableEntry(order, amp, ph, dist))
            byte_offset += chunk
            remaining -= chunk
        return entries

    def get_harmonics_rd33(self, axis: str = "voltage", harmonic_number: int = 1,
                           tracking: bool = False, timeout_s: float = 5.0) -> dict:
        """
        High-level helper for RD-33.
        axis: "voltage" or "current"
        harmonic_number: which harmonic to trigger (1 = fundamental)
        tracking: False = fixed 50/60Hz, True = tracking fundamental
        Flow: 1. Trigger 28h  2. Poll status until ready  3. Read 0Fh table for 1..50
        Returns: {"trigger": {...}, "tables": {'A': [...], 'B': [...], 'C': [...]}}
        """
        if axis.lower().startswith("v"):
            ctrl_trigger = HarmonicControl.TRIGGER_VOLTAGE_TRACKING if tracking else HarmonicControl.TRIGGER_VOLTAGE_FIXED
            ctrl_status = HarmonicControl.STATUS_TRACKING if tracking else HarmonicControl.STATUS_FIXED
            expected_status = HarmonicStatus.VOLTAGE_READY
        else:
            ctrl_trigger = HarmonicControl.TRIGGER_CURRENT_TRACKING if tracking else HarmonicControl.TRIGGER_CURRENT_FIXED
            ctrl_status = HarmonicControl.STATUS_TRACKING if tracking else HarmonicControl.STATUS_FIXED
            expected_status = HarmonicStatus.CURRENT_READY

        # 1. Trigger - Master DCA gets all 3 phases at once
        old_dca = self.dca
        self.dca = DCA.MASTER
        try:
            trigger_resp = self.trigger_harmonic(ctrl_trigger, harmonic_number)
            # 2. Poll until ready
            start = time.time()
            while time.time() - start < timeout_s:
                status_resp = self.trigger_harmonic(ctrl_status, 0)
                st = status_resp.get('A')
                if st and st.status == expected_status:
                    break
                if st and st.status == HarmonicStatus.NOT_RUNNING:
                    break
                time.sleep(0.2)
        finally:
            self.dca = old_dca

        # 3. Read full tables per phase
        result = {"trigger": trigger_resp, "tables": {}}
        for dca, ph in [(DCA.PHASE_A, 'A'), (DCA.PHASE_B, 'B'), (DCA.PHASE_C, 'C')]:
            old = self.dca
            self.dca = dca
            try:
                result["tables"][ph] = self.read_harmonic_table()
            finally:
                self.dca = old
        return result

    def stop_harmonic(self):
        """Stop harmonic analysis: Trigger with harmonic 0"""
        return self.trigger_harmonic(HarmonicControl.TRIGGER_VOLTAGE_FIXED, 0)

    def harmonic_status(self, tracking: bool = False) -> dict:
        """Poll harmonic status without triggering"""
        ctrl = HarmonicControl.STATUS_TRACKING if tracking else HarmonicControl.STATUS_FIXED
        return self.trigger_harmonic(ctrl, 0)

# ---------------------------------------------------------------------------
# Example
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # Windows: "COM3", Linux: "/dev/ttyUSB0", macOS: "/dev/tty.usbserial-..."
    with RDxx("COM3") as rd:
        print(rd.identification())
        print("NOP:", rd.nop())

        # Instantaneous - RD-2x block (all 16 metrics)
        inst = rd.read_instantaneous_rd2x()
        for k, v in inst.items():
            print(f"{k}: {v}")

        # Instantaneous - RD-3x single metric, e.g. Volts index 0
        print("Volts RD-3x:", rd.read_instantaneous_rd3x(0))

        # Accumulating test
        rd.reset_metrics(0x1F)
        rd.start_accumulating(control=0x01)
        time.sleep(5)
        rd.stop_accumulating()
        print("Accum:", rd.read_accumulating_rd2x())

        # Min / Max
        print("Min:", rd.read_min_rd2x())
        print("Max:", rd.read_max_rd2x())

        # --- HARMONICS - RD-33 ---
        # Quick single harmonic (5th, voltage)
        res = rd.trigger_harmonic(HarmonicControl.TRIGGER_VOLTAGE_FIXED, 5)
        print("5th Harmonic Phase A:", res['A'])

        # Full spectrum 1..50, all phases, voltage
        full = rd.get_harmonics_rd33(axis="voltage", harmonic_number=1)
        for ph, table in full["tables"].items():
            print(f"\n--- Phase {ph} Voltage Harmonics ---")
            for e in table[:10]:
                print(f"H{e.order:2d}: Amp={e.amplitude:.3f}  Phase={e.phase_deg:.1f}°  Dist={e.distortion_thd:.4f}")
            print(f"THD (H1): {table[0].distortion_thd}")

        # Current harmonics
        full_i = rd.get_harmonics_rd33(axis="current", harmonic_number=1)
        print("Current THD Phase A:", full_i["tables"]['A'][0].distortion_thd)

        rd.stop_harmonic()