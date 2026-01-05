"""
Khepri Drive Python API (Single File)

This SDK provides:
- Serial connection handling
- Position / Velocity / Torque / Impedance / Compliance modes
- PID & SMC gain tuning
- Motor parameter configuration
- Telemetry decoding
"""

import serial
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


@dataclass
class MotorParams:
    R: float
    Lq: float
    Kt: float
    pole_pairs: int
    supply_voltage: float
    sense_direction: int
    mechanical_offset: int


@dataclass
class TelemetryFrame:
    mode: int
    Vin: float
    Ia: float
    Ib: float
    angle: float
    velocity: float
    Id: float
    Iq: float
    desired_Iq: float
    desired_velocity: float
    desired_angle: float


class GainID(IntEnum):
    POS_KP = 40
    POS_KI = 41
    POS_KD = 42
    VEL_KP = 43
    VEL_KI = 44
    VEL_KD = 45
    ID_KP = 46
    ID_KI = 47
    ID_KD = 48
    IQ_KP = 49
    IQ_KI = 50
    IQ_KD = 51
    SMC_LAMBDA = 52
    SMC_RHO = 53
    SMC_LAMBDA2 = 54
    SMC_RHO2 = 55


class KhepriDrive:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _send(self, payload: bytes):
        if not self.is_connected():
            raise RuntimeError("KhepriDrive not connected")
        self.ser.write(payload)

    def calibrate(self):
        self._send(bytes([0]))

    def emergency_stop(self):
        self._send(bytes([100]))

    def restart(self):
        self._send(bytes([101]))

    def save_to_flash(self):
        self._send(bytes([56]))

    def set_open_loop(self, value: int):
        self._send(bytes([1]) + int(value).to_bytes(2, 'little'))

    def set_position(self, degrees: float):
        raw = int(degrees * 10000 / 360)
        self._send(bytes([3]) + raw.to_bytes(2, 'little'))

    def set_velocity(self, velocity: float):
        raw = int(velocity + 5000)
        self._send(bytes([4]) + raw.to_bytes(2, 'little'))

    def set_torque(self, position_deg: float, torque: int):
        pos_raw = int(position_deg * 10000 / 360)
        self._send(bytes([5]) + pos_raw.to_bytes(2, 'little') + int(torque).to_bytes(1, 'little'))

    def impedance_mode(self, position_deg: float):
        raw = int(position_deg * 10000 / 360)
        self._send(bytes([7]) + raw.to_bytes(2, 'little'))

    def compliance_mode(self):
        self._send(bytes([8]))

    def _set_gain(self, gain: GainID, value: float):
        value = max(0.0, min(1.0, value))
        raw = int(value * 65535)
        self._send(bytes([gain.value]) + raw.to_bytes(2, 'little'))

    def set_position_pid(self, kp: float, ki: float, kd: float):
        self._set_gain(GainID.POS_KP, kp)
        self._set_gain(GainID.POS_KI, ki)
        self._set_gain(GainID.POS_KD, kd)

    def set_velocity_pid(self, kp: float, ki: float, kd: float):
        self._set_gain(GainID.VEL_KP, kp)
        self._set_gain(GainID.VEL_KI, ki)
        self._set_gain(GainID.VEL_KD, kd)

    def set_id_pid(self, kp: float, ki: float, kd: float):
        self._set_gain(GainID.ID_KP, kp)
        self._set_gain(GainID.ID_KI, ki)
        self._set_gain(GainID.ID_KD, kd)

    def set_iq_pid(self, kp: float, ki: float, kd: float):
        self._set_gain(GainID.IQ_KP, kp)
        self._set_gain(GainID.IQ_KI, ki)
        self._set_gain(GainID.IQ_KD, kd)

    def set_smc(self, lambda_: float, rho: float):
        self._set_gain(GainID.SMC_LAMBDA, lambda_)
        self._set_gain(GainID.SMC_RHO, rho)

    def set_smc_secondary(self, lambda2: float, rho2: float):
        self._set_gain(GainID.SMC_LAMBDA2, lambda2)
        self._set_gain(GainID.SMC_RHO2, rho2)

    def set_motor_params(self, p: MotorParams):
        packet = (
            bytes([70]) +
            int(p.R * 1000).to_bytes(2, 'little') +
            int(p.Lq * 10000).to_bytes(2, 'little') +
            int(p.Kt * 10000).to_bytes(2, 'little') +
            int(p.pole_pairs).to_bytes(2, 'little') +
            int(p.supply_voltage * 10).to_bytes(2, 'little') +
            int(p.sense_direction).to_bytes(2, 'little') +
            int(p.mechanical_offset).to_bytes(2, 'little')
        )
        self._send(packet)

    def request_motor_params(self):
        self._send(bytes([13]))

    def read_telemetry(self) -> Optional[TelemetryFrame]:
        if not self.ser or self.ser.in_waiting < 24:
            return None

        raw = self.ser.read(24)
        v = [int.from_bytes(raw[i:i + 2], 'little') for i in range(0, 24, 2)]

        return TelemetryFrame(
            mode=v[0],
            Vin=(v[2] - 10000) / 100,
            Ia=(v[3] - 10000) / 100,
            Ib=(v[4] - 10000) / 100,
            angle=v[5],
            velocity=(v[6] - 10000) / 10,
            Id=(v[7] - 10000) / 100,
            Iq=(v[8] - 10000) / 100,
            desired_Iq=(v[9] - 10000) / 100,
            desired_velocity=(v[10] - 10000) / 10,
            desired_angle=v[11],
        )
