# Khepri Drive – Python API Reference

This document provides a complete reference for the **Khepri Drive Python SDK**.
It describes all public classes, functions, parameters, and expected behavior.

The API abstracts the low-level serial protocol and exposes a clean,
high-level motor control interface.

---

## 📦 Module Overview

The SDK consists of a single Python file:

- `khepri.py`

It exposes the following public components:

- `KhepriDrive` — main interface to the motor drive
- `MotorParams` — motor electrical & mechanical parameters
- `TelemetryFrame` — decoded feedback data

---

## 🧩 Data Structures

### `MotorParams`

Represents motor electrical and configuration parameters.

```python
MotorParams(
    R: float,
    Lq: float,
    Kt: float,
    pole_pairs: int,
    supply_voltage: float,
    sense_direction: int,
    mechanical_offset: int
)
```

| Field | Description | Unit |
|------|------------|------|
| `R` | Phase resistance | Ω |
| `Lq` | Q-axis inductance | H |
| `Kt` | Torque constant | Nm/A |
| `pole_pairs` | Number of pole pairs | — |
| `supply_voltage` | DC bus voltage | V |
| `sense_direction` | Encoder direction (0 normal, 1 inverted) | — |
| `mechanical_offset` | Encoder mechanical offset | counts |

---

### `TelemetryFrame`

Represents one decoded telemetry packet received from the drive.

```python
TelemetryFrame(
    mode: int,
    Vin: float,
    Ia: float,
    Ib: float,
    angle: float,
    velocity: float,
    Id: float,
    Iq: float,
    desired_Iq: float,
    desired_velocity: float,
    desired_angle: float
)
```

| Field | Description | Unit |
|------|------------|------|
| `mode` | Active controller mode ID | — |
| `Vin` | Supply voltage | V |
| `Ia`, `Ib` | Phase currents | A |
| `Id`, `Iq` | D/Q currents | A |
| `angle` | Mechanical angle | deg |
| `velocity` | Mechanical velocity | deg/s |
| `desired_Iq` | Commanded Iq | A |
| `desired_velocity` | Commanded velocity | deg/s |
| `desired_angle` | Commanded position | deg |

---

## ⚙️ Class: `KhepriDrive`

Main interface to the Khepri motor drive.

---

### Constructor

```python
KhepriDrive(port: str, baudrate: int = 115200, timeout: float = 1.0)
```

| Parameter | Description |
|----------|------------|
| `port` | Serial port name (e.g. `COM6`, `/dev/ttyUSB0`) |
| `baudrate` | Serial baudrate |
| `timeout` | Serial timeout in seconds |

---

## 🔌 Connection Management

### `connect()`
Opens the serial connection.

### `disconnect()`
Closes the serial connection safely.

### `is_connected() → bool`
Returns connection status.

---

## 🛑 Safety & System Commands

### `calibrate()`
Starts encoder and system calibration.

### `emergency_stop()`
Immediately stops motor motion and disables control loops.

⚠️ Always call before exiting your program.

### `restart()`
Soft-restarts the controller firmware.

### `save_to_flash()`
Stores current parameters and gains in non-volatile memory.

---

## 🎛 Control Modes

### `set_open_loop(value: int)`
Open-loop voltage or duty control. the value provided is between 0 and 10000 such that 5000 means zero, 10000 means max duty in CW rotation while 0 means max duty in CCW rotation. be careful when using this function as just setting the value to zero will turn the motor with max velocity.

### `set_position(degrees: float)`
Closed-loop position control.

### `set_velocity(velocity: float)`
Closed-loop velocity control.

### `set_torque(position_deg: float, torque: int)`
Torque control with position reference.

### `impedance_mode(position_deg: float)`
Impedance control around a position.

### `compliance_mode()`
Enables compliant / gravity compensation mode.

---

## 🎚 PID Controller Tuning

All gains are normalized in range **[0.0 – 1.0]**.

- `set_position_pid(kp, ki, kd)`
- `set_velocity_pid(kp, ki, kd)`
- `set_id_pid(kp, ki, kd)`
- `set_iq_pid(kp, ki, kd)`

---

## 🧠 Sliding Mode Control (SMC)

- `set_smc(lambda_, rho)`
- `set_smc_secondary(lambda2, rho2)`

---

## 🔧 Motor Parameters

- `set_motor_params(params: MotorParams)`
- `request_motor_params()`

---

## 📡 Telemetry

### `read_telemetry() → TelemetryFrame | None`

Reads and decodes one telemetry packet if available.

---

## ⚠️ Usage Notes

- Always call `emergency_stop()` before exit
- Tune gains gradually
- Ensure correct motor parameters before enabling current control

---

## 👤 Author

Eng Ahmed soliman
