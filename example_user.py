from khepri import KhepriDrive, MotorParams
import time

drive = KhepriDrive(port="COM6")
drive.connect()

drive.emergency_stop()
time.sleep(0.5)
drive.calibrate()
time.sleep(2.0)

motor = MotorParams(
    R=0.38,
    Lq=0.00021,
    Kt=0.11,
    pole_pairs=7,
    supply_voltage=24.0,
    sense_direction=0,
    mechanical_offset=120
)

drive.set_motor_params(motor)
drive.save_to_flash()

drive.set_position_pid(0.6, 0.01, 0.0)
drive.set_velocity_pid(0.8, 0.02, 0.0)
drive.set_iq_pid(0.9, 0.1, 0.0)
drive.set_smc(0.5, 0.2)

drive.set_position(90)

start = time.time()
while time.time() - start < 5:
    t = drive.read_telemetry()
    if t:
        print(t.angle, t.velocity, t.Iq)
    time.sleep(0.05)

drive.emergency_stop()
drive.disconnect()
