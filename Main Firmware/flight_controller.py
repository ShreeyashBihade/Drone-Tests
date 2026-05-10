"""
Drone Flight Controller — Pi Zero
══════════════════════════════════════════════════════════════════
Hover-only build:
  • 2D Kalman filter  (pitch + roll attitude estimation)
  • Throttle PID only (altitude hold via stable throttle)
  • Tilt safety cutoff (motors killed if tilt > TILT_CUTOFF_DEG)
  • Live PID tuning   (UDP port 5006 — use tuner.py on laptop)
  • Signal watchdog   (motors killed after TIMEOUT_SEC silence)

Pitch / Roll / Yaw setpoints are fixed at 0 — no movement control.
Motor mixing is retained so the frame stays level via attitude error.

Pin / Motor layout:
    ESC1  TR  GPIO12  CW
    ESC2  BR  GPIO13  CCW
    ESC3  BL  GPIO18  CW
    ESC4  TL  GPIO19  CCW

Motor mixing (X-frame):
         Throttle  Pitch  Roll
    TR      +1      -1    -1
    BR      +1      +1    -1
    BL      +1      +1    +1
    TL      +1      -1    +1

Usage:
    sudo pigpiod
    python3 flight_controller.py
"""

import socket
import json
import math
import pigpio
from mpu6050 import mpu6050
from time import perf_counter, sleep


# ══════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════

MPU_ADDRESS = 0x68

MOTOR_PINS = {
    "TR": 12,
    "BR": 13,
    "BL": 18,
    "TL": 19,
}

PWM_MIN = 1000   # µs
PWM_MAX = 2000   # µs

# ── Network ────────────────────────────────────────────────────────
UDP_IP          = "0.0.0.0"
FLIGHT_PORT     = 5005   # throttle commands from controller
TUNING_PORT     = 5006   # PID updates from tuner.py
TIMEOUT_SEC     = 1.0

# ── Safety ─────────────────────────────────────────────────────────
TILT_CUTOFF_DEG = 45.0   # degrees — motors killed beyond this

# ── Calibration ────────────────────────────────────────────────────
CALIBRATION_SAMPLES = 750

# ── 2D Kalman ──────────────────────────────────────────────────────
Q_ANGLE   = 0.001
Q_BIAS    = 0.003
R_MEASURE = 0.03

# ── PID (throttle only) ────────────────────────────────────────────
# These are the starting values; tune live via tuner.py
PID_CFG = dict(Kp=1.2, Ki=0.05, Kd=0.3, integral_limit=50.0)

# Max µs the PID can add/subtract from the base throttle per motor
PID_OUTPUT_LIMIT = 300   # µs


# ══════════════════════════════════════════════════════════════════
#  2D KALMAN FILTER
# ══════════════════════════════════════════════════════════════════

class Kalman2D:
    """
    Tracks [angle, gyro_bias].
    Continuously estimates and corrects gyro drift during flight.
    """
    def __init__(self):
        self.angle = 0.0
        self.bias  = 0.0
        self.p00 = self.p01 = self.p10 = self.p11 = 0.0

    def update(self, accel_angle: float, gyro_rate: float, dt: float) -> float:
        # Predict
        rate = gyro_rate - self.bias
        self.angle += rate * dt
        self.p00 += dt * (dt * self.p11 - self.p01 - self.p10 + Q_ANGLE)
        self.p01 -= dt * self.p11
        self.p10 -= dt * self.p11
        self.p11 += Q_BIAS * dt

        # Update
        innov = accel_angle - self.angle
        S  = self.p00 + R_MEASURE
        k0 = self.p00 / S
        k1 = self.p10 / S

        self.angle += k0 * innov
        self.bias  += k1 * innov

        p00s, p01s = self.p00, self.p01
        self.p00 -= k0 * p00s
        self.p01 -= k0 * p01s
        self.p10 -= k1 * p00s
        self.p11 -= k1 * p01s

        return self.angle


# ══════════════════════════════════════════════════════════════════
#  PID CONTROLLER
# ══════════════════════════════════════════════════════════════════

class PID:
    def __init__(self, Kp, Ki, Kd, integral_limit=50.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_limit = integral_limit
        self._integral   = 0.0
        self._prev_meas  = 0.0

    def update_gains(self, Kp=None, Ki=None, Kd=None, integral_limit=None):
        if Kp  is not None: self.Kp = Kp
        if Ki  is not None: self.Ki = Ki
        if Kd  is not None: self.Kd = Kd
        if integral_limit is not None:
            self.integral_limit = integral_limit
        self.reset()   # clear state so old integral doesn't spike

    def compute(self, setpoint: float, measured: float, dt: float) -> float:
        error = setpoint - measured
        self._integral = max(-self.integral_limit,
                              min(self.integral_limit,
                                  self._integral + error * dt))
        derivative = -(measured - self._prev_meas) / dt if dt > 0 else 0.0
        self._prev_meas = measured
        out = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
        return max(-PID_OUTPUT_LIMIT, min(PID_OUTPUT_LIMIT, out))

    def reset(self):
        self._integral  = 0.0
        self._prev_meas = 0.0


# ══════════════════════════════════════════════════════════════════
#  MOTOR HELPERS
# ══════════════════════════════════════════════════════════════════

def mix_motors(throttle_us: float, pitch_out: float, roll_out: float) -> dict:
    """
    X-frame mixing — throttle only build, yaw locked at 0.
         Throttle  Pitch  Roll
    TR      +1      -1    -1
    BR      +1      +1    -1
    BL      +1      +1    +1
    TL      +1      -1    +1
    """
    raw = {
        "TR": throttle_us - pitch_out - roll_out,
        "BR": throttle_us + pitch_out - roll_out,
        "BL": throttle_us + pitch_out + roll_out,
        "TL": throttle_us - pitch_out + roll_out,
    }
    return {k: max(PWM_MIN, min(PWM_MAX, int(v))) for k, v in raw.items()}


def set_motors(pi_h, pwm: dict):
    for label, us in pwm.items():
        pi_h.set_servo_pulsewidth(MOTOR_PINS[label], us)


def all_motors_min(pi_h):
    set_motors(pi_h, {k: PWM_MIN for k in MOTOR_PINS})


def throttle_pct_to_us(pct: float) -> float:
    return PWM_MIN + (max(0.0, min(100.0, pct)) / 100.0) * (PWM_MAX - PWM_MIN)


def arm(pi_h):
    print("  Arming ESCs — minimum throttle for 3 s...")
    all_motors_min(pi_h)
    sleep(3)
    print("  Armed!\n")


def kill(pi_h):
    print("\n  Killing all motors...")
    all_motors_min(pi_h)
    sleep(0.5)
    for pin in MOTOR_PINS.values():
        pi_h.set_servo_pulsewidth(pin, 0)
    pi_h.stop()
    print("  Safe. Goodbye.")


# ══════════════════════════════════════════════════════════════════
#  CALIBRATION
# ══════════════════════════════════════════════════════════════════

def calibrate(mpu) -> dict:
    pitch_off = roll_off = gx_off = gy_off = gz_off = 0.0

    print("=" * 50)
    print("  CALIBRATING — Keep the drone flat!")
    print("=" * 50)
    for i in range(3, 0, -1):
        print(f"  Starting in {i}...")
        sleep(1)
    print(f"  Sampling {CALIBRATION_SAMPLES} readings...\n")

    for i in range(CALIBRATION_SAMPLES):
        accel = mpu.get_accel_data()
        gyro  = mpu.get_gyro_data()
        ax, ay, az = accel['x'], accel['y'], accel['z']
        gx, gy, gz = gyro['x'],  gyro['y'],  gyro['z']

        pitch_off += math.atan2(-ax, az)                       * (180 / math.pi)
        roll_off  += -math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180 / math.pi)
        gx_off += gx;  gy_off += gy;  gz_off += gz

        if (i + 1) % 50 == 0:
            p   = int(((i + 1) / CALIBRATION_SAMPLES) * 20)
            bar = "█" * p + "░" * (20 - p)
            print(f"  [{bar}] {i+1}/{CALIBRATION_SAMPLES}")

        sleep(0.01)

    n = CALIBRATION_SAMPLES
    off = dict(pitch=pitch_off/n, roll=roll_off/n,
               gx=gx_off/n, gy=gy_off/n, gz=gz_off/n)

    print("\n" + "=" * 50)
    print("  CALIBRATION COMPLETE")
    print(f"  Pitch : {off['pitch']:+.4f}°   Roll : {off['roll']:+.4f}°")
    print(f"  Gx : {off['gx']:+.4f}   Gy : {off['gy']:+.4f}   Gz : {off['gz']:+.4f}")
    print("=" * 50 + "\n")
    sleep(1)
    return off


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

def main():
    mpu = mpu6050(MPU_ADDRESS)

    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: pigpiod not running  →  sudo pigpiod")
        return

    offsets = calibrate(mpu)

    kf_pitch = Kalman2D()
    kf_roll  = Kalman2D()

    pid = PID(**PID_CFG)

    # ── Sockets ────────────────────────────────────────────────────
    # settimeout(0.001) = 1 ms timeout per recv call.
    # setblocking(False) is avoided — it behaves inconsistently on
    # ARM/Raspbian and silently stops delivering packets after the
    # first successful recvfrom.
    flight_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    flight_sock.bind((UDP_IP, FLIGHT_PORT))
    flight_sock.settimeout(0.001)

    tuning_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tuning_sock.bind((UDP_IP, TUNING_PORT))
    tuning_sock.settimeout(0.001)

    arm(pi)

    throttle_pct  = 0.0
    # Start far enough in the past that the watchdog fires immediately.
    # This guarantees motors stay at MIN (and pigpio pulsewidth=0 = off)
    # before the controller connects — prevents the GPIO12 pre-arm spin.
    last_received = perf_counter() - (TIMEOUT_SEC * 10)
    last_time     = perf_counter()
    tilt_killed   = False
    controller_connected = False

    print("=" * 50)
    print("  HOVER CONTROLLER ACTIVE")
    print(f"  Flight UDP  : port {FLIGHT_PORT}")
    print(f"  Tuning UDP  : port {TUNING_PORT}")
    print(f"  Tilt cutoff : ±{TILT_CUTOFF_DEG}°")
    print(f"  PID start   : Kp={pid.Kp} Ki={pid.Ki} Kd={pid.Kd}")
    print("  Ctrl+C to stop")
    print("=" * 50 + "\n")

    try:
        while True:
            now = perf_counter()
            dt  = max(now - last_time, 1e-6)
            last_time = now

            # ── Flight commands ────────────────────────────────────
            # Drain ALL pending packets — take only the latest value.
            # This prevents stale packets from shadowing newer ones
            # when the loop runs faster than the 50Hz send rate.
            _latest_throttle = None
            while True:
                try:
                    data, _ = flight_sock.recvfrom(1024)
                    pkt = json.loads(data.decode())
                    _latest_throttle = float(pkt.get("throttle", throttle_pct))
                    last_received = now
                except socket.timeout:
                    break   # buffer empty — done draining
                except (json.JSONDecodeError, KeyError, ValueError):
                    break

            if _latest_throttle is not None:
                if not controller_connected:
                    controller_connected = True
                    print(f"\n  Controller connected!")
                throttle_pct = _latest_throttle

            # ── Live tuning commands ───────────────────────────────
            try:
                tdata, _ = tuning_sock.recvfrom(1024)
                tpkt = json.loads(tdata.decode())
                pid.update_gains(
                    Kp=tpkt.get("kp"),
                    Ki=tpkt.get("ki"),
                    Kd=tpkt.get("kd"),
                )
                print(f"\n  [TUNE] Kp={pid.Kp:.4f}  Ki={pid.Ki:.4f}  Kd={pid.Kd:.4f}  "
                      f"(integral reset)")
            except (socket.timeout, json.JSONDecodeError):
                pass

            # ── Watchdog ───────────────────────────────────────────
            if now - last_received > TIMEOUT_SEC:
                all_motors_min(pi)
                elapsed = now - last_received
                if controller_connected:
                    controller_connected = False
                    print(f"\n  Controller disconnected!")
                print(f"  NO SIGNAL {elapsed:.1f}s — motors MIN  ", end="\r")
                pid.reset()
                tilt_killed = False
                continue

            # ── IMU ────────────────────────────────────────────────
            accel = mpu.get_accel_data()
            gyro  = mpu.get_gyro_data()
            ax, ay, az = accel['x'], accel['y'], accel['z']
            gx = gyro['x'] - offsets['gx']
            gy = gyro['y'] - offsets['gy']

            accel_pitch = math.atan2(-ax, az)                       * (180/math.pi) - offsets['pitch']
            accel_roll  = -math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180/math.pi) - offsets['roll']

            pitch = kf_pitch.update(accel_pitch, gy, dt)
            roll  = kf_roll.update( accel_roll,  gx, dt)

            # ── Tilt safety ────────────────────────────────────────
            if abs(pitch) > TILT_CUTOFF_DEG or abs(roll) > TILT_CUTOFF_DEG:
                if not tilt_killed:
                    print(f"\n  ⚠ TILT CUTOFF! pitch={pitch:.1f}° roll={roll:.1f}° — MOTORS KILLED")
                    tilt_killed = True
                all_motors_min(pi)
                pid.reset()
                continue
            else:
                tilt_killed = False

            # ── PID & mixing ───────────────────────────────────────
            if throttle_pct < 5.0:
                all_motors_min(pi)
                pid.reset()
                print(f"  IDLE  P:{pitch:+5.1f}°  R:{roll:+5.1f}°  Thr:  0%  "
                      f"[all motors MIN]        ", end="\r")
                continue

            throttle_us = throttle_pct_to_us(throttle_pct)

            # PID corrects tilt — setpoint is 0° for both axes (hover flat)
            pitch_corr = pid.compute(0.0, pitch, dt)
            roll_corr  = pid.compute(0.0, roll,  dt)

            pwm = mix_motors(throttle_us, pitch_corr, roll_corr)
            set_motors(pi, pwm)

            print(
                f"  Thr:{throttle_pct:3.0f}%  "
                f"P:{pitch:+5.1f}°  R:{roll:+5.1f}°  "
                f"Pcorr:{pitch_corr:+5.0f}  Rcorr:{roll_corr:+5.0f}  "
                f"TR:{pwm['TR']} BR:{pwm['BR']} BL:{pwm['BL']} TL:{pwm['TL']}µs  ",
                end="\r"
            )

            sleep(0.002)

    except KeyboardInterrupt:
        pass
    finally:
        kill(pi)
        flight_sock.close()
        tuning_sock.close()


if __name__ == "__main__":
    main()
