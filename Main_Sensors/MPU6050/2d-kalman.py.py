from mpu6050 import mpu6050
from time import perf_counter, sleep
import math

MPU = mpu6050(0x68)

# ═══════════════════════════════════════════
#           CALIBRATION / OFFSET CALCULATOR
# ═══════════════════════════════════════════

CALIBRATION_SAMPLES = 500       # Increased from 200 — ~5s at 100Hz for better gyro bias accuracy
TARGET_LOOP_HZ      = 50        # Hz
TARGET_DT           = 1.0 / TARGET_LOOP_HZ

pitch_offset = 0.0
roll_offset  = 0.0
gx_offset    = 0.0
gy_offset    = 0.0
gz_offset    = 0.0

print("=" * 45)
print("  CALIBRATING — Keep the drone flat!")
print("  Do NOT touch or move the sensor...")
print("=" * 45)

for i in range(3, 0, -1):
    print(f"  Starting in {i}...")
    sleep(1)

print(f"  Sampling {CALIBRATION_SAMPLES} readings...")

for i in range(CALIBRATION_SAMPLES):
    accel = MPU.get_accel_data()
    gyro  = MPU.get_gyro_data()

    ax, ay, az = accel['x'], accel['y'], accel['z']
    gx, gy, gz = gyro['x'],  gyro['y'],  gyro['z']

    pitch_offset += math.atan2(-ax, az)                      * (180/math.pi)
    roll_offset  += -math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180/math.pi)

    gx_offset += gx
    gy_offset += gy
    gz_offset += gz

    if (i + 1) % 50 == 0:
        progress = int(((i + 1) / CALIBRATION_SAMPLES) * 20)
        bar = "█" * progress + "░" * (20 - progress)
        print(f"  [{bar}] {i+1}/{CALIBRATION_SAMPLES}")

    sleep(0.01)

pitch_offset /= CALIBRATION_SAMPLES
roll_offset  /= CALIBRATION_SAMPLES
gx_offset    /= CALIBRATION_SAMPLES
gy_offset    /= CALIBRATION_SAMPLES
gz_offset    /= CALIBRATION_SAMPLES

print("=" * 45)
print("  CALIBRATION COMPLETE!")
print(f"  Pitch offset : {pitch_offset:+.4f}°")
print(f"  Roll  offset : {roll_offset:+.4f}°")
print(f"  Gyro X offset: {gx_offset:+.4f} °/s")
print(f"  Gyro Y offset: {gy_offset:+.4f} °/s")
print(f"  Gyro Z offset: {gz_offset:+.4f} °/s")
print("=" * 45)
sleep(1)

# ═══════════════════════════════════════════
#     2-STATE KALMAN FILTER SETUP
#     State vector: [angle, gyro_bias]
#     This lets the filter estimate AND cancel
#     gyro drift in real time during flight.
# ═══════════════════════════════════════════

# --- Noise tuning constants ---
# Q_angle : how much you trust the gyro integration  (lower = smoother, more gyro-reliant)
# Q_bias  : how fast the filter thinks gyro bias changes (keep very small)
# R_meas  : how much you trust the accelerometer     (raise this in-flight if vibration is high)
Q_angle = 0.001
Q_bias  = 0.003
R_meas  = 0.03

def kalman_init():
    """Return a fresh Kalman state dict for one axis."""
    return {
        "angle": 0.0,
        "bias":  0.0,
        # 2x2 error covariance matrix (flattened: P00, P01, P10, P11)
        "P00": 1.0, "P01": 0.0,
        "P10": 0.0, "P11": 1.0,
    }

def kalman_update(state, gyro_rate, accel_angle, dt):
    """
    Full 2-state Kalman update for one axis.

    gyro_rate   : bias-subtracted gyro reading (°/s) for this axis
    accel_angle : angle computed from accelerometer (°)
    dt          : time delta in seconds

    Returns the filtered angle (°).
    The state dict is updated in-place so bias estimation carries forward.
    """
    # ── PREDICT ──
    # Project state ahead using gyro (minus estimated bias)
    rate = gyro_rate - state["bias"]
    state["angle"] += dt * rate

    # Project error covariance ahead
    state["P00"] += dt * (dt * state["P11"] - state["P01"] - state["P10"] + Q_angle)
    state["P01"] -= dt * state["P11"]
    state["P10"] -= dt * state["P11"]
    state["P11"] += Q_bias * dt

    # ── KALMAN GAIN ──
    S = state["P00"] + R_meas          # Innovation (residual) covariance
    K0 = state["P00"] / S              # Gain for angle
    K1 = state["P10"] / S              # Gain for bias

    # ── UPDATE ──
    y = accel_angle - state["angle"]   # Innovation (measurement residual)
    state["angle"] += K0 * y
    state["bias"]  += K1 * y

    # Update error covariance
    P00_tmp = state["P00"]
    P01_tmp = state["P01"]
    state["P00"] -= K0 * P00_tmp
    state["P01"] -= K0 * P01_tmp
    state["P10"] -= K1 * P00_tmp
    state["P11"] -= K1 * P01_tmp

    return state["angle"]

# Initialise one filter per axis
pitch_state = kalman_init()
roll_state  = kalman_init()

# Seed initial angles from accelerometer so we don't start from 0°
_accel = MPU.get_accel_data()
_ax, _ay, _az = _accel['x'], _accel['y'], _accel['z']
pitch_state["angle"] = math.atan2(-_ax, _az)                      * (180/math.pi) - pitch_offset
roll_state["angle"]  = -math.atan2(_ay, math.sqrt(_ax**2 + _az**2)) * (180/math.pi) - roll_offset

last_time = perf_counter()

print("  Running Kalman filter — Ctrl+C to stop")
print("=" * 45)

# ═══════════════════════════════════════════
#           MAIN LOOP
# ═══════════════════════════════════════════

# Diagnostic counters — printed periodically, NOT every loop iteration
_loop_count   = 0
_overrun_count = 0

while True:
    loop_start = perf_counter()

    current_time = perf_counter()
    dt        = current_time - last_time
    last_time = current_time

    accel = MPU.get_accel_data()
    gyro  = MPU.get_gyro_data()

    ax, ay, az = accel['x'], accel['y'], accel['z']
    gx, gy, gz = gyro['x'],  gyro['y'],  gyro['z']

    # Subtract gyro drift offsets
    gx -= gx_offset
    gy -= gy_offset
    gz -= gz_offset

    # Accelerometer angles
    pitch_accel = math.atan2(-ax, az)                      * (180/math.pi) - pitch_offset
    roll_accel  = -math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180/math.pi) - roll_offset

    # ── 2-STATE KALMAN UPDATE ──
    # pitch is driven by gy (rotation around Y axis)
    # roll  is driven by gx (rotation around X axis)
    pitch_angle = kalman_update(pitch_state, gy, pitch_accel, dt)
    roll_angle  = kalman_update(roll_state,  gx, roll_accel,  dt)

    # ── PRINT ONLY EVERY 25 LOOPS (~2Hz) to avoid blocking the loop ──
    _loop_count += 1
    if _loop_count % 25 == 0:
        print(f"Pitch: {pitch_angle:7.2f}°  Roll: {roll_angle:7.2f}°  "
              f"| P_bias: {pitch_state['bias']:+.4f}  R_bias: {roll_state['bias']:+.4f}  "
              f"| Overruns: {_overrun_count}")

    # ── REMAINDER-SLEEP LOOP TIMER ──
    # Only sleep what's left of the target period.
    # If I2C reads ran long, we skip the sleep entirely rather than falling further behind.
    elapsed = perf_counter() - loop_start
    remaining = TARGET_DT - elapsed
    if remaining > 0:
        sleep(remaining)
    else:
        _overrun_count += 1  # Track how often the loop is running over budget