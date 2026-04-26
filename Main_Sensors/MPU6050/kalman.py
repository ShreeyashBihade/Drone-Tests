from mpu6050 import mpu6050
from time import perf_counter, sleep
import math

MPU = mpu6050(0x68)

# ═══════════════════════════════════════════
#           CALIBRATION / OFFSET CALCULATOR
# ═══════════════════════════════════════════

CALIBRATION_SAMPLES = 200  # more samples = more accurate offset

pitch_offset = 0.0
roll_offset  = 0.0
gx_offset    = 0.0
gy_offset    = 0.0
gz_offset    = 0.0

print("=" * 45)
print("  CALIBRATING — Keep the drone flat!")
print("  Do NOT touch or move the sensor...")
print("=" * 45)

# Countdown so you have time to set it down
for i in range(3, 0, -1):
    print(f"  Starting in {i}...")
    sleep(1)

print(f"  Sampling {CALIBRATION_SAMPLES} readings...")

for i in range(CALIBRATION_SAMPLES):
    accel = MPU.get_accel_data()
    gyro  = MPU.get_gyro_data()

    ax, ay, az = accel['x'], accel['y'], accel['z']
    gx, gy, gz = gyro['x'],  gyro['y'],  gyro['z']

    # Accelerometer angle offsets
    pitch_offset += math.atan2(-ax, az)                      * (180/math.pi)
    roll_offset  += math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180/math.pi)

    # Gyro drift offsets
    gx_offset += gx
    gy_offset += gy
    gz_offset += gz

    # Progress bar
    if (i + 1) % 20 == 0:
        progress = int(((i + 1) / CALIBRATION_SAMPLES) * 20)
        bar = "█" * progress + "░" * (20 - progress)
        print(f"  [{bar}] {i+1}/{CALIBRATION_SAMPLES}")

    sleep(0.01)  # 100Hz during calibration

# Average all samples
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
#           KALMAN FILTER SETUP
# ═══════════════════════════════════════════

pitch_angle = 0.0
roll_angle  = 0.0
pitch_P     = 1.0
roll_P      = 1.0

Q = 0.001
R = 0.03

last_time = perf_counter()

print("  Running Kalman filter — Ctrl+C to stop")
print("=" * 45)

# ═══════════════════════════════════════════
#           MAIN LOOP
# ═══════════════════════════════════════════

while True:
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

    # Accelerometer angles with offset subtracted
    pitch = math.atan2(-ax, az)                      * (180/math.pi) - pitch_offset
    roll  = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180/math.pi) - roll_offset

    # ── PREDICT ──
    pitch_angle = pitch_angle + gy * dt
    roll_angle  = roll_angle  + gx * dt
    pitch_P     = pitch_P + Q
    roll_P      = roll_P  + Q

    # ── KALMAN GAIN ──
    k_pitch = pitch_P / (pitch_P + R)
    k_roll  = roll_P  / (roll_P  + R)

    # ── UPDATE ──
    pitch_angle = pitch_angle + k_pitch * (pitch - pitch_angle)
    roll_angle  = roll_angle  + k_roll  * (roll  - roll_angle)
    pitch_P     = (1 - k_pitch) * pitch_P
    roll_P      = (1 - k_roll)  * roll_P

    print(f"Raw      → pitch: {pitch:7.2f}°  roll: {roll:7.2f}°")
    print(f"Filtered → pitch: {pitch_angle:7.2f}°  roll: {roll_angle:7.2f}°")
    print("─" * 45)

    sleep(0.02)
