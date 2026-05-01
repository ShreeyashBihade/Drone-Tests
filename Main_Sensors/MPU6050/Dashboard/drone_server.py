"""
drone_server.py  —  MPU6050 Kalman filter + WebSocket broadcast
Run on the Raspberry Pi 4B:  python3 drone_server.py

Clients connect to  ws://<pi-ip>:8765
and receive JSON frames at ~50 Hz:
  {
    "pitch": float,   "roll": float,
    "ax": float, "ay": float, "az": float,
    "gx": float, "gy": float, "gz": float,
    "pitch_bias": float, "roll_bias": float,
    "loop_hz": float, "overruns": int
  }
"""

import asyncio
import json
import math
import signal
import sys
from time import perf_counter, sleep

# ── Graceful import of hardware libs ──────────────────────────────────────────
try:
    from mpu6050 import mpu6050
    MPU = mpu6050(0x68)
    SIMULATION = False
except Exception:
    print("[WARN] mpu6050 not found — running in simulation mode (sine waves).")
    SIMULATION = True

try:
    import websockets
except ModuleNotFoundError:
    print("[ERROR] websockets package missing. Install with:")
    print("         pip3 install websockets")
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════
WS_HOST   = "0.0.0.0"   # Accept connections from any network interface
WS_PORT   = 8765
TARGET_HZ = 50
TARGET_DT = 1.0 / TARGET_HZ

CALIBRATION_SAMPLES = 500

# Kalman noise parameters
Q_ANGLE = 0.001
Q_BIAS  = 0.003
R_MEAS  = 0.03

# ══════════════════════════════════════════════════════════════════════════════
#  KALMAN FILTER
# ══════════════════════════════════════════════════════════════════════════════
def kalman_init():
    return {"angle": 0.0, "bias": 0.0,
            "P00": 1.0, "P01": 0.0, "P10": 0.0, "P11": 1.0}

def kalman_update(state, gyro_rate, accel_angle, dt):
    rate = gyro_rate - state["bias"]
    state["angle"] += dt * rate
    state["P00"] += dt * (dt * state["P11"] - state["P01"] - state["P10"] + Q_ANGLE)
    state["P01"] -= dt * state["P11"]
    state["P10"] -= dt * state["P11"]
    state["P11"] += Q_BIAS * dt

    S  = state["P00"] + R_MEAS
    K0 = state["P00"] / S
    K1 = state["P10"] / S
    y  = accel_angle - state["angle"]
    state["angle"] += K0 * y
    state["bias"]  += K1 * y

    P00_tmp = state["P00"]
    P01_tmp = state["P01"]
    state["P00"] -= K0 * P00_tmp
    state["P01"] -= K0 * P01_tmp
    state["P10"] -= K1 * P00_tmp
    state["P11"] -= K1 * P01_tmp
    return state["angle"]

# ══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION
# ══════════════════════════════════════════════════════════════════════════════
def calibrate():
    if SIMULATION:
        return 0.0, 0.0, 0.0, 0.0, 0.0

    print("=" * 50)
    print("  CALIBRATING — keep the drone flat!")
    print("=" * 50)
    for i in range(3, 0, -1):
        print(f"  Starting in {i}...")
        sleep(1)

    po = ro = gxo = gyo = gzo = 0.0
    for i in range(CALIBRATION_SAMPLES):
        accel = MPU.get_accel_data()
        gyro  = MPU.get_gyro_data()
        ax, ay, az = accel['x'], accel['y'], accel['z']
        gx, gy, gz = gyro['x'],  gyro['y'],  gyro['z']

        po  += math.atan2(-ax, az)                        * (180 / math.pi)
        ro  += -math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180 / math.pi)
        gxo += gx;  gyo += gy;  gzo += gz

        if (i + 1) % 100 == 0:
            pct = int(((i + 1) / CALIBRATION_SAMPLES) * 20)
            print(f"  [{'█'*pct}{'░'*(20-pct)}] {i+1}/{CALIBRATION_SAMPLES}")
        sleep(0.01)

    n = CALIBRATION_SAMPLES
    po /= n;  ro /= n;  gxo /= n;  gyo /= n;  gzo /= n
    print("  Calibration complete!")
    print(f"  Pitch offset: {po:+.4f}°  Roll offset: {ro:+.4f}°")
    print(f"  Gyro offsets: gx={gxo:+.4f}  gy={gyo:+.4f}  gz={gzo:+.4f}")
    print("=" * 50)
    sleep(1)
    return po, ro, gxo, gyo, gzo

# ══════════════════════════════════════════════════════════════════════════════
#  SENSOR READ  (real or simulated)
# ══════════════════════════════════════════════════════════════════════════════
_sim_t = 0.0

def read_sensor(pitch_off, roll_off, gx_off, gy_off, gz_off):
    global _sim_t
    if SIMULATION:
        _sim_t += TARGET_DT
        # Gentle oscillation so the 3-D model moves visibly in the browser
        pitch = 20 * math.sin(_sim_t * 0.4)
        roll  = 15 * math.sin(_sim_t * 0.6 + 1.0)
        ax = -math.sin(math.radians(pitch))
        ay =  math.sin(math.radians(roll))
        az =  math.cos(math.radians(pitch)) * math.cos(math.radians(roll))
        gx =  15 * math.cos(_sim_t * 0.6 + 1.0) * 0.6
        gy =  20 * math.cos(_sim_t * 0.4) * 0.4
        gz =  0.0
        return ax, ay, az, gx, gy, gz, pitch, roll

    accel = MPU.get_accel_data()
    gyro  = MPU.get_gyro_data()
    ax, ay, az = accel['x'], accel['y'], accel['z']
    gx  = gyro['x'] - gx_off
    gy  = gyro['y'] - gy_off
    gz  = gyro['z'] - gz_off
    return ax, ay, az, gx, gy, gz, None, None   # angles computed by Kalman

# ══════════════════════════════════════════════════════════════════════════════
#  SHARED STATE  (filled by the sensor loop, read by WS handlers)
# ══════════════════════════════════════════════════════════════════════════════
latest_frame = {}
connected_clients = set()

# ══════════════════════════════════════════════════════════════════════════════
#  SENSOR / KALMAN LOOP  (runs in a background thread via asyncio executor)
# ══════════════════════════════════════════════════════════════════════════════
def sensor_loop(pitch_off, roll_off, gx_off, gy_off, gz_off,
                pitch_state, roll_state):
    global latest_frame
    last_time     = perf_counter()
    loop_count    = 0
    overrun_count = 0
    hz_acc        = 0.0

    while True:
        t0 = perf_counter()
        dt = t0 - last_time
        last_time = t0
        if dt <= 0:
            dt = TARGET_DT

        ax, ay, az, gx, gy, gz, sim_pitch, sim_roll = \
            read_sensor(pitch_off, roll_off, gx_off, gy_off, gz_off)

        if SIMULATION:
            pitch_angle = sim_pitch
            roll_angle  = sim_roll
            pitch_state["angle"] = pitch_angle
            roll_state["angle"]  = roll_angle
            pitch_state["bias"]  = 0.0
            roll_state["bias"]   = 0.0
        else:
            pitch_accel = math.atan2(-ax, az)                        * (180/math.pi) - pitch_off
            roll_accel  = -math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180/math.pi) - roll_off
            pitch_angle = kalman_update(pitch_state, gy, pitch_accel, dt)
            roll_angle  = kalman_update(roll_state,  gx, roll_accel,  dt)

        hz_acc += 1.0 / dt if dt > 0 else TARGET_HZ
        loop_count += 1

        if loop_count % TARGET_HZ == 0:
            avg_hz = hz_acc / TARGET_HZ
            hz_acc = 0.0
            latest_frame = {
                "pitch": round(pitch_angle, 3),
                "roll":  round(roll_angle,  3),
                "ax": round(ax, 4), "ay": round(ay, 4), "az": round(az, 4),
                "gx": round(gx, 3), "gy": round(gy, 3), "gz": round(gz, 3),
                "pitch_bias": round(pitch_state["bias"], 5),
                "roll_bias":  round(roll_state["bias"],  5),
                "loop_hz":    round(avg_hz, 1),
                "overruns":   overrun_count,
                "sim":        SIMULATION,
            }
        else:
            # Still update angles every loop so WS pushes are current
            latest_frame["pitch"] = round(pitch_angle, 3)
            latest_frame["roll"]  = round(roll_angle,  3)
            latest_frame["gx"]    = round(gx, 3)
            latest_frame["gy"]    = round(gy, 3)
            latest_frame["gz"]    = round(gz, 3)
            latest_frame["ax"]    = round(ax, 4)
            latest_frame["ay"]    = round(ay, 4)
            latest_frame["az"]    = round(az, 4)

        elapsed   = perf_counter() - t0
        remaining = TARGET_DT - elapsed
        if remaining > 0:
            sleep(remaining)
        else:
            overrun_count += 1

# ══════════════════════════════════════════════════════════════════════════════
#  WEBSOCKET  —  broadcast latest frame to all clients at ~50 Hz
# ══════════════════════════════════════════════════════════════════════════════
async def broadcast_loop():
    while True:
        if latest_frame and connected_clients:
            msg = json.dumps(latest_frame)
            # Send to all; drop clients that have disconnected
            dead = set()
            for ws in connected_clients:
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            connected_clients.difference_update(dead)
        await asyncio.sleep(TARGET_DT)

async def handler(websocket):
    print(f"[WS] Client connected: {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        print(f"[WS] Client disconnected: {websocket.remote_address}")

async def main():
    pitch_off, roll_off, gx_off, gy_off, gz_off = calibrate()

    pitch_state = kalman_init()
    roll_state  = kalman_init()

    if not SIMULATION:
        _accel = MPU.get_accel_data()
        _ax, _ay, _az = _accel['x'], _accel['y'], _accel['z']
        pitch_state["angle"] = math.atan2(-_ax, _az) * (180/math.pi) - pitch_off
        roll_state["angle"]  = -math.atan2(_ay, math.sqrt(_ax**2 + _az**2)) * (180/math.pi) - roll_off

    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, sensor_loop,
                         pitch_off, roll_off, gx_off, gy_off, gz_off,
                         pitch_state, roll_state)

    print(f"[WS] Server listening on ws://{WS_HOST}:{WS_PORT}")
    print(f"[WS] Simulation mode: {SIMULATION}")
    print("     Open the dashboard HTML in a browser on your laptop.")
    print("     Enter the Pi's IP address when prompted.\n")

    async with websockets.serve(handler, WS_HOST, WS_PORT):
        await broadcast_loop()

if __name__ == "__main__":
    signal.signal(signal.SIGINT,  lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    asyncio.run(main())
