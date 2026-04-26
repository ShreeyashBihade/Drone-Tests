from mpu6050 import mpu6050
from time import sleep
import math

MPU = mpu6050(0x68)

while True:
        accel = MPU.get_accel_data()
        gyro = MPU.get_gyro_data()

        ax, ay, az = accel['x'], accel['y'], accel['z']

        pitch = math.atan2(-ax, az) * (180/math.pi)
        roll = math.atan2(ay, math.sqrt(ax**2 * az**2)) * (180/math.pi)

        print(f"Pitch : {pitch:.2f}, Roll : {roll:.2f}")

        sleep(0.1)
