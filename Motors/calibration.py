import pigpio
import time

pi = pigpio.pi()

ESC_PIN = 18

print("ESC Calibration")
print("===============")
print("Step 1 — Disconnect battery now!")
input("Press Enter when battery is disconnected...")

# Send max throttle signal
print("Sending MAX throttle signal (2000µs)...")
pi.set_servo_pulsewidth(ESC_PIN, 2000)

print("Step 2 — Connect battery NOW")
print("Wait for ESC beeps...")
input("Press Enter after you hear the beeps...")

# Send min throttle signal
print("Sending MIN throttle signal (1000µs)...")
pi.set_servo_pulsewidth(ESC_PIN, 1000)

print("Wait for confirmation beeps...")
time.sleep(3)

print("Calibration complete!")
pi.set_servo_pulsewidth(ESC_PIN, 0)  # turn off signal
pi.stop()
