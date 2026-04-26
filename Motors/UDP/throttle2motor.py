import requests
import time
from rpi_hardware_pwm import HardwarePWM

URL = "http://192.168.1.40/throttle"
POLL_INTERVAL = 0.02  # 50Hz

PWM_FREQ = 50          # Hz
PWM_MIN_US = 1000      # microseconds - zero throttle
PWM_MAX_US = 2000      # microseconds - full throttle
PERIOD_US  = 1_000_000 / PWM_FREQ  # 20,000µs per cycle

def throttle_to_duty(throttle_percent):
    """Convert 0-100% throttle to duty cycle percentage for 50Hz PWM."""
    pulse_us = PWM_MIN_US + (throttle_percent / 100) * (PWM_MAX_US - PWM_MIN_US)
    return (pulse_us / PERIOD_US) * 100  # duty cycle as %

def get_throttle():
    response = requests.get(URL, timeout=2)
    return response.json()["throttle"]

# Setup hardware PWM on GPIO 18 (channel 0)
pwm = HardwarePWM(pwm_channel=0, hz=PWM_FREQ, chip=0)
pwm.start(throttle_to_duty(0))  # Start at zero throttle

print("Arming ESC - sending zero throttle for 2 seconds...")
time.sleep(2)
print("Armed! Fetching throttle... (Ctrl+C to stop)")

try:
    while True:
        try:
            throttle = get_throttle()
            duty = throttle_to_duty(throttle)
            pwm.change_duty_cycle(duty)
            print(f"Throttle: {throttle:3d}%  |  Pulse: {PWM_MIN_US + (throttle/100)*1000:.0f}µs  |  Duty: {duty:.2f}%", end="\r")
        except requests.ConnectionError:
            # Safety: drop to zero if connection lost
            pwm.change_duty_cycle(throttle_to_duty(0))
            print("Connection lost - throttle zeroed!          ", end="\r")
        except requests.Timeout:
            pwm.change_duty_cycle(throttle_to_duty(0))
            print("Timeout - throttle zeroed!                  ", end="\r")

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("\nStopping...")
    pwm.change_duty_cycle(throttle_to_duty(0))
    time.sleep(0.5)
    pwm.stop()
    print("Done.")
