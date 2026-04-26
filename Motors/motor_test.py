import pigpio
import time

pi = pigpio.pi()
ESC_PIN = 18

def set_throttle(percent):
    """
    percent: 0.0 to 100.0
    Maps to 1000µs (stop) to 2000µs (full)
    """
    pulse = int(1000 + (percent / 100.0) * 1000)
    pulse = max(1000, min(2000, pulse))  # clamp for safety
    pi.set_servo_pulsewidth(ESC_PIN, pulse)
    return pulse

try:
    print("Motor Test — 2200kV BLDC")
    print("========================")
    print("Arming ESC...")
    set_throttle(0)   # send zero throttle to arm
    time.sleep(3)     # wait for ESC arm beep
    print("ESC Armed!")

    # Gentle ramp up test
    print("\nRamping up slowly to 20%...")
    for pct in range(0, 21, 1):
        pulse = set_throttle(pct)
        print(f"  Throttle: {pct:3d}%  Pulse: {pulse}µs")
        time.sleep(0.1)

    print("\nHolding at 20% for 3 seconds...")
    time.sleep(3)

    print("\nRamping back down...")
    for pct in range(20, -1, -1):
        set_throttle(pct)
        time.sleep(0.1)

    print("\nMotor stopped. Test complete!")

except KeyboardInterrupt:
    print("\nInterrupted! Stopping motor...")

finally:
    # Always stop motor on exit
    set_throttle(0)
    pi.set_servo_pulsewidth(ESC_PIN, 0)
    pi.stop()
    print("Safe — motor signal off.")
