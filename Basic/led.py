import RPi.GPIO as GPIO
import time

LED_PIN = 2

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

print("CTRL+C to stop")

try:
        while True:
                GPIO.output(LED_PIN, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(LED_PIN, GPIO.LOW)
                time.sleep(1)
except KeyboardInterrupt:
        print("Program Stopped")
finally:
        GPIO.cleanup()
        print("GPIO Released")
