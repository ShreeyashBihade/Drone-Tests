import requests
import time

URL = "http://192.168.1.40/throttle"
POLL_INTERVAL = 0.02  # seconds (50Hz)

def get_throttle():
    response = requests.get(URL, timeout=2)
    return response.json()["throttle"]

print("Fetching throttle data... (Ctrl+C to stop)")

while True:
    try:
        throttle = get_throttle()
        print(f"Throttle: {throttle}%", end="\r")
    except requests.ConnectionError:
        print("Connection failed - is the controller PC running?", end="\r")
    except requests.Timeout:
        print("Request timed out", end="\r")
    except Exception as e:
        print(f"Error: {e}", end="\r")

    time.sleep(POLL_INTERVAL)
