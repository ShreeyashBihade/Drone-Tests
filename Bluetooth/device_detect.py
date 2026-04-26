import asyncio
from bleak import BleakScanner

seen = {}

def detection_callback(device, advertisement_data):
    name = device.name or "Unknown"
    mac = device.address

    # NEW: RSSI now comes from advertisement_data
    rssi = advertisement_data.rssi

    if mac not in seen:
        seen[mac] = rssi
        print(f"🟢 NEW DEVICE: {name} | {mac} | RSSI: {rssi} dBm")
    else:
        old_rssi = seen[mac]
        seen[mac] = rssi

        # movement / signal change detection
        if abs(old_rssi - rssi) > 5:
            print(f"📶 UPDATE: {name} | {mac} | RSSI: {rssi} dBm")

async def main():
    print("🔵 BLE RSSI tracking started...\n")

    scanner = BleakScanner(detection_callback)

    while True:
        await scanner.start()
        await asyncio.sleep(3)
        await scanner.stop()

        # FIXED: use correct modern API
        current = set(d.address for d in scanner.discovered_devices)

        for mac in list(seen.keys()):
            if mac not in current:
                print(f"🔴 OUT OF RANGE: {mac}")
                del seen[mac]

asyncio.run(main())
