import time
import board
import busio
import digitalio
from circuitpython_nrf24l01.rf24 import RF24

# Initialize SPI bus and pins
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
ce = digitalio.DigitalInOut(board.D22)
csn = digitalio.DigitalInOut(board.D8)

# Initialize the nRF24L01
nrf = RF24(spi, csn, ce)

# Basic connection check
if not nrf.is_lna:
    print("Hardware check failed: nRF24L01 not found. Check wiring!")
else:
    print("nRF24L01 Detected!")
    nrf.print_details() # Prints detailed register info for verification

    print("\nScanning for active RF channels...")
    # Simple scanner loop
    for channel in range(126):
        nrf.channel = channel
        nrf.listen = True
        time.sleep(0.01) # Short delay to catch signals
        if nrf.carrier_detect():
            print(f"Activity detected on Channel {channel}")
        nrf.listen = False
