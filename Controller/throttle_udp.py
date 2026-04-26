"""
Game Controller Input Detector
Detects and displays which buttons/keys are being pressed on a connected game controller.
Works with Xbox, PlayStation, and generic USB controllers connected via dongle/wire.

Requirements:
    pip install inputs

Usage:
    sudo python controller_input_detector.py
    
    Note: Port 80 requires administrator privileges (sudo).
    The Raspberry Pi can fetch throttle data from: http://<your-ip>/throttle
"""

import sys
import time
import json
import socket
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from inputs import get_gamepad, devices, GamePad
from collections import defaultdict

sys.stdout.reconfigure(encoding='utf-8')

class ReusableHTTPServer(HTTPServer):
    """HTTP Server that allows address reuse."""
    allow_reuse_address = True
    allow_reuse_port = True


class ThrottleHTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for throttle data API endpoint."""
    
    # Reference to the controller detector instance
    detector = None
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/throttle' or self.path == '/':
            # Return throttle data as JSON
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            response = json.dumps({"throttle": self.detector.throttle_percent})
            self.wfile.write(response.encode('utf-8'))
        else:
            # Return 404 for other paths
            self.send_response(404)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            response = json.dumps({"error": "Not found"})
            self.wfile.write(response.encode('utf-8'))
    
    def log_message(self, format, *args):
        """Suppress default logging to keep console clean."""
        pass


class ControllerDetector:
    def __init__(self, host='0.0.0.0', port=80):
        self.pressed_buttons = set()
        self.axis_values = {}
        self.running = True
        
        # HTTP server configuration
        self.host = host
        self.port = port
        self.http_server = None
        self.server_thread = None
        
        # Track last throttle value to detect changes
        self.last_throttle_sent = -1
        
        # Controller state tracking
        self.button_states = defaultdict(bool)
        self.axis_states = {}
        
        # Deadzone threshold to ignore small drift/noise (0.1 = 10% of full range)
        self.deadzone = 0.1
        
        # Throttle control feature
        self.throttle_percent = 0
        self.throttle_step = 2  # Increment/decrement by 2%
        self.throttle_min = 0
        self.throttle_max = 100
        
        # Track D-pad button press state to prevent rapid firing
        self.dpad_up_pressed = False
        self.dpad_down_pressed = False
        
    def detect_controllers(self):
        """Detect connected game controllers."""
        print("=" * 60)
        print("CONTROLLER INPUT DETECTOR")
        print("=" * 60)
        print("\nSearching for connected controllers...\n")
        
        # Check for game controllers
        if devices.gamepads:
            print(f"✓ Found {len(devices.gamepads)} controller(s):")
            for i, gamepad in enumerate(devices.gamepads, 1):
                # Get controller name - handle different versions of inputs library
                name = getattr(gamepad, 'name', 'Unknown Controller')
                # Try to get more info about the controller
                phys = getattr(gamepad, 'phys', 'N/A')
                print(f"  - Controller {i}: {name}")
                print(f"    Physical connection: {phys}")
            print()
            return True
        else:
            print("✗ No game controllers detected!")
            print("\nTroubleshooting tips:")
            print("  1. Make sure your controller is connected (USB/wireless dongle)")
            print("  2. Try pressing a button on the controller to wake it up")
            print("  3. Check if the controller is paired/connected in system settings")
            print("  4. Try reconnecting the USB dongle/cable")
            print("  5. Some wireless controllers need to be in 'wired mode' to work")
            return False
    
    def format_button_name(self, code):
        """Format button codes into readable names."""
        button_names = {
            # Xbox/Standard buttons
            'BTN_SOUTH': 'A',
            'BTN_EAST': 'B', 
            'BTN_C': 'X',
            'BTN_Z': 'Y',
            'BTN_WEST': 'X',
            'BTN_NORTH': 'Y',
            
            # Shoulder buttons
            'BTN_TL': 'LB (Left Bumper)',
            'BTN_TR': 'RB (Right Bumper)',
            'BTN_TL2': 'LT (Left Trigger)',
            'BTN_TR2': 'RT (Right Trigger)',
            
            # Center buttons
            'BTN_SELECT': 'Back/Select',
            'BTN_START': 'Start',
            'BTN_THUMBL': 'LS (Left Stick Click)',
            'BTN_THUMBR': 'RS (Right Stick Click)',
            'BTN_MODE': 'Guide/Home',
            
            # Common joystick/gamepad buttons
            'BTN_TRIGGER': 'Button 1 (Trigger)',
            'BTN_THUMB': 'Button 2 (Thumb)',
            'BTN_THUMB2': 'Button 3 (Thumb2)',
            'BTN_TOP': 'Button 4 (Top)',
            'BTN_TOP2': 'Button 5 (Top2)',
            'BTN_PINKIE': 'Button 6 (Pinkie)',
            'BTN_BASE': 'Button 7 (Base)',
            'BTN_DEAD': 'Button 8 (Dead)',
        }
        
        return button_names.get(code, code)
    
    def format_axis_name(self, code):
        """Format axis codes into readable names."""
        axis_names = {
            'ABS_X': 'Left Stick X',
            'ABS_Y': 'Left Stick Y',
            'ABS_Z': 'Right Stick X',
            'ABS_RZ': 'Right Stick Y',
            'ABS_RX': 'Left Trigger',
            'ABS_RY': 'Right Trigger',
            'ABS_HAT0X': 'D-Pad Horizontal',
            'ABS_HAT0Y': 'D-Pad Vertical',
            # Alternative D-pad axis names
            'ABS_HAT0': 'D-Pad',
            'ABS_HAT1X': 'D-Pad 2 X',
            'ABS_HAT1Y': 'D-Pad 2 Y',
        }
        
        return axis_names.get(code, code)
    
    def get_dpad_direction(self, x_value, y_value):
        """Convert D-pad axis values to direction."""
        # D-pad typically returns -1, 0, or 1
        if y_value < -0.5:
            return "UP"
        elif y_value > 0.5:
            return "DOWN"
        elif x_value < -0.5:
            return "LEFT"
        elif x_value > 0.5:
            return "RIGHT"
        return None
    
    def display_status(self):
        """Display current controller status."""
        # Clear screen (works on both Windows and Unix)
        print("\033[H\033[J", end="")
        
        print("=" * 60)
        print("CONTROLLER INPUT DETECTOR (Press Ctrl+C to exit)")
        print("=" * 60)
        print()
        
        # Display active buttons
        print("ACTIVE BUTTONS:")
        print("-" * 30)
        if self.pressed_buttons:
            for button in sorted(self.pressed_buttons):
                print(f"  ● {self.format_button_name(button)}")
        else:
            print("  (none)")
        
        # Display axis values
        print("\nANALOG STICKS & TRIGGERS:")
        print("-" * 30)
        if self.axis_values:
            has_activity = False
            for axis, value in sorted(self.axis_values.items()):
                # The stored values are already normalized (-1 to 1) with deadzone applied
                # For D-pad (HAT), values are typically -1, 0, 1
                if 'HAT' in axis:
                    normalized = float(value)
                else:
                    # Values for sticks/triggers are already normalized and deadzoned
                    normalized = float(value)
                
                # Skip if value is 0 or very close to 0 (centered/neutral)
                if abs(normalized) < 0.01:
                    continue
                
                has_activity = True
                bar_length = 30
                fill = int(abs(normalized) * bar_length)
                direction = "→" if normalized > 0 else "←"
                bar = "█" * fill + "░" * (bar_length - fill)
                axis_name = self.format_axis_name(axis)
                
                # Add D-pad direction if applicable
                if 'HAT0X' in axis or 'HAT0Y' in axis:
                    x_val = self.axis_values.get('ABS_HAT0X', 0)
                    y_val = self.axis_values.get('ABS_HAT0Y', 0)
                    # Convert to float for D-pad
                    x_val = float(x_val) if x_val != 0 else 0
                    y_val = float(y_val) if y_val != 0 else 0
                    direction_text = self.get_dpad_direction(x_val, y_val)
                    if direction_text:
                        print(f"  {axis_name:25} [{bar}] {direction} {normalized:+.2f}  → {direction_text}")
                    else:
                        print(f"  {axis_name:25} [{bar}] {direction} {normalized:+.2f}")
                else:
                    print(f"  {axis_name:25} [{bar}] {direction} {normalized:+.2f}")
            
            if not has_activity:
                print("  (all centered)")
        else:
            print("  (all centered)")
        
        # Display throttle control
        print("\nTHROTTLE:")
        print("-" * 30)
        throttle_bar_length = 50
        throttle_fill = int((self.throttle_percent / 100) * throttle_bar_length)
        throttle_bar = "█" * throttle_fill + "░" * (throttle_bar_length - throttle_fill)
        print(f"  [{throttle_bar}] {self.throttle_percent:3d}%")
        print("  D-pad UP: +2%  |  D-pad DOWN: -2%")
        
        # Show if throttle value has changed and will be sent to Pi
        if self.throttle_percent != self.last_throttle_sent:
            print("  ✓ Changed - will be sent to Raspberry Pi")
        
        print("\n" + "=" * 60)
    
    def start_http_server(self):
        """Start the HTTP server in a background thread."""
        try:
            # Set the detector reference for the handler
            ThrottleHTTPHandler.detector = self
            
            # Create and start the HTTP server (using ReusableHTTPServer to allow address reuse)
            self.http_server = ReusableHTTPServer((self.host, self.port), ThrottleHTTPHandler)
            self.server_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
            self.server_thread.start()
            
            # Get local IP address for display
            local_ip = self.get_local_ip()
            
            print(f"\n✓ HTTP server started on port {self.port}")
            print(f"  Throttle API endpoint: http://{local_ip}:{self.port}/throttle")
            print(f"  Local access: http://localhost:{self.port}/throttle")
            print(f"\n  Raspberry Pi should fetch data from: http://{local_ip}:{self.port}/throttle")
            print("=" * 60)
            
        except OSError as e:
            print(f"\n✗ Failed to start HTTP server on port {self.port}")
            print(f"  Error: {e}")
            print("  Make sure port 80 is not already in use.")
            print("  Try running with sudo/administrator privileges (port 80 requires elevated permissions).")
            sys.exit(1)
    
    def get_local_ip(self):
        """Get the local IP address of this machine."""
        try:
            # Create a socket to determine the local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"
    
    def monitor_controller(self):
        """Main loop to monitor controller inputs."""
        print("Initializing controller monitoring...")
        
        # Get the first available gamepad
        if not devices.gamepads:
            print("No gamepads found!")
            return
        
        gamepad = devices.gamepads[0]
        print(f"Using controller: {gamepad.name}")
        print("\n" + "=" * 60)
        print("CONTROLLER INPUT DETECTOR")
        print("=" * 60)
        print("\nWaiting for input...")
        print("Press any button, move a stick, or press a trigger on your controller.")
        print("(Press Ctrl+C to exit)\n")
        
        # Give user a moment to get ready
        for i in range(3, 0, -1):
            print(f"Starting in {i}...", end="\r")
            time.sleep(1)
        
        try:
            while self.running:
                try:
                    # Get all available gamepad events
                    # This will block and wait for input
                    events = get_gamepad()
                    
                    for event in events:
                        # Handle button presses
                        if event.ev_type == 'Key':
                            if event.state == 1:  # Button pressed
                                self.pressed_buttons.add(event.code)
                                self.button_states[event.code] = True
                            elif event.state == 0:  # Button released
                                self.pressed_buttons.discard(event.code)
                                self.button_states[event.code] = False
                        
                        # Handle analog inputs (sticks, triggers, D-pad)
                        # Capture ALL absolute axis events for maximum compatibility
                        elif event.ev_type == 'Absolute':
                            # Apply deadzone to reduce drift on analog sticks
                            raw_value = event.state
                            
                            # For D-pad (HAT), keep original values (-1, 0, 1)
                            # Also handle throttle control with D-pad up/down
                            if 'HAT' in event.code:
                                self.axis_values[event.code] = raw_value
                                
                                # Throttle control: D-pad up increases, D-pad down decreases
                                if event.code == 'ABS_HAT0Y':
                                    old_throttle = self.throttle_percent
                                    if raw_value == -1:  # D-pad UP pressed
                                        if not self.dpad_up_pressed:
                                            self.throttle_percent = min(self.throttle_max, 
                                                                        self.throttle_percent + self.throttle_step)
                                            self.dpad_up_pressed = True
                                    else:
                                        self.dpad_up_pressed = False
                                    
                                    if raw_value == 1:  # D-pad DOWN pressed
                                        if not self.dpad_down_pressed:
                                            self.throttle_percent = max(self.throttle_min, 
                                                                        self.throttle_percent - self.throttle_step)
                                            self.dpad_down_pressed = True
                                    else:
                                        self.dpad_down_pressed = False
                                    
                                    # Check if throttle value changed
                                    if old_throttle != self.throttle_percent:
                                        self.last_throttle_sent = -1  # Mark as needing update
                            else:
                                # Normalize the value first
                                # Determine the appropriate normalization based on typical controller ranges
                                if abs(raw_value) <= 127:
                                    # Range: -128 to 127 (center at 0)
                                    normalized = raw_value / 128.0
                                elif abs(raw_value) <= 255:
                                    # Range: 0 to 255 (center at 128)
                                    normalized = (raw_value - 128) / 128.0
                                else:
                                    # Range: -32768 to 32767 (center at 0)
                                    normalized = raw_value / 32768.0
                                
                                # Apply deadzone - set to 0 if within deadzone range
                                if abs(normalized) < self.deadzone:
                                    # Store 0 to indicate centered/neutral position
                                    self.axis_values[event.code] = 0.0
                                else:
                                    # Re-scale to full range after deadzone
                                    # This makes the stick more responsive after leaving deadzone
                                    if normalized > 0:
                                        scaled = (normalized - self.deadzone) / (1 - self.deadzone)
                                    else:
                                        scaled = (normalized + self.deadzone) / (1 - self.deadzone)
                                    # Store the scaled value
                                    self.axis_values[event.code] = scaled
                        
                        # Handle D-pad as buttons (some controllers report it this way)
                        elif event.ev_type == 'Key' and 'HAT' in event.code:
                            if event.state == 1:
                                self.pressed_buttons.add(event.code)
                            elif event.state == 0:
                                self.pressed_buttons.discard(event.code)
                    
                    # Always update display after processing events
                    self.display_status()
                    
                except EOFError:
                    # No events available, wait a bit
                    time.sleep(0.05)
                    continue
                    
                except OSError as e:
                    # Handle device disconnection
                    print(f"\nDevice error: {e}")
                    print("Controller may have been disconnected.")
                    time.sleep(1)
                    # Try to redetect controllers
                    if not devices.gamepads:
                        print("No controllers found. Exiting...")
                        self.running = False
                    continue
                    
        except KeyboardInterrupt:
            print("\n\nStopping controller monitor...")
            self.running = False
    
    def run(self):
        """Main execution method."""
        if not self.detect_controllers():
            print("\nExiting...")
            sys.exit(1)
        
        # Start the HTTP server for throttle data
        self.start_http_server()
        
        print("\nStarting input monitor...\n")
        self.monitor_controller()
        
        # Shutdown HTTP server
        if self.http_server:
            self.http_server.shutdown()
        
        print("\nThanks for using Controller Input Detector!")


def main():
    """Entry point for the script."""
    try:
        detector = ControllerDetector()
        detector.run()
    except Exception as e:
        print(f"\nError: {e}")
        print("\nMake sure you have the 'inputs' library installed:")
        print("  pip install inputs")
        sys.exit(1)


if __name__ == "__main__":
    main()
