from evdev import InputDevice, categorize, ecodes

controller = InputDevice('/dev/input/event7')  # Update to match your device path

print(f"Listening on {controller.path}...")

for event in controller.read_loop():
    if event.type == ecodes.EV_KEY:  # Button Presses
        print(f"Button {event.code} {'pressed' if event.value else 'released'}")
    elif event.type == ecodes.EV_ABS:  # Joystick Movement
        print(f"Joystick {event.code} moved to {event.value}")