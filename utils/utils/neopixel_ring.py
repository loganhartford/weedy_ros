from pi5neo import Pi5Neo
import time

class NeoPixelRing:
    """
    A simple wrapper for controlling a NeoPixel ring via the Pi5Neo library.
    """

    def __init__(self):
        self.neo = Pi5Neo('/dev/spidev0.0', 16, 800)
        self.last_settings = (0, 0, 0, 0)  # (r, g, b, brightness)

    def set_color(self, r, g, b, a):
        """
        Set the NeoPixel ring color.

        Args:
            r (float): Red component (0-255).
            g (float): Green component (0-255).
            b (float): Blue component (0-255).
            a (float): Brightness multiplier (0-1).
        """
        self.last_settings = (r, g, b, a)
        self.neo.fill_strip(int(r * a), int(g * a), int(b * a))
        self.neo.update_strip()

    def flash_color(self, r, g, b, a, duration=0.5):
        """
        Temporarily flash a given color on the ring.

        The ring displays the specified color for 'duration' seconds, then reverts
        to the last set color.

        Args:
            r (float): Red component (0-255).
            g (float): Green component (0-255).
            b (float): Blue component (0-255).
            a (float): Brightness multiplier (0-1).
            duration (float): Duration in seconds to display the flash color.
        """
        self.neo.fill_strip(int(r * a), int(g * a), int(b * a))
        self.neo.update_strip()
        time.sleep(duration)
        self.set_color(*self.last_settings)

    def off(self):
        """Turn off the NeoPixel ring by clearing and updating the strip."""
        self.last_settings = (0, 0, 0, 0)
        self.neo.clear_strip()
        self.neo.update_strip()

    def __del__(self):
        self.neo.clear_strip()
        self.neo.update_strip()
