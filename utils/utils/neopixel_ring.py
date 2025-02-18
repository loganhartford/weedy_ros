from pi5neo import Pi5Neo
import time

class NeoPixelRing:
    def __init__(self):
        self.neo = Pi5Neo('/dev/spidev0.0', 16, 800)

        self. last_settings = (0, 0, 0, 0)
    
    def __del__(self):
        self.neo.clear_strip()
        self.neo.update_strip()
    
    def set_color(self, r, g, b, a):
        self.last_settings = (r, g, b, a)
        self.neo.fill_strip(int(r * a), int(g * a), int(b * a))
        self.neo.update_strip()

    def flash_color(self, r, g, b, a, duration=0.5):
        self.neo.fill_strip(int(r * a), int(g * a), int(b * a))
        self.neo.update_strip()
        time.sleep(duration)
        self.set_color(*self.last_settings)
    
    def off(self):
        self.neo.clear_strip()
        self.neo.update_strip()
