from pi5neo import Pi5Neo

class NeoPixelRing:
    def __init__(self):
        self.neo = Pi5Neo('/dev/spidev0.0', 16, 800)
    
    def __del__(self):
        self.neo.clear_strip()
        self.neo.update_strip()
    
    def set_color(self, r, g, b, a):
        self.neo.fill_strip(int(r * a), int(g * a), int(b * a))
        self.neo.update_strip()
    
    def off(self):
        self.neo.clear_strip()
        self.neo.update_strip()
