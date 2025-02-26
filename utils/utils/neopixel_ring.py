import time
from pi5neo import Pi5Neo

class NeoPixelRing:
    def __init__(self):
        self.neo = Pi5Neo('/dev/spidev0.0', 16, 800)
        self.last_settings = (0, 0, 0, 0)  # (r, g, b, brightness)
        self.animation_index = 0  # Keeps track of the current animation step

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
        self.last_settings = (0, 0, 0, 0.0)
        self.neo.clear_strip()
        self.neo.update_strip()

    def step_animation(self):
        num_pixels = 16
        brightness = 1.0
        fade1 = 0.2
        fade2 = 0.1
        r, g, b = 255, 255, 0  # Yellow rotating pixel

        self.neo.clear_strip()

        idx1 = self.animation_index
        trail1 = (self.animation_index - 1) % num_pixels
        trail2 = (self.animation_index - 2) % num_pixels
        lead1 = (self.animation_index + 1) % num_pixels
        lead2 = (self.animation_index + 2) % num_pixels

        self.neo.set_led_color(idx1, int(r * brightness), int(g * brightness), int(b * brightness))
        self.neo.set_led_color(trail1, int(r * fade1), int(g * fade1), int(b * fade1))
        self.neo.set_led_color(trail2, int(r * fade2), int(g * fade2), int(b * fade2))
        self.neo.set_led_color(lead1, int(r * fade1), int(g * fade1), int(b * fade1))
        self.neo.set_led_color(lead2, int(r * fade2), int(g * fade2), int(b * fade2))

        self.neo.update_strip()

        self.animation_index = (self.animation_index + 1) % num_pixels


    def __del__(self):
        self.neo.clear_strip()
        self.neo.update_strip()
