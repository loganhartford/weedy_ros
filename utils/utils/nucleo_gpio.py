import lgpio
import time
from utils.exceptions import GPIOError

class NucleoGPIO:
    def __init__(self):
        self.reset_pin = 24
        self.chip = None

    def toggle_reset(self):
        try:
            self._open()
            lgpio.gpio_claim_output(self.chip, self.reset_pin, level=1)
            time.sleep(0.001)
            lgpio.gpio_write(self.chip, self.reset_pin, 0)
            self._close()
            time.sleep(0.001)
        except Exception as e:
            raise GPIOError(f"Error toggling reset: {e}")

    def latch_reset(self):
        try:
            self._open()
            lgpio.gpio_claim_output(self.chip, self.reset_pin, level=1)
        except Exception as e:
            raise GPIOError(f"Error latching reset: {e}")

    def release_reset(self):
        try:
            lgpio.gpio_write(self.chip, self.reset_pin, 0)
            self._close()
        except Exception as e:
            raise GPIOError(f"Error releasing reset: {e}")

    def _open(self):
        self.chip = lgpio.gpiochip_open(0)

    def _close(self):
        lgpio.gpiochip_close(self.chip)
        self.chip = None

    def __del__(self):
        if self.chip:
            self._close()
