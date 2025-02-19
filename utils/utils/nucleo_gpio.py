import lgpio
import time
from utils.exceptions import GPIOError

class NucleoGPIO:
    """
    Manages GPIO operations for the Nucleo board's reset pin.
    
    Notes:
      - Only one module should claim the output; this class opens/closes the chip as needed.
      - WARNING: The Nucleo reset pin is not 5V tolerant; never drive it high permanently.
    """
    
    def __init__(self):
        self.reset_pin = 24
        self.chip = None

    def toggle_reset(self):
        """
        Temporarily drives the reset pin high for 0.1 seconds, then sets it low.
        """
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
        """
        Sets the reset pin high and holds it there.
        Note: Remember that the pin is not 5V tolerant.
        """
        try:
            self._open()
            lgpio.gpio_claim_output(self.chip, self.reset_pin, level=1)
        except Exception as e:
            raise GPIOError(f"Error latching reset: {e}")

    def release_reset(self):
        """
        Releases the reset by setting the pin low and closing the chip.
        """
        try:
            lgpio.gpio_write(self.chip, self.reset_pin, 0)
            self._close()
        except Exception as e:
            raise GPIOError(f"Error releasing reset: {e}")

    def _open(self):
        """Opens the GPIO chip."""
        self.chip = lgpio.gpiochip_open(0)

    def _close(self):
        """Closes the GPIO chip."""
        lgpio.gpiochip_close(self.chip)
        self.chip = None

    def __del__(self):
        if self.chip:
            self._close()
