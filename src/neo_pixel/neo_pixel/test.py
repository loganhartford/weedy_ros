from pi5neo import Pi5Neo

# Initialize the Pi5Neo class with 16 LEDs and an SPI speed of 800kHz
neo = Pi5Neo('/dev/spidev0.0', 16, 800)

# Fill the strip with a red color
neo.fill_strip(0, 0, 0)
neo.update_strip()  # Commit changes to the LEDs

# Set the 5th LED to blue
neo.set_led_color(4, 0, 0, 0)
neo.update_strip()