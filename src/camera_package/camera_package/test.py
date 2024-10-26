import requests
from PIL import Image
from io import BytesIO

# Replace '<raspberry-pi-ip>' with the actual IP address of your Raspberry Pi
url = "http://10.0.0.171:8000"

try:
    response = requests.get(url)
    response.raise_for_status()  # Check for errors

    # Load image from response content
    image = Image.open(BytesIO(response.content))

    # Save the image to a file
    image.save("downloaded_image.jpg")
    print("Image saved as downloaded_image.jpg")

except requests.exceptions.RequestException as e:
    print("Failed to fetch image:", e)
