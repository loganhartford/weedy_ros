#!/mnt/shared/weedy_ros/src/decisions/decisions/venv/bin/python3
import numpy as np
import requests
from ultralytics import YOLO
from PIL import Image as PILImage
from io import BytesIO
from datetime import datetime

from utils.exceptions import ModelError, CameraError


class YOLOModel:
    def __init__(self):
        self.model_path = "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_all_ncnn_model"
        self.model = YOLO(self.model_path, task="pose", verbose=False)
        self.image_url = "http://localhost:8000"

    def run_inference(self, save_data=True, save_result=False):
        try:
            img = self.get_img()
            if save_data:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                PILImage.fromarray(img).save(f"/mnt/shared/weedy_ros/src/decisions/decisions/img/train/{timestamp}.jpg")
        except CameraError:
            return None

        try:
            results = self.model(img, verbose=False)
            result = results[0]
            if len(result) == 0:
                return None
            if save_result:
                result.save("/mnt/shared/weedy_ros/src/decisions/decisions/outputs/result.jpg")
            return result
        except Exception as e:
            raise ModelError("Error during model inference") from e

    def get_img(self):
        try:
            response = requests.get(self.image_url)
            response.raise_for_status()
            image = PILImage.open(BytesIO(response.content))
            return np.array(image)
        except requests.exceptions.RequestException as e:
            raise CameraError("Error fetching image from camera") from e

    def capture_and_save_image(self):
        try:
            response = requests.get(self.image_url)
            response.raise_for_status()
            image = PILImage.open(BytesIO(response.content))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/mnt/shared/weedy_ros/src/decisions/decisions/img/{timestamp}.jpg"
            image.save(filename)
        except requests.exceptions.RequestException as e:
            raise CameraError("Error fetching image from camera") from e
