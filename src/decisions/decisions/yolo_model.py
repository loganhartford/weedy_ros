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

    def run_inference(self, save_data=False, save_result=False):
        try:
            img = self.get_img()
            if save_data:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                img.save(f"/mnt/shared/weedy_ros/src/decisions/decisions/img/train/{timestamp}.jpg")
        except CameraError:
            return None

        try:
            results = self.model(img, verbose=False)
            result = results[0]
            result.save(f"/mnt/shared/weedy_ros/src/decisions/decisions/img/ModelOutput.jpg")
            if len(result) == 0:
                return None
            if save_result:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                result.save(f"/mnt/shared/weedy_ros/src/decisions/decisions/img/outputs/result_{timestamp}.jpg")
                
                
            return result
        except Exception as e:
            raise ModelError("Error during model inference") from e

    def get_img(self):
        try:
            response = requests.get(self.image_url)
            response.raise_for_status()
            image = PILImage.open(BytesIO(response.content))
            return image
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

if __name__ == "__main__":
    model = YOLOModel()
    model.run_inference(save_data=True, save_result=True)
    model.capture_and_save_image()
