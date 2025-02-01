#!/mnt/shared/weedy_ros/src/decisions/decisions/venv/bin/python3

import numpy as np
import cv2
import requests
from ultralytics import YOLO
from PIL import Image as PILImage
from io import BytesIO

from utils.exceptions import ModelError, CameraError
from datetime import datetime

class YOLOModel:
    def __init__(self):
        # YOLO Model
        self.model_path = "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_pose_ncnn_model"
        self.model = YOLO(self.model_path, task="pose", verbose=False)

        # Camera server
        self.image_url = "http://localhost:8000"
        self.image = None

    def run_inference(self, save=False):
        try:
            self.img = self.get_img()
        except CameraError:
            return None

        try:
            # Run inference
            results = self.model(self.img, verbose=False)
            result = results[0]
            
            if len(result) == 0:
                return None
            else:
                if save: result.save("/mnt/shared/weedy_ros/src/decisions/decisions/outputs/result.jpg")
                return result

        except Exception as e:
            raise ModelError("Error during model inference") from e
    
    def get_img(self):
        try:
            response = requests.get(self.image_url)
            response.raise_for_status()

            image = PILImage.open(BytesIO(response.content))
            image_np = np.array(image)[:, :, ::-1]

            return image_np
        
        except requests.exceptions.RequestException as e:
            raise CameraError("Error fetching image from camera") from e
    
    def capture_and_save_image(self):
        try:
            response = requests.get(self.image_url)
            response.raise_for_status()

            image = PILImage.open(BytesIO(response.content))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename_with_timestamp = f"/mnt/shared/weedy_ros/src/decisions/decisions/img/{timestamp}.jpg"
            
            image.save(filename_with_timestamp)
            
        except requests.exceptions.RequestException as e:
            raise CameraError("Error fetching image from camera") from e
