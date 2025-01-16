#!/mnt/shared/weedy_ros/src/decisions/decisions/venv/bin/python3

import numpy as np
import cv2
import requests
from ultralytics import YOLO
from PIL import Image as PILImage
from io import BytesIO

CAMERA_ERROR = -2
MODEL_ERROR = -1

class YOLOModel:
    def __init__(self):
        # YOLO Model
        self.model_path = "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_pose_ncnn_model"
        self.model = YOLO(self.model_path, task="pose", verbose=False)

        # Camera server
        self.image_url = "http://10.0.0.171:8000"
        self.image = None

    def run_inference(self, msg):
        self.img = self.get_img()
        if self.img == CAMERA_ERROR:
            return CAMERA_ERROR

        try:
            # Run inference
            results = self.model(self.img, verbose=False)
            result = results[0]
            
            if len(result) == 0:
                return None
            else:
                # NOTE: Optionally save the image
                result.save("/mnt/shared/weedy_ros/src/decisions/decisions/outputs/result.jpg")
                return result

        except Exception as e:
            return MODEL_ERROR
    
    def get_img(self):
        try:
            response = requests.get(self.image_url)
            response.raise_for_status()

            image = PILImage.open(BytesIO(response.content))
            image_np = np.array(image)[:, :, ::-1]

            return image_np
        
        except requests.exceptions.RequestException as e:
            return CAMERA_ERROR
