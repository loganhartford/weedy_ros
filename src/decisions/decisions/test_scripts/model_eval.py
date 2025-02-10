#! /mnt/shared/weedy_ros/src/decisions/decisions/venv/bin/python3

from ultralytics import YOLO
from pi5neo import Pi5Neo
import requests
from PIL import Image
from io import BytesIO
import time
import numpy as np
import csv
import math
import cv2

pixel_points = np.array([
    [1291, 953],
    [694, 964],
    [1410, 652],
    [986, 659],
    [563, 668],
    [1385, 242],
    [977, 251],
    [571, 258],
    [982, 451],
    [991, 876],
    [1198, 655],
    [774, 664],
    [1262, 371],
    [699, 382],
], dtype=np.float32)

x = 0
y = 30
ground_points = np.array([
    [x+30, y+30],
    [x+30, y+160],
    [x+95, y+0],
    [x+95, y+95],
    [x+95, y+190],
    [x+190, y+0],
    [x+190, y+95],
    [x+190, y+190],
    [x+142.5, y+95],
    [x+47.5, y+95],
    [x+95, y+47.5],
    [x+95, y+142.5],
    [x+160, y+30],
    [x+160, y+160],
], dtype=np.float32)


models = {
    # "indoor_pose": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_pose_ncnn_model",
    # "indoor_angled": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_angled_ncnn_model",
    "indoor_all": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_all_ncnn_model",
    # "indoor_indoor": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_indoor_ncnn_model",
}

def init_picture():
    neo.fill_strip(int(255 * a), int(255 * a), int(255 * a))
    neo.update_strip()

    time.sleep(3)

    get_image()

def get_image():
    try:
        response = requests.get(url)
        response.raise_for_status()

        image = Image.open(BytesIO(response.content))

        image_path = f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/eval/{test_name}/downloaded_image.jpg"
        image.save(image_path)

        return image_path

    except requests.exceptions.RequestException as e:
        print("Failed to fetch image:", e)

def homography_transform(pixel):
    # Convert the point to homogeneous coordinates.
    image_point = np.array([pixel[0], pixel[1], 1], dtype=np.float32)
    # Apply the homography matrix.
    ground_point = np.dot(H, image_point)
    # Normalize to convert from homogeneous coordinates.
    ground_point /= ground_point[2]
    return ground_point[:2]

def main():
    print()
    print("################# TEST #################")
    print()
    # Turn on light
    neo.fill_strip(int(255 * a), int(255 * a), int(255 * a))
    neo.update_strip()

    # Wait for light to turn on
    time.sleep(3)

    results_data = {model_name: {"distances": [], "confidences": [], "box_conf": [], "point": [], "num_boxes": []} for model_name in models}

    # Load models
    loaded_models = {model_name: YOLO(model_path, task="pose", verbose=False) for model_name, model_path in models.items()}

    # Run inference
    for iter in range(5):
        image_path = get_image()

        for model_name, model in loaded_models.items():
            # Run inference
            results = model(image_path, verbose=False)
            result = results[0]

            # If no results, add placeholder values
            if len(result) == 0:
                results_data[model_name]["distances"].append(1080)
                results_data[model_name]["confidences"].append(0)
                results_data[model_name]["box_conf"].append(0)
                results_data[model_name]["point"].append(-1)
                results_data[model_name]["num_boxes"].append(0)
            else:
                # Save image
                result.save(f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/eval/{test_name}/{model_name}_{iter}.jpg")

                # Calculate distances and confidences
                best_distance = 1080
                best_confidence = 0
                best_box_conf = 0
                point = -1
                for i, kp in enumerate(result.keypoints):
                    if not kp.has_visible:
                        continue

                    keypoints = list(zip(["flower", "base", "upper", "lower"], kp.data[0]))
                    keypoints = sorted(keypoints, key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
                    for j, (name, tensor) in enumerate(keypoints):
                        if all(tensor.numpy() != 0):
                            coords = homography_transform((tensor[0], tensor[1]))
                            distance = math.sqrt((coords[0] - actual_base[0]) ** 2 + (coords[1] - actual_base[1]) ** 2)
                            confidence = tensor[2]
                            if distance < best_distance:
                                best_distance = distance
                                best_confidence = confidence
                                best_box_conf = result.boxes.conf[i]
                                point = j

                results_data[model_name]["distances"].append(best_distance)
                results_data[model_name]["confidences"].append(best_confidence)
                results_data[model_name]["box_conf"].append(best_box_conf)
                results_data[model_name]["point"].append(point)
                results_data[model_name]["num_boxes"].append(len(result.boxes))

    # Calculate and print averages
    for model_name in models:
        print()
        print(f"Results for {model_name}:")
        average_distance = np.mean(results_data[model_name]["distances"])
        average_confidence = np.mean(results_data[model_name]["confidences"])
        average_box_conf = np.mean(results_data[model_name]["box_conf"])
        average_point = np.mean(results_data[model_name]["point"])
        average_num_boxes = np.mean(results_data[model_name]["num_boxes"])
        print(f"Average best Euclidean distance for {model_name}: {average_distance:.2f}mm")
        print(f"Average confidence for {model_name}: {average_confidence:.2f}")
        print(f"Average box confidence for {model_name}: {average_box_conf:.2f}")
        print(f"Average point for {model_name}: {average_point:.2f}")
        print(f"Average number of boxes for {model_name}: {average_num_boxes:.2f}")

        results_data[model_name] = [average_distance, average_confidence, average_box_conf, average_point, average_num_boxes]

    # Save data to CSV
    csv_path = f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/eval/{test_name}/results.csv"
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Model Name", "Average Euclidean Distance (mm)", "Average Point Confidence", "Average Box Confidence", "Average Point (0-3)", "Average Number of Boxes"])
        for model_name, data in results_data.items():
            writer.writerow([model_name] + data)
    print(f"Results saved to {csv_path}")

def cleanup():
    neo.clear_strip()
    neo.update_strip()

url = "http://localhost:8000"
neo = Pi5Neo('/dev/spidev0.0', 16, 800)
a = 1.0

H, status = cv2.findHomography(pixel_points, ground_points, cv2.RANSAC, 5.0)

base_pixels = (843, 620)
actual_base = homography_transform(base_pixels)
test_name = "set2/test6"

if __name__ == "__main__":

    main()

    # init_picture()

    cleanup()
