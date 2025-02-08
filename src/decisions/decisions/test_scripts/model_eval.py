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



models = {
    "indoor_pose": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_pose_ncnn_model",
    "indoor_angled": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_angled_ncnn_model",
    "indoor_all": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_all_ncnn_model",
    "indoor_indoor": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_indoor_ncnn_model",
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

def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))




def main():
    print()
    print("################# TEST #################")
    print()
    # Turn on light
    neo.fill_strip(int(255 * a), int(255 * a), int(255 * a))
    neo.update_strip()

    # Wait for light to turn on
    time.sleep(3)

    results_data = {model_name: {"distances": [], "confidences": [], "box_conf": [], "point": []} for model_name in models}

    # Load models
    loaded_models = {model_name: YOLO(model_path, task="pose", verbose=False) for model_name, model_path in models.items()}

    # Run inference
    for iter in range(5):
        # Get image

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
            else:
                # Filter by box confidence
                # confidence_threshold = 0.6
                # result.keypoints = result.keypoints[result.boxes.conf >= confidence_threshold]
                # result.boxes = result.boxes[result.boxes.conf >= confidence_threshold]

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
                            distance = math.sqrt((tensor[0] - actual_base[0]) ** 2 + (tensor[1] - actual_base[1]) ** 2)
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

    # Calculate and print averages
    for model_name in models:
        print()
        print(f"Results for {model_name}:")
        average_distance = np.mean(results_data[model_name]["distances"])
        average_confidence = np.mean(results_data[model_name]["confidences"])
        average_box_conf = np.mean(results_data[model_name]["box_conf"])
        average_point = np.mean(results_data[model_name]["point"])
        print(f"Average best Euclidean distance for {model_name}: {average_distance:.2f}")
        print(f"Average confidence for {model_name}: {average_confidence:.2f}")
        print(f"Average box confidence for {model_name}: {average_box_conf:.2f}")
        print(f"Average point for {model_name}: {average_point:.2f}")

        results_data[model_name] = [average_distance, average_confidence, average_box_conf, average_point]

    # Save data to CSV
    csv_path = f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/eval/{test_name}/results.csv"
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Model Name", "Average Euclidean Distance (Pixels)", "Average Point Confidence", "Average Box Confidence", "Average Point"])
        for model_name, data in results_data.items():
            writer.writerow([model_name] + data)
    print(f"Results saved to {csv_path}")

def cleanup():
    neo.clear_strip()
    neo.update_strip()

url = "http://localhost:8000"
neo = Pi5Neo('/dev/spidev0.0', 16, 800)
a = 1.0

actual_base = (658, 325)
test_name = "set2/test_"

if __name__ == "__main__":

    # main()

    init_picture()

    cleanup()
