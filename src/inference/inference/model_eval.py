from ultralytics import YOLO
from pi5neo import Pi5Neo
import requests
from PIL import Image
from io import BytesIO
import time
import numpy as np
import csv

url = "http://10.0.0.171:8000"
neo = Pi5Neo('/dev/spidev0.0', 16, 800)
a = 1.0

actual_base = (604, 293)
test_name = "test5"

models = {
    "indoor_pose": "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model",
    "indoor_bright": "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_bright_ncnn_model",
    "all_partial": "/mnt/shared/weedy_ros/src/inference/inference/models/all_partial_ncnn_model",
    "outdoor_partial": "/mnt/shared/weedy_ros/src/inference/inference/models/outdoor_partial_ncnn_model",
    "outdoor_then_indoor": "/mnt/shared/weedy_ros/src/inference/inference/models/outdoor_then_indoor_ncnn_model",
}

def get_image():
    try:
        response = requests.get(url)
        response.raise_for_status()

        image = Image.open(BytesIO(response.content))

        image_path = f"/mnt/shared/weedy_ros/src/inference/inference/eval/{test_name}/downloaded_image.jpg"
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

    results_data = {model_name: {"distances": [], "confidences": []} for model_name in models}

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
            else:
                # Filter by box confidence
                confidence_threshold = 0.6
                result.keypoints = result.keypoints[result.boxes.conf >= confidence_threshold]
                result.boxes = result.boxes[result.boxes.conf >= confidence_threshold]

                # Save image
                result.save(f"/mnt/shared/weedy_ros/src/inference/inference/eval/{test_name}/{model_name}_{iter}.jpg")

                # Calculate distances and confidences
                best_distance = 1080
                best_confidence = 0
                for kp in result.keypoints:
                    if not kp.has_visible:
                        continue

                    keypoints = list(zip(["flower", "base", "upper", "lower"], kp.data[0]))
                    keypoints = sorted(keypoints, key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
                    for name, tensor in keypoints:
                        if all(tensor.numpy() != 0):
                            distance = euclidean_distance(actual_base, (tensor[0], tensor[1]))
                            confidence = tensor[2]
                            if best_confidence < confidence:
                                best_distance = distance
                                best_confidence = confidence
                            break
                results_data[model_name]["distances"].append(best_distance)
                results_data[model_name]["confidences"].append(best_confidence)

    # Calculate and print averages
    for model_name in models:
        print()
        print(f"Results for {model_name}:")
        average_distance = np.mean(results_data[model_name]["distances"])
        average_confidence = np.mean(results_data[model_name]["confidences"])
        print(f"Average best Euclidean distance for {model_name}: {average_distance:.2f}")
        print(f"Average confidence for {model_name}: {average_confidence:.2f}")

        results_data[model_name] = [average_distance, average_confidence]

    # Save data to CSV
    csv_path = f"/mnt/shared/weedy_ros/src/inference/inference/eval/{test_name}/results.csv"
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Model Name", "Average Euclidean Distance", "Average Confidence"])
        for model_name, data in results_data.items():
            writer.writerow([model_name] + data)
    print(f"Results saved to {csv_path}")

def cleanup():
    neo.clear_strip()
    neo.update_strip()

if __name__ == "__main__":
    model = YOLO("/mnt/shared/weedy_ros/src/inference/inference/models/all_partial_ncnn_model", task="pose", verbose=False)

    main()
    cleanup()
