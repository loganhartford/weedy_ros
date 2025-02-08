#! /mnt/shared/weedy_ros/src/decisions/decisions/venv/bin/python3

from ultralytics import YOLO
import numpy as np
import csv
import cv2
import os



models = {
    "indoor_pose": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_pose_ncnn_model",
    "indoor_angled": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_angled_ncnn_model",
    "indoor_all": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_all_ncnn_model",
    "indoor_indoor": "/mnt/shared/weedy_ros/src/decisions/decisions/models/indoor_indoor_ncnn_model",
}

def main():
    print()
    print("################# TEST #################")
    print()

    results_data = {model_name: { "box_conf": [], "num_boxes": []} for model_name in models}

    # Load models
    loaded_models = {model_name: YOLO(model_path, task="pose", verbose=False) for model_name, model_path in models.items()}

    # Run inference
    img_folder = f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/moving/{test_name}/img"
    for i, image_path in enumerate(os.listdir(img_folder)):
        for model_name, model in loaded_models.items():
            # Run inference
            results = model(f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/moving/{test_name}/img/{image_path}", verbose=False)
            result = results[0]

            # If no results, add placeholder values
            if len(result) == 0:
                results_data[model_name]["box_conf"].append(0)
                results_data[model_name]["num_boxes"].append(0)
            else:
                # Save image
                result.save(f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/moving/{test_name}/{model_name}_{i}.jpg")

                best_box_conf = max(result.boxes.conf)
                num_boxes = len(result.boxes)

                results_data[model_name]["box_conf"].append(best_box_conf)
                results_data[model_name]["num_boxes"].append(num_boxes)

    # Calculate and print averages
    for model_name in models:
        print()
        print(f"Results for {model_name}:")
        average_box_conf = np.mean(results_data[model_name]["box_conf"])
        average_num_boxes = np.mean(results_data[model_name]["num_boxes"])
        print(f"Average box confidence for {model_name}: {average_box_conf:.2f}")
        print(f"Average number of boxes for {model_name}: {average_num_boxes:.2f}")

        results_data[model_name] = [ average_box_conf,  average_num_boxes]

    # Save data to CSV
    csv_path = f"/mnt/shared/weedy_ros/src/decisions/decisions/test_scripts/moving/{test_name}/results.csv"
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Model Name", "Average Box Confidence", "Average Number of Boxes"])
        for model_name, data in results_data.items():
            writer.writerow([model_name] + data)
    print(f"Results saved to {csv_path}")


url = "http://localhost:8000"
test_name = "set2"

if __name__ == "__main__":

    main()



