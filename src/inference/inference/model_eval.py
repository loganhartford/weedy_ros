from ultralytics import YOLO
from pi5neo import Pi5Neo
import requests
from PIL import Image
from io import BytesIO
import time

url = "http://10.0.0.171:8000"
neo = Pi5Neo('/dev/spidev0.0', 16, 800)
a = 1.0


models = {
    "indoor_pose": "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model",
    "indoor_bright": "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_bright_ncnn_model",
    "all_partial": "/mnt/shared/weedy_ros/src/inference/inference/models/all_partial_ncnn_model",
    "outdoor_partial": "/mnt/shared/weedy_ros/src/inference/inference/models/outdoor_partial_ncnn_model",
}


def get_image():
    try:
        response = requests.get(url)
        response.raise_for_status()

        image = Image.open(BytesIO(response.content))

        image.save("downloaded_image.jpg")

        print("Image fetched")
        return image

    except requests.exceptions.RequestException as e:
        print("Failed to fetch image:", e)


def main():
    print()
    print("################# TEST #################")
    print()
    # Turn on light
    neo.fill_strip(int(255 * a), int(255 * a), int(255 * a))
    neo.update_strip()

    time.sleep(3)

    # Get image
    image = get_image()

    # Run inference
    for model_name in models:
        print()
        model_path = models[model_name]
        model = YOLO(model_path, task="pose", verbose=False)
        print(f"Model: {model_name}")
        
        results = model("downloaded_image.jpg", verbose=False)
        result = results[0]
        if len(result) == 0:
            print("No results")
        else:
            # Filter by box confidence
            confidence_threshold = 0.6
            result.keypoints = result.keypoints[result.boxes.conf >= confidence_threshold]
            result.boxes = result.boxes[result.boxes.conf >= confidence_threshold]

            # Save image
            result.save(f"/mnt/shared/weedy_ros/src/inference/inference/eval/{model_name}.jpg")

            # Print keypoint locations
            for i, kp in enumerate(result.keypoints):
                # print(f"Keypoint set {i}")
                if not kp.has_visible:
                    continue
            
                keypoints = list(zip(["flower", "base", "upper", "lower"], kp.data[0]))
                keypoints = sorted(keypoints, key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
                for name, tensor in keypoints:
                    if all(tensor.numpy() != 0):
                        print(f"{name}: ({tensor[0]:.2f}, {tensor[1]:.2f}, {tensor[2]:.2f})")
                        break


def cleanup():
    neo.clear_strip()
    neo.update_strip()

if __name__ == "__main__":
    main()
    cleanup()

