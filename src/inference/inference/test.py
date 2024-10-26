from ultralytics import YOLO


model_path = "/mnt/shared/weedy_ros/src/inference/inference/models/coco_ncnn_model"
img_path = "/mnt/shared/weedy_ros/src/inference/inference/latest_image.jpg"

if __name__ == '__main__':
    ncnn_model = YOLO(model_path, task="detect")
    results = ncnn_model(img_path)

    if not results:
        print("No detections found.")
    else:
        result = results[0]
        #results.show()
        result.save()

        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()  # Get [xmin, ymin, xmax, ymax] format
            conf = box.conf
            cls = box.cls
            print(f"Bounding box coordinates: {xyxy}, Confidence: {conf}, Class: {cls}")