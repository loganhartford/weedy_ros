from ultralytics import YOLO


model_path = "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model"
img_path = "/mnt/shared/weedy_ros/downloaded_image.jpg"

if __name__ == '__main__':
    ncnn_model = YOLO(model_path, task="pose")
    results = ncnn_model(img_path)

    if not results:
        print("No detections found.")
    else:
        result = results[0]
        #results.show()
        
        print(result.boxes.conf)
        # print(result.keypoints)

        confidences = result.boxes.conf  # Assuming result.boxes.conf is a tensor

        # Create a mask for boxes with confidence >= 0.6
        mask = result.boxes.conf >= 0.6

        # Filter keypoints and boxes using the mask
        

        
        print(len(result.keypoints))
        # good_kps = []
        # good_boxes = []
        # for conf in result.boxes.conf:
        #     if conf > 0.6:



        result.save("./result.jpg")

        # for box in result.boxes:
        #     xyxy = box.xyxy[0].cpu().numpy()  # Get [xmin, ymin, xmax, ymax] format
        #     conf = box.conf
        #     cls = box.cls
        #     print(f"Bounding box coordinates: {xyxy}, Confidence: {conf}, Class: {cls}")