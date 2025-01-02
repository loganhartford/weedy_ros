import cv2

def get_pixel_coordinates(image_path):
    points = []

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
            print(f"Clicked at: ({x}, {y})")
            points.append((x, y))

    # Load and display the image
    image = cv2.imread(image_path)
    cv2.imshow("Image", image)
    cv2.setMouseCallback("Image", click_event)

    # Wait until any key is pressed
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return points

# Example usage
image_path = "downloaded_image.jpg"
selected_points = get_pixel_coordinates(image_path)
print(f"Selected points: {selected_points}")
