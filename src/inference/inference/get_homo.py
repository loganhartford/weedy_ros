import cv2
import numpy as np

# Pixel points (image space)
pixel_points = np.array([[265, 443], [258, 874], [1633, 869], [1625, 440]], dtype=np.float32)

# Ground points (robot frame, assume z=0)
ground_points = np.array([[294, 101.4], [294, 0], [-26, 0], [-26, 101.4]], dtype=np.float32)

# Compute homography
H, _ = cv2.findHomography(pixel_points, ground_points)

# Map a new pixel to robot frame
pixel = np.array([265, 443, 1], dtype=np.float32)
ground_point = np.dot(H, pixel)
ground_point /= ground_point[2]  # Normalize
x_robot, y_robot = ground_point[0], ground_point[1]

print(f"Ground coordinates in robot frame: ({x_robot}, {y_robot})")
