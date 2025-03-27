import cv2
import numpy as np

def detect_keypoints(image, lower_bound, upper_bound):
    """Detects color-coded keypoints in the given image using HSV thresholding."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    keypoints = []
    for cnt in contours:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if radius > 2:  # Filter small detections
            keypoints.append((int(x), int(y)))

    keypoints = np.array(keypoints)
    return keypoints if len(keypoints) > 0 else None

# Load images
img1 = cv2.imread('image1.png')
img2 = cv2.imread('image2.png')
img3 = cv2.imread('image3.png')
img4 = cv2.imread('image4.png')

# Define HSV color bounds for detecting red dots
lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])

# Detect keypoints in both images
kp1 = detect_keypoints(img1, lower_red, upper_red)
kp2 = detect_keypoints(img2, lower_red, upper_red)
kp3 = detect_keypoints(img3, lower_red, upper_red)
kp4 = detect_keypoints(img4, lower_red, upper_red)

def compute_displacement(kp1, kp2):
    """Computes displacement between corresponding keypoints."""
    if kp1 is None or kp2 is None:
        print("Error: Keypoints not detected in one or both images.")
        return None

    if len(kp1) != len(kp2):
        print("Warning: Mismatch in keypoints detected! Matching may be incorrect.")

    # Sort keypoints to maintain consistent order (e.g., left-to-right)
    kp1 = kp1[np.argsort(kp1[:, 0])]
    kp2 = kp2[np.argsort(kp2[:, 0])]

    displacement = kp2 - kp1  # Calculate movement vectors
    return displacement

displacement = compute_displacement(kp1, kp2)

if displacement is not None:
    print("Displacements (x, y) per keypoint:\n", displacement)