import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotMover(Node):
    """ROS2 Node to send trajectory commands to a robotic arm."""
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def send_movement(self, displacements):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

        for i, displacement in enumerate(displacements):
            point = JointTrajectoryPoint()
            point.positions = displacement.tolist()  # Convert NumPy array to list
            point.time_from_start.sec = i + 1  # Set different time points for execution
            msg.points.append(point)

        self.publisher_.publish(msg)

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

    return np.array(keypoints) if keypoints else None

def compute_displacement(kp_prev, kp_next):
    """Computes displacement between corresponding keypoints."""
    if kp_prev is None or kp_next is None:
        print("Error: Keypoints not detected in one or both images.")
        return None

    if len(kp_prev) != len(kp_next):
        print("Warning: Mismatch in keypoints detected! Matching may be incorrect.")

    # Sort keypoints to maintain consistent order (e.g., left-to-right)
    kp_prev = kp_prev[np.argsort(kp_prev[:, 0])]
    kp_next = kp_next[np.argsort(kp_next[:, 0])]

    displacement = kp_next - kp_prev  # Calculate movement vectors
    return displacement

def main():
    # Load 4 images
    images = [
        cv2.imread('/home/chrisrvt/projects/Implementation_of_Intelligent_Robotics/computer_vision/src/images/image1.png'),
        cv2.imread('/home/chrisrvt/projects/Implementation_of_Intelligent_Robotics/computer_vision/src/images/image2.png'),
        cv2.imread('/home/chrisrvt/projects/Implementation_of_Intelligent_Robotics/computer_vision/src/images/image3.png'),
        cv2.imread('/home/chrisrvt/projects/Implementation_of_Intelligent_Robotics/computer_vision/src/images/image4.png')
    ]

    # Define HSV color bounds for detecting red dots
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])

    # Detect keypoints for all images
    keypoints = [detect_keypoints(img, lower_red, upper_red) for img in images]

    # Compute displacements between consecutive images
    displacements = []
    for i in range(len(keypoints) - 1):
        disp = compute_displacement(keypoints[i], keypoints[i + 1])
        if disp is None:
            return
        displacements.append(disp[:, 1] * 0.01)  # Convert Y-displacement to joint movements

    print("Displacements (x, y) per keypoint:\n", displacements)

    # Send movement commands to ROS 2
    rclpy.init()
    mover = RobotMover()
    mover.send_movement(displacements)
    rclpy.shutdown()

if __name__ == '__main__':
    main()