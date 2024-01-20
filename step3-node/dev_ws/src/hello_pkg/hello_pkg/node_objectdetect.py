import rclpy
from rclpy.node import Node
import cv2
import numpy as np

lower_red = np.array([0, 90, 128])
upper_red = np.array([100, 255, 255])


def object_detect(node, image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for idx, contour in enumerate(contours):
        if contour.shape[0] < 150:
            continue
        node.get_logger().info(f"contour, {idx = }, {contour.shape = }")
        x, y, w, h = cv2.boundingRect(contour)
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
    
    cv2.imshow("Object Detection", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    

def main(args=None):
    rclpy.init(args=args)
    node = Node("object_detection_node")
    image = cv2.imread('/home/ubuntu/workspace/ros2/step3/dev_ws/src/hello_pkg/resource/apple.jpg')
    object_detect(node, image)
    node.destroy_node()
    rclpy.shutdown()
