from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSourceSubscriber(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        
        self.sub = self.create_subscription(msg_type=Image, topic='image_source', callback=self.read_image, qos_profile=10)
        self.bridge = CvBridge()
        
    def read_image(self, ros_image):
        image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
        if image is not None:
            cv2.imshow("Image", image)
            cv2.waitKey(1)
        else:
            self.get_logger().warn("Image subscriber get image is None")
        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSourceSubscriber("image_subscriber")
    rclpy.spin(image_subscriber)
    cv2.destroyAllWindows()
    image_subscriber.destroy_node()
    rclpy.shutdown()
    