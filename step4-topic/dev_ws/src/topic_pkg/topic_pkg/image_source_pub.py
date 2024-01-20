from pathlib import Path
from copy import deepcopy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSourcePublisher(Node):
    
    def __init__(self, node_name, image_file: str = None):
        super().__init__(node_name)
        
        self.pub = self.create_publisher(msg_type=Image, topic='image_source', qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=0.5, callback=self.publish_image)

        self.bridge = CvBridge()
        self.image_path = Path(image_file)
        if self.image_path.exists():
            self.image = cv2.imread(self.image_path.absolute().as_posix())
        else:
            self.get_logger().error(f"Image file does not exist: {self.image_path.absolute().as_posix()}")
            assert False
        self.index = 0
        
    def publish_image(self):
        frame = cv2.putText(deepcopy(self.image), f"{self.index}", (40, 200), cv2.FONT_HERSHEY_PLAIN, 12, (0, 255, 0), 2)
        self.index += 1
        ros2_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(ros2_img)
        self.get_logger().info(f'Image publish: {self.index = }')
        

def main(args=None):
    rclpy.init(args=args)
    image_file = '/home/ubuntu/workspace/ros2/step4-topic/dev_ws/src/topic_pkg/resource/apple.jpg'
    image_source_publisher = ImageSourcePublisher("image_source_publisher", image_file=image_file)
    rclpy.spin(image_source_publisher)
    image_source_publisher.destroy_node()
    rclpy.shutdown()