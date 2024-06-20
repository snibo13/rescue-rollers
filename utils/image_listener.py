import rclpy
from rclpy.node import Node

# from std_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import numpy as np
from PIL import Image


import cv2 as cv


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            CompressedImage, "image_publisher", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        image_np = cv.imdecode(np.array(msg.data, np.uint8), cv.IMREAD_COLOR)
        cv.imshow("Rec", image_np)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
