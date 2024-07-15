import rclpy
from rclpy.node import Node
import numpy as np


from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

w_b = 90 / 1000  # 90 mm
r = 50 / 1000  # 50 mm

rpm_max = 100
v_max = 2 * np.pi * r * rpm_max / 60.0

PWM_SCALING = 1024 / v_max

publisher_l = None
publisher_r = None


def listener_callback(msg):
    v = msg.linear.x
    omega = msg.angular.z
    v_r = v + (omega * w_b) / 2.0
    v_l = v - (omega * w_b) / 2.0
    print(v_r)
    print(v_l)
    pwm_l = Int32()
    pwm_r = Int32()
    print("A")
    pwm_l.data = int(np.clip(0, 512, round(v_l * PWM_SCALING)))
    print("B")
    pwm_r.data = int(np.clip(0, 512, round(v_r * PWM_SCALING)))
    print("C")
    print(pwm_r.data)
    publisher_l.publish(pwm_l)
    publisher_r.publish(pwm_r)


def main(args=None):
    rclpy.init(args=args)

    translator_node = Node("teleop_translator")

    # minimal_subscriber = MinimalSubscriber()
    subscription = translator_node.create_subscription(
        Twist, "/cmd_vel", listener_callback, 10
    )
    global publisher_l
    publisher_l = translator_node.create_publisher(Int32, "wheel_speed", 10)
    global publisher_r
    publisher_r = translator_node.create_publisher(Int32, "r_wheel_speed", 10)

    rclpy.spin(translator_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    translator_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
