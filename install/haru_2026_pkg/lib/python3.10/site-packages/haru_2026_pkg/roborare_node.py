import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class RobowereNode(Node):
    def __init__(self):
        super().__init__('robowere_node')
        self.get_logger().info("Robowere Node 起動")

        # パブリッシャ
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ps3', 10)

        # サブスクライバ
        self.create_subscription(
            Float32MultiArray,
            'ps3_axes',
            self.axis_callback,
            10
        )

    def axis_callback(self, msg: Float32MultiArray):
        # 受信順: [左X, 左Y, 右X, 右Y]
        axes = msg.data
        if len(axes) < 4:
            self.get_logger().warn("軸データが不足しています")
            return

        left_x = axes[0]   # 左スティック左右
        left_y = axes[1]   # 左スティック上下
        right_x = axes[2]  # 右スティック左右

        twist_msg = Twist()
        twist_msg.linear.x = left_x    # Vx
        twist_msg.linear.y = left_y    # Vy
        twist_msg.angular.z = right_x  # Omega

        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobowereNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("終了します")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()