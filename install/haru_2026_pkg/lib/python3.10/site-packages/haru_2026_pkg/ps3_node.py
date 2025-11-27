import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import pygame
import time

class PS3Node(Node):
    def __init__(self):
        super().__init__('ps3_node')
        self.get_logger().info("PS3 Controller Node 起動（デバッグモード）")

        # ROS2 パブリッシャ
        self.axis_pub = self.create_publisher(Float32MultiArray, 'ps3_axes', 10)
        self.button_pub = self.create_publisher(Int32MultiArray, 'ps3_buttons', 10)

        # pygame 初期化
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("コントローラーが接続されていません")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"使用中のコントローラー: {self.joystick.get_name()}")

        # デッドゾーン
        self.DEADZONE = 0.08

        # タイマーでループ（10Hz）
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()

        # 軸値
        axes = []
        axes_names = ['左X', '左Y', '右X', '右Y']
        for i in range(self.joystick.get_numaxes()):
            if i in [2, 5]:
                continue
            val = self.joystick.get_axis(i)
            # 左スティック上下と右スティック上下は反転
            if i == 1 or i == 4:
                val = -val
            if abs(val) < self.DEADZONE:
                val = 0.0
            axes.append(round(val, 2))

        # デバッグ表示
        axis_log = " | ".join([f"{name}:{v:+.2f}" for name, v in zip(axes_names, axes)])
        self.get_logger().info(f"[AXES] {axis_log}")

        # パブリッシュ
        self.axis_pub.publish(Float32MultiArray(data=axes))

        # ボタン値
        buttons = [i for i in range(self.joystick.get_numbuttons()) if self.joystick.get_button(i)]
        self.get_logger().info(f"[BUTTONS] {buttons}")

        # パブリッシュ
        self.button_pub.publish(Int32MultiArray(data=buttons))


def main(args=None):
    rclpy.init(args=args)
    node = PS3Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("終了します")
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()