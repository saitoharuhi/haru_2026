import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import pygame
import time


class PS3Node(Node):
    def __init__(self):
        super().__init__('ps3_node')
        self.get_logger().info("PS4 Controller Node 起動（L2/R2・十字キー対応）")  # ★ 変更（PS4対応表記）

        # ROS2 パブリッシャ
        self.axis_pub = self.create_publisher(Float32MultiArray, 'ps3_axes', 10)     # ★ 変更（トピック名）
        self.button_pub = self.create_publisher(Int32MultiArray, 'ps3_buttons', 10) # ★ 変更（トピック名）

        # pygame 初期化
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("コントローラーが接続されていません")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"使用中のコントローラー: {self.joystick.get_name()}")

        self.DEADZONE = 0.08

        # ★ 追加：ボタン番号 → 名前（PS4）
        self.button_map = {
            0: "×", 1: "○", 2: "□", 3: "△",
            4: "L1", 5: "R1",
            6: "L2", 7: "R2",
            8: "SHARE", 9: "OPTIONS",
            10: "L3", 11: "R3",
            12: "PS"
        }

        # ★ 追加：十字キー用ID
        self.hat_map = {100: "↑", 101: "↓", 102: "←", 103: "→"}

        # タイマーでループ（100Hz）
        self.timer = self.create_timer(0.01, self.timer_callback)

    # ★ 追加：L2/R2 正規化関数
    def normalize_trigger(self, val):
        return round((val + 1.0) / 2.0, 2)

    def timer_callback(self):
        pygame.event.pump()

        # =====================
        # 軸（スティック）
        # =====================
        axes = []
        axes_names = ['LX', 'LY', 'RX', 'RY']  # ★ 変更（PS4用に整理）

        axis_index = [0, 1, 3, 4]  # ★ 変更（PS4の軸配置）

        for name, i in zip(axes_names, axis_index):
            val = self.joystick.get_axis(i)

            # ★ 変更：Y軸反転
            if name in ['LY', 'RY']:
                val = -val

            if abs(val) < self.DEADZONE:
                val = 0.0

            axes.append(round(val, 2))

        # =====================
        # ★ 追加：L2 / R2 アナログ
        # =====================
        l2_val = self.normalize_trigger(self.joystick.get_axis(2))
        r2_val = self.normalize_trigger(self.joystick.get_axis(5))

        axes.extend([l2_val, r2_val])  # ★ 追加

        self.axis_pub.publish(Float32MultiArray(data=axes))

        # =====================
        # ボタン
        # =====================
        pressed_buttons = []
        pressed_names = []  # ★ 追加（表示用）

        # 通常ボタン
        for i in range(self.joystick.get_numbuttons()):
            if self.joystick.get_button(i):
                pressed_buttons.append(i)
                pressed_names.append(self.button_map.get(i, f"BTN{i}"))  # ★ 追加

        # =====================
        # ★ 追加：十字キー（HAT）
        # =====================
        if self.joystick.get_numhats() > 0:
            hat_x, hat_y = self.joystick.get_hat(0)

            if hat_y == 1:
                pressed_buttons.append(100)
                pressed_names.append("↑")
            elif hat_y == -1:
                pressed_buttons.append(101)
                pressed_names.append("↓")

            if hat_x == -1:
                pressed_buttons.append(102)
                pressed_names.append("←")
            elif hat_x == 1:
                pressed_buttons.append(103)
                pressed_names.append("→")

        self.button_pub.publish(Int32MultiArray(data=pressed_buttons))

        # =====================
        # ★ 追加：デバッグ表示
        # =====================
        self.get_logger().info(f"[AXES] {axes}")

        if pressed_names:
            self.get_logger().info(f"[BUTTONS] 押下中: {', '.join(pressed_names)}")
        else:
            self.get_logger().info("[BUTTONS] 押下なし")


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