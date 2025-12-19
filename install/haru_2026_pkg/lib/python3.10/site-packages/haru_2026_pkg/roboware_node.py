#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray, UInt8MultiArray

class RobowereNode(Node):
    def __init__(self):
        super().__init__('robowere_node')
        self.get_logger().info("Robowere Node 起動（デバッグ・単位変換版）")

        # パブリッシャ
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ps3', 10)
        # ボタン→CAN送信用トピック
        self.btn_pub = self.create_publisher(UInt8MultiArray, 'cmd_buttons', 10)

        # サブスクライバ
        self.create_subscription(
            Float32MultiArray,
            'ps3_axes',
            self.axis_callback,
            10
        )

        # ボタン購読して CAN 送信用トピックへ変換
        self.create_subscription(
            Int32MultiArray,
            'ps3_buttons',
            self.button_callback,
            10
        )

        # 最大値設定
        self.MAX_LINEAR = 500.0    # mm/s
        self.MAX_ANGULAR = 45.0    # deg/s

    def axis_callback(self, msg: Float32MultiArray):
        # 受信順: [左X, 左Y, 右X, 右Y]
        axes = msg.data
        if len(axes) < 4:
            self.get_logger().warn("軸データが不足しています")
            return

        left_x = axes[0]   # 左スティック左右
        left_y = axes[1]   # 左スティック上下
        right_x = axes[2]  # 右スティック左右

        # 単位変換
        vx_mm_s = left_x * self.MAX_LINEAR
        vy_mm_s = left_y * self.MAX_LINEAR
        omega_deg_s = right_x * self.MAX_ANGULAR

        # Twist メッセージ作成
        twist_msg = Twist()
        twist_msg.linear.x = vx_mm_s
        twist_msg.linear.y = vy_mm_s
        twist_msg.angular.z = omega_deg_s

        # 配信
        self.cmd_pub.publish(twist_msg)

        # デバッグ表示
        self.get_logger().info(
            f"[DEBUG] Vx: {vx_mm_s:+.1f} mm/s | Vy: {vy_mm_s:+.1f} mm/s | Omega: {omega_deg_s:+.1f} deg/s"
        )

    def button_callback(self, msg: Int32MultiArray):
        # msg.data は押されているボタンのインデックスリスト
        indices = list(msg.data)

        # PS4 の hat コードやボタン番号を can_node が期待する 0..17 のインデックスへマップする
        mapped = []
        for raw in indices:
            try:
                r = int(raw)
            except Exception:
                continue

            # HAT 特殊コード -> D-pad indices
            if r == 100:
                mapped.append(14)  # Up
                continue
            if r == 101:
                mapped.append(15)  # Down
                continue
            if r == 102:
                mapped.append(16)  # Left
                continue
            if r == 103:
                mapped.append(17)  # Right
                continue

            # Face button remapping (PS4 joystick button numbering -> can_node が期待する indices)
            # PS4 raw: 0:×, 1:○, 2:□, 3:△
            # can_node expects: 0->□, 1->×, 2->○, 3->△
            if r == 0:
                mapped.append(1)  # × -> index 1
                continue
            if r == 1:
                mapped.append(2)  # ○ -> index 2
                continue
            if r == 2:
                mapped.append(0)  # □ -> index 0
                continue
            if r == 3:
                mapped.append(3)  # △ -> index 3
                continue

            # その他のボタンはそのまま使う（例: L1=4,R1=5,L2=6,R2=7,L3=10,R3=11,SHARE=8,OPTIONS=9,PS=12）
            if 0 <= r <= 17:
                mapped.append(r)

        # CAN 1フレームは最大8バイト。ここでは 1バイト目に count、残りにインデックスを入れる方式とする
        max_indices = 7
        count = min(len(mapped), max_indices)
        data_bytes = [count]
        for i in range(count):
            idx = int(mapped[i]) & 0xFF
            data_bytes.append(idx)
        # pad to at most 8 bytes is not necessary for UInt8MultiArray, but CAN send will trim
        uba = UInt8MultiArray()
        uba.data = data_bytes
        self.btn_pub.publish(uba)
        self.get_logger().info(f"ボタン送信準備: count={count} indices={data_bytes[1:]} (mapped from {indices})")

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