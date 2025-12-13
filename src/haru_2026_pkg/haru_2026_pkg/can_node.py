import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
from geometry_msgs.msg import Twist
import can, struct

class CANNode(Node):
    def __init__(self):
        super().__init__('can_node')
        self.get_logger().info("CAN Node 起動")

        self.bus = None
        try:
            self.bus = can.Bus(interface='socketcan', channel='can0')
        except OSError as e:
            self.get_logger().error(f"SocketCANにアクセスできません: {e}")
            return

        # 購読: roboware から速度指令を受け取る
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_ps3',
            self.send_can_message_callback,
            10
        )

        # 購読: ボタンからのCAN送信用トピック
        self.btn_subscription = self.create_subscription(
            UInt8MultiArray,
            'cmd_buttons',
            self.send_button_can_callback,
            10
        )

        # 位置情報パブリッシュ
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)  # 10ms周期

    def send_can_message_callback(self, msg: Twist):
        vx = msg.linear.x   # mm/s
        vy = msg.linear.y
        omega = msg.angular.z  # deg/s

        # 固定小数点変換（例：×10）
        scale = 10
        try:
            data_160 = struct.pack('>hhh', int(vx * scale), int(vy * scale), int(omega * scale))
            can_msg = can.Message(arbitration_id=0x160, data=data_160, is_extended_id=False)
            self.bus.send(can_msg)
            self.get_logger().debug(f"送信[0x160] vx:{vx:.1f} vy:{vy:.1f} ω:{omega:.1f}")
        except can.CanError as e:
            self.get_logger().error(f"CAN送信失敗: {e}")
        except struct.error as e:
            self.get_logger().error(f"データ変換エラー: {e}")

    def send_button_can_callback(self, msg: UInt8MultiArray):
        # msg.data は 0~255 のバイトリストを期待（先頭は count）
        data_list = list(msg.data)
        if not data_list:
            self.get_logger().warn("cmd_buttons が空です")
            return
        # Trim or pad to 8 bytes
        if len(data_list) > 8:
            self.get_logger().warn(f"cmd_buttons 長すぎるので先頭8バイトに切り詰めます (len={len(data_list)})")
            data_list = data_list[:8]
        # Ensure each is 0-255
        try:
            data_bytes = bytes([int(x) & 0xFF for x in data_list])
        except Exception as e:
            self.get_logger().error(f"cmd_buttons のバイト変換に失敗: {e}")
            return

        try:
            can_msg = can.Message(arbitration_id=0x100, data=data_bytes, is_extended_id=False)
            self.bus.send(can_msg)
            hexstr = ' '.join(f"{b:02X}" for b in data_bytes)
            self.get_logger().info(f"送信[0x100] data: {hexstr}")
        except can.CanError as e:
            self.get_logger().error(f"CAN送信失敗 (0x100): {e}")

    def timer_callback(self):
        if not self.bus:
            return
        msg = self.bus.recv(timeout=0.001)
        if msg and msg.arbitration_id == 0x150 and len(msg.data) >= 6:
            try:
                x = struct.unpack('>h', msg.data[0:2])[0]
                y = struct.unpack('>h', msg.data[2:4])[0]
                theta = struct.unpack('>h', msg.data[4:6])[0]
                scale = 10.0
                arr = Float32MultiArray()
                arr.data = [x / scale, y / scale, theta / scale]
                self.publisher_.publish(arr)
            except struct.error as e:
                self.get_logger().error(f"受信データ解析エラー: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CANNode()
    if node.bus:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()