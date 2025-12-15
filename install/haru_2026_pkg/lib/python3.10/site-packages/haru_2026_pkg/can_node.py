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
        # msg.data は多様な形式で来る可能性があるため柔軟に扱う
        # 1) 完全なボタン状態配列 (長さ >= 18): 各インデックスが 0/1
        # 2) count + indices 形式 (roboware_node の出力): [count, idx1, idx2, ...]
        # 3) 8バイトの既成ブロック (btn0-7 等)

        data_list = list(msg.data)
        if not data_list:
            self.get_logger().warn("cmd_buttons が空です")
            return

        # Reconstruct full btn_state (indices 0..17 -> up to D-pad)
        btn_state = [0] * 18

        # Case A: full state
        if len(data_list) >= 18 and all((int(x) in (0, 1) for x in data_list[:18])):
            for i in range(18):
                btn_state[i] = 1 if int(data_list[i]) else 0

        # Case B: count + indices (first element small count)
        elif 1 <= len(data_list) <= 8 and int(data_list[0]) <= 7 and len(data_list) == (1 + int(data_list[0])):
            count = int(data_list[0])
            for i in range(count):
                idx = int(data_list[1 + i])
                if 0 <= idx < len(btn_state):
                    btn_state[idx] = 1

        # Case C: 8-byte block -> assume btn0-7
        elif len(data_list) == 8 and all(0 <= int(x) <= 1 for x in data_list):
            for i in range(8):
                btn_state[i] = 1 if int(data_list[i]) else 0

        else:
            # その他: try to interpret any small list as indices
            for v in data_list:
                try:
                    idx = int(v)
                    if 0 <= idx < len(btn_state):
                        btn_state[idx] = 1
                except Exception:
                    continue

        # Build CAN payloads according to user spec
        try:
            # 0x100: ○(2), △(3), ×(1), □(0), Dpad Up(14), Down(15), Left(16), Right(17)
            can100 = [
                btn_state[2], btn_state[3], btn_state[1], btn_state[0],
                btn_state[14], btn_state[15], btn_state[16], btn_state[17]
            ]

            # 0x101: R1(5), R2(7), R3(11), L1(4), L2(6), L3(10), pad, pad
            can101 = [
                btn_state[5], btn_state[7], btn_state[11],
                btn_state[4], btn_state[6], btn_state[10],
                0, 0
            ]

            # 0x102: PS(12), SHARE(8), OPTIONS(9), pad x5
            can102 = [
                btn_state[12], btn_state[8], btn_state[9], 0, 0, 0, 0, 0
            ]

            # Send messages
            m100 = can.Message(arbitration_id=0x100, data=bytes(can100), is_extended_id=False)
            self.bus.send(m100)
            self.get_logger().info(f"送信[0x100] {can100}")

            m101 = can.Message(arbitration_id=0x101, data=bytes(can101), is_extended_id=False)
            self.bus.send(m101)
            self.get_logger().info(f"送信[0x101] {can101}")

            m102 = can.Message(arbitration_id=0x102, data=bytes(can102), is_extended_id=False)
            self.bus.send(m102)
            self.get_logger().info(f"送信[0x102] {can102}")

        except can.CanError as e:
            self.get_logger().error(f"CAN送信失敗: {e}")

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