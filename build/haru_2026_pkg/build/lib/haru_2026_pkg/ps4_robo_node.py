import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import UInt8MultiArray
import glob


class PS4CanNode(Node):
    def __init__(self):
        super().__init__('ps4_node')
        self.get_logger().info('PS4 Node 起動 (読み取りのみ)')

        # pygame 初期化
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error('PS4 コントローラが接続されていません')
            raise SystemExit('No joystick')

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        name = self.joystick.get_name()
        self.get_logger().info(f'使用中のコントローラ: {name}')

        # デッドゾーン
        self.DEADZONE = 0.08

        # タイマーでループ (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # 簡易ボタン名称（表示用、環境により異なる）
        self.button_names = {
            0: 'Square', 1: 'Cross', 2: 'Circle', 3: 'Triangle',
            4: 'L1', 5: 'R1', 6: 'L2', 7: 'R2',
            8: 'Share', 9: 'Options', 10: 'L3', 11: 'R3',
            12: 'PS', 13: 'Touch'
        }

        # ROS publishers: publish axis and button arrays
        self.axes_pub = self.create_publisher(UInt8MultiArray, 'ps4/axes', 10)
        self.buttons_pub = self.create_publisher(UInt8MultiArray, 'ps4/buttons', 10)

    def timer_callback(self):
        # pygame のイベント処理
        pygame.event.pump()

        # 軸値を読み出してデッドゾーン処理、-1..+1 -> 0..255（中立値が128になるよう round を使用）
        axes_bytes = []
        axes_vals = []
        for i in range(self.joystick.get_numaxes()):
            val = self.joystick.get_axis(i)
            if abs(val) < self.DEADZONE:
                val = 0.0
            # Y軸（ユーザ指定: インデックス1と4）を反転させる場合は
            # 値の符号を反転してからマッピングする。こうすると中立(0.0)は
            # 128 のまま保たれる。
            if i in (1, 4):
                val = -val
            # map -1..+1 -> 0..255 with center = 128
            b = int(round((val + 1.0) * 127.5))
            if b < 0:
                b = 0
            if b > 255:
                b = 255
            axes_vals.append(round(val, 3))
            axes_bytes.append(b)

        # 通常ボタン
        buttons = [i for i in range(self.joystick.get_numbuttons()) if self.joystick.get_button(i)]

        # ハット(D-pad)
        hat_buttons = []
        for h in range(self.joystick.get_numhats()):
            hx, hy = self.joystick.get_hat(h)
            if hy == 1:
                hat_buttons.append(14)  # Up
            if hy == -1:
                hat_buttons.append(15)  # Down
            if hx == -1:
                hat_buttons.append(16)  # Left
            if hx == 1:
                hat_buttons.append(17)  # Right

        all_buttons = sorted(set(buttons + hat_buttons))

        # 端末表示
        axis_str = ' '.join([f'{v:03d}' for v in axes_bytes])

        max_btn_index = max(self.joystick.get_numbuttons(), 18)
        btn_state = [0] * max_btn_index
        for b in buttons:
            if b < max_btn_index:
                btn_state[b] = 1
        for hb in hat_buttons:
            if hb < max_btn_index:
                btn_state[hb] = 1

        btn_row = ''.join(str(x) for x in btn_state)
        btn_names = [self.button_names.get(b, str(b)) for b in all_buttons]
        btn_label = ','.join(btn_names) if btn_names else 'None'

        # シンプルにログ出力
        self.get_logger().info(f'[PS4] AXES(0..255): {axis_str}  BUTTONS: {btn_row}  ({btn_label})')

        # Publish axis and button arrays as UInt8MultiArray
        try:
            axes_msg = UInt8MultiArray()
            axes_msg.data = axes_bytes
            self.axes_pub.publish(axes_msg)
        except Exception as e:
            self.get_logger().error(f'axes publish error: {e}')

        try:
            btn_msg = UInt8MultiArray()
            btn_msg.data = btn_state
            self.buttons_pub.publish(btn_msg)
        except Exception as e:
            self.get_logger().error(f'buttons publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PS4CanNode()
    except SystemExit:
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('終了します')
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
