#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import can
import glob


class PS4CanNode(Node):
    """PS4 を USB で読み、スティックとボタンを端末表示するだけの簡素なノード。

    注意: 現在はCAN処理やトピック配信は行いません。将来 CAN を追加する場合は
    別途実装してください。
    """

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

        # CAN 初期化: socketcan can0 を優先、なければシリアルデバイスを探す
        self.bus = None
        try:
            self.bus = can.Bus(interface='socketcan', channel='can0')
            self.get_logger().info("SocketCAN 'can0' に接続しました")
        except Exception:
            # シンプルにデバイス名候補を探す
            devs = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            if devs:
                dev = devs[0]
                try:
                    self.bus = can.Bus(interface='serial', channel=dev, baudrate=115200)
                    self.get_logger().info(f'シリアルCANデバイスに接続: {dev}')
                except Exception as e:
                    self.get_logger().warn(f'シリアルCAN接続失敗: {e} (CAN送信は無効)')
                    self.bus = None
            else:
                self.get_logger().info('CANデバイスが見つかりません。CAN送信は無効です')

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

        # シンプルにログ出力（必要ならここをファイル出力やトピックに差し替え）
        self.get_logger().info(f'[PS4] AXES(0..255): {axis_str}  BUTTONS: {btn_row}  ({btn_label})')

        # --- CAN 送信 ---
        # 仕様（仮定）:
        # - ボタン8個分をそのままの0/1で送るメッセージを2つ (ID: 0x100, 0x101)
        #   各メッセージは8バイト (不足分は0埋め)
        # - アナログ値6個分を0x00-0xFFで送るメッセージを1つ (ID: 0x160)
        if self.bus:
            try:
                # ボタン配列 0-7 と 8-15（存在しなければ0で埋める）
                btn0_7 = [0] * 8
                btn8_15 = [0] * 8
                for i in range(8):
                    if i < len(btn_state):
                        btn0_7[i] = 1 if btn_state[i] else 0
                for i in range(8, 16):
                    if i < len(btn_state):
                        btn8_15[i - 8] = 1 if btn_state[i] else 0

                # アナログ 6 バイト（axes_bytes の先頭6要素を使用、足りなければ0で埋める）
                analog6 = [0] * 6
                for i in range(6):
                    if i < len(axes_bytes):
                        analog6[i] = axes_bytes[i]

                # 送信: create messages and send
                msg1 = can.Message(arbitration_id=0x100, data=bytes(btn0_7), is_extended_id=False)
                self.bus.send(msg1)
                self.get_logger().info(f'SENT CAN 0x100 buttons0-7: {btn0_7}')

                # ユーザ指定: ID0x160 のアナログ配列の前に L3(11) と R3(12) を
                # 01 の状態で追加して送信する。
                l3 = btn_state[11] if 11 < len(btn_state) else 0
                r3 = btn_state[12] if 12 < len(btn_state) else 0

                # ID0x101 は 11(L3) と 12(R3) を除外し、D-pad の Left/Right を
                # 末尾に含める形式にする（8バイト）。
                # 仮定: 配列は [8,9,10,13,14,15, DpadLeft, DpadRight]
                def bs(i):
                    return btn_state[i] if i < len(btn_state) else 0

                dpad_left = btn_state[16] if 16 < len(btn_state) else 0
                dpad_right = btn_state[17] if 17 < len(btn_state) else 0

                btn8_15_mod = [
                    bs(8), bs(9), bs(10),
                    bs(13), bs(14), bs(15),
                    dpad_left, dpad_right
                ]

                msg2 = can.Message(arbitration_id=0x101, data=bytes(btn8_15_mod), is_extended_id=False)
                self.bus.send(msg2)
                self.get_logger().info(f'SENT CAN 0x101 buttons8-15(mod): {btn8_15_mod}')

                # ID0x160: [L3, R3, analog0..5]
                data160 = [l3 & 0xFF, r3 & 0xFF] + [a & 0xFF for a in analog6]
                msg3 = can.Message(arbitration_id=0x160, data=bytes(data160), is_extended_id=False)
                self.bus.send(msg3)
                self.get_logger().info(f'SENT CAN 0x160 [L3,R3,analog6]: {data160}')
            except Exception as e:
                self.get_logger().error(f'CAN送信エラー: {e}')


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
