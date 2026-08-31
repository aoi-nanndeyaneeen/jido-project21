"""
core/communication.py
ground_receiver (main_pc.cpp) との USB シリアル通信。

--------------------------------------------------------------------------
プロトコル
--------------------------------------------------------------------------
受信 (ground_receiver → PC):
    "DATA,<高度>,<ax>,<ay>,<az>"        既存の構造化行
    "Alt : <高度> m"                     既存のダッシュボード表示行
    "Acc : ax=.. ay=.. az=.."            既存のダッシュボード表示行
    "Att : Roll=.. Pitch=.. Yaw=.."      姿勢角。ヨー推定の旋回ゲートに使用
    "DV,<t_ms>,<dvx_body>,<dvy_body>"    ★新規。ヨー推定用の機体Δv

送信 (PC → ground_receiver):
    "AP,<roll>,<pitch>,<yaw>,<throttle>" 既存のオートパイロット指令
    "YAW,<yaw_deg>,<valid>"              ★新規。カメラ由来の絶対ヨー

★新規の2行は ground_receiver / flight_controller 側の実装が必要。
  仕様は YAW_HANDOFF.md §5 を参照。未実装でも本モジュールは正常に動く
  （DV行が来なければヨー推定が収束しないだけ）。

DV行の座標系: FRD の (前, 右) [m/s]。IMU は FLU を返すので機体側で変換すること。
"""

import serial
import threading
import time
import re
from collections import deque

class SerialReceiver:
    def __init__(self, port="COM7", baudrate=115200):
        self.port     = port
        self.baudrate = baudrate
        self.latest_altitude = 0.0
        self.latest_accel    = (0.0, 0.0, 0.0)
        self.latest_body_yaw = None      # 機体のジャイロ積分ヨー [deg]（相対値）
        self._accel_received = False
        self._dv_received    = False
        self.is_running      = True

        # 受信スレッドが積み、メインループが drain する。
        # ヨー推定器を単一スレッドに保つためのキュー。
        self._dv_queue = deque(maxlen=200)
        self._dv_lock  = threading.Lock()

        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"[SerialReceiver] {port} でRP2040と接続しました。")
        except Exception as e:
            print(f"[SerialReceiver] エラー: {port} が開けません。詳細: {e}")
            self.is_running = False
            return

        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def send_autopilot_command(self, cmd):
        """
        オートパイロットが計算したRCCommand（roll/pitch/yaw/throttle）を
        ground_receiver(main_pc.cpp)へ送信する。
        ground_receiver側は "AP," で始まる行だけをコマンドとして解釈し、
        受信した最新値をmailboxに保持して一定間隔でIM920経由ドローンへ転送する。
        """
        if not self.is_running or not self.ser.is_open:
            return

        send_str = f"AP,{cmd.roll:.4f},{cmd.pitch:.4f},{cmd.yaw:.4f},{cmd.throttle:.4f}\n"

        try:
            self.ser.write(send_str.encode('utf-8'))
        except Exception as e:
            print(f"[Serial] 送信エラー: {e}")

    def _receive_loop(self):
        print(f"[SerialReceiver] 受信ループを開始しました ({self.port})")
        while self.is_running:
            try:
                if self.ser.in_waiting > 0:
                    raw  = self.ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    # デバッグ用: 受信した生の行を表示（必要に応じてコメントアウト）
                    # print(f"[DEBUG RAW] {line}")

                    # パターン0: "DV,t_ms,dvx_body,dvy_body"（ヨー推定用）
                    # 受信時刻をここで打刻する。時刻合わせの精度に直結するため
                    # メインループまで持ち越さず、受信した瞬間に記録する。
                    if line.startswith("DV,"):
                        parts = line.split(",")
                        if len(parts) >= 4:
                            try:
                                rec = (int(float(parts[1])),   # t_ms
                                       float(parts[2]),        # dvx (前)
                                       float(parts[3]),        # dvy (右)
                                       time.time())            # 受信時刻
                            except ValueError:
                                continue
                            with self._dv_lock:
                                self._dv_queue.append(rec)
                            if not self._dv_received:
                                print(f"[SerialReceiver] 機体Δvの受信を確認: "
                                      f"dvx={rec[1]:+.3f} dvy={rec[2]:+.3f}")
                                self._dv_received = True
                        continue

                    # パターン1: "DATA,高度,ax,ay,az"
                    if line.startswith("DATA,"):
                        parts = line.split(",")
                        if len(parts) >= 5:
                            self.latest_altitude = float(parts[1])
                            self.latest_accel    = (float(parts[2]), float(parts[3]), float(parts[4]))
                        continue

                    # パターン2b: "Att: Roll=.. Pitch=.. Yaw=.."
                    # ヨー推定の旋回ゲート用。窓の間に機体が首を振っていると
                    # 機体座標系でのΔv合成が意味を失うため、その検出に使う。
                    # ※ "Alt" 判定より先に置くこと（"Att" と紛らわしいため）
                    if "Att" in line and "Yaw" in line:
                        m = re.search(r"Yaw\s*=\s*([-+]?\d+\.?\d*)", line)
                        if m:
                            self.latest_body_yaw = float(m.group(1))
                        continue

                    # パターン2: "Alt  : -8.92 m" または "lt: -31.08 m"
                    if "Alt" in line or "lt:" in line:
                        # "lt: -31.08 m" のような形式に対応
                        m = re.search(r"[-+]?\d+\.?\d*", line)
                        if m:
                            self.latest_altitude = float(m.group())

                    # パターン3: "Accel: [-45.96, -84.84, 152.80] g" 
                    # または "Acc: ax=-0.2144 ay=-0.3411 az=+1.0781 [g]"
                    if "Acc" in line or "accel" in line:
                        # "ax=-0.2144 ay=-0.3411 az=+1.0781" から数値を抽出
                        nums = re.findall(r"[-+]?\d+\.?\d*", line)
                        if len(nums) >= 3:
                            # 最初の3つの数値を ax, ay, az として取得
                            self.latest_accel = (float(nums[0]), float(nums[1]), float(nums[2]))
                            if not self._accel_received:
                                print(f"[SerialReceiver] 加速度の受信を確認: {self.latest_accel}")
                                self._accel_received = True

            except Exception as e:
                print(f"[SerialReceiver] 受信エラー: {e}")

            time.sleep(0.001) # ループを少し速める

    def send_yaw(self, yaw_deg: float, valid: bool):
        """
        カメラ由来の絶対ヨーを機体へ送る（YAW_HANDOFF.md §5-4）。

        機体側は valid=0 のとき完全に無視すること。
        送信頻度は 0.5〜1Hz でよい。速くしても通信ジッタが姿勢に乗るだけ。
        """
        if not self.is_running or not self.ser.is_open:
            return
        try:
            self.ser.write(f"YAW,{yaw_deg:.2f},{1 if valid else 0}\n".encode("utf-8"))
        except Exception as e:
            print(f"[Serial] ヨー送信エラー: {e}")

    def drain_body_dv(self):
        """
        受信済みの機体Δvを取り出して空にする。

        メインループから毎フレーム呼ぶ。ヨー推定器を単一スレッドで
        扱うために、受信スレッドとの受け渡しをここに閉じ込めている。

        Returns:
            [(t_ms, dvx_body, dvy_body, t_recv), ...]
        """
        with self._dv_lock:
            out = list(self._dv_queue)
            self._dv_queue.clear()
        return out

    def get_altitude(self):
        return self.latest_altitude

    def get_accel(self):
        return self.latest_accel

    def get_body_yaw(self):
        """機体のジャイロ積分ヨー [deg]。未受信なら None。"""
        return self.latest_body_yaw

    def stop(self):
        self.is_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()