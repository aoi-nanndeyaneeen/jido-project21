#シリアル通信

import serial
import threading
import time
import re

class SerialReceiver:
    def __init__(self, port="COM7", baudrate=115200):
        self.port     = port
        self.baudrate = baudrate
        self.latest_altitude = 0.0
        self.latest_accel    = (0.0, 0.0, 0.0)
        self._accel_received = False
        self.is_running      = True

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

                    # パターン1: "DATA,高度,ax,ay,az"
                    if line.startswith("DATA,"):
                        parts = line.split(",")
                        if len(parts) >= 5:
                            self.latest_altitude = float(parts[1])
                            self.latest_accel    = (float(parts[2]), float(parts[3]), float(parts[4]))
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

    def get_altitude(self):
        return self.latest_altitude

    def get_accel(self):
        return self.latest_accel

    def stop(self):
        self.is_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()