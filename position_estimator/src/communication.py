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

    def _receive_loop(self):
        while self.is_running:
            try:
                if self.ser.in_waiting > 0:
                    raw  = self.ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    # パターン1: "DATA,高度,ax,ay,az"
                    if line.startswith("DATA,"):
                        parts = line.split(",")
                        if len(parts) >= 5:
                            self.latest_altitude = float(parts[1])
                            self.latest_accel    = (float(parts[2]), float(parts[3]), float(parts[4]))
                        continue

                    # パターン2: "Alt  : -8.92 m"
                    if "Alt" in line:
                        m = re.search(r"[-+]?\d+\.?\d*", line)
                        if m:
                            self.latest_altitude = float(m.group())

                    # パターン3: "Accel: [-45.96, -84.84, 152.80] g"
                    if "Accel" in line or "accel" in line:
                        nums = re.findall(r"[-+]?\d+\.?\d*", line)
                        if len(nums) >= 3:
                            self.latest_accel = (float(nums[0]), float(nums[1]), float(nums[2]))
                            if not self._accel_received:
                                print(f"[SerialReceiver] 加速度の受信を確認: {self.latest_accel}")
                                self._accel_received = True

            except Exception as e:
                print(f"[SerialReceiver] 受信エラー: {e}")

            time.sleep(0.005)

    def get_altitude(self):
        return self.latest_altitude

    def get_accel(self):
        return self.latest_accel

    def stop(self):
        self.is_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()