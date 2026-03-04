import socket
import threading

class AltitudeReceiver:
    def __init__(self, port=5005):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)
        
        # 最新のデータを保持する変数
        self.latest_altitude = 0.0
        self.is_running = True
        
        # 受信用のバックグラウンドスレッドを開始
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print(f"[AltitudeReceiver] UDPポート {self.port} で高度データの受信待機を開始しました。")

    def _receive_loop(self):
        while self.is_running:
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode('utf-8')
                row = message.split(',')
                
                # rowの中身: [Time, Temp, Press, Raw_Alt, Smoothed_Alt]
                if len(row) >= 5:
                    # 補正後高度(Smoothed_Alt)を浮動小数点数として保存
                    self.latest_altitude = float(row[4])
                    
            except socket.timeout:
                continue # データが来なくてもフリーズせずにループを回す
            except Exception as e:
                print(f"[AltitudeReceiver] 受信エラー: {e}")

    def get_altitude(self):
        """いつでも最新の高度を即座に返すメソッド"""
        return self.latest_altitude

    def stop(self):
        """終了処理"""
        self.is_running = False
        self.sock.close()