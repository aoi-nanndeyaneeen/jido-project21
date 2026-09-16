import socket
import csv
import os

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
# ★ 追加：1秒間データが来なければ一旦待ちを解除する（フリーズ対策）
sock.settimeout(1.0)

csv_filename = "bmp280_log_competition.csv"

if not os.path.exists(csv_filename):
    with open(csv_filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Temperature(C)", "Pressure(hPa)", "Raw_Alt(m)", "Smoothed_Alt(m)"])

print(f"PC側(IP: 192.168.11.15)で受信待機中... ポート: {UDP_PORT}")
print(f"受信したデータは '{csv_filename}' に自動保存されます。")
print("(終了するには Ctrl+C を押してください)")

try:
    while True:
        try:
            # ラズパイからデータを受信
            data, addr = sock.recvfrom(1024) 
            message = data.decode('utf-8')
            
            row = message.split(',')
            print(f"[{addr[0]}から受信] {row}")
            
            with open(csv_filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
                
        except socket.timeout:
            # ★ 追加：1秒間データが来なかったらここを通り、再度whileループの頭に戻る
            # この瞬間に Ctrl+C を受け付けることができるようになります
            continue

except KeyboardInterrupt:
    print("\n受信を終了しました。")
finally:
    sock.close()