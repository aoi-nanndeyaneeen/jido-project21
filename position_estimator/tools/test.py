import serial
import time
import threading
import re
import collections
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# 設定
# ==========================================
PORT = "COM7"      # ★実際のCOMポートに合わせてください
BAUDRATE = 115200
MAX_POINTS = 100   # グラフに表示するデータ数

# データを保持するキュー（描画用）
times_acc = collections.deque(maxlen=MAX_POINTS)
acc_x = collections.deque(maxlen=MAX_POINTS)
acc_y = collections.deque(maxlen=MAX_POINTS)
acc_z = collections.deque(maxlen=MAX_POINTS)

times_alt = collections.deque(maxlen=MAX_POINTS)
alts = collections.deque(maxlen=MAX_POINTS)

# ★最新の「生の受信データ」を保持（キャリブレーション用）
raw_x, raw_y, raw_z = 0.0, 0.0, 0.0
raw_alt = 0.0

# ★キャリブレーション（ゼロ地点）のオフセット値
offset_x, offset_y, offset_z = 0.0, 0.0, 0.0
offset_alt = 0.0

start_time = time.time()
is_running = True

# ==========================================
# キーボード操作（キャリブレーション機能）
# ==========================================
def on_key_press(event):
    global offset_x, offset_y, offset_z, offset_alt
    if event.key == 'c':  # 'c'キーでゼロリセット
        offset_x = raw_x
        offset_y = raw_y
        offset_z = raw_z
        offset_alt = raw_alt
        print(f"\n【キャリブレーション実行】現在の姿勢・高度を「0」に設定しました！")
        print(f"オフセット値 -> X:{offset_x:.2f}, Y:{offset_y:.2f}, Z:{offset_z:.2f}, Alt:{offset_alt:.2f}m\n")

# ==========================================
# シリアル通信＆CSVログ保存用スレッド
# ==========================================
def serial_thread():
    global is_running, raw_x, raw_y, raw_z, raw_alt
    
    log_filename = f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    try:
        print(f"{PORT} に接続を試みます...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("接続成功！データを受信し、グラフを描画します。")
        print(f"★ ログ保存先: {log_filename}")
        print("★ グラフ選択中にキーボードの『 C 』を押すとゼロリセットされます ★\n")
        
        # CSVファイルのヘッダーを書き込み
        with open(log_filename, "w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time", "Roll_X(deg)", "Pitch_Y(deg)", "Yaw_Z(deg)", "Altitude(m)"])
            
            while is_running:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    current_t = time.time() - start_time
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    # データの更新があったかどうかのフラグ
                    updated = False
                    
                    # パターン1: "DATA, 高度, X, Y, Z" の形式で来た場合
                    if line.startswith("DATA,"):
                        parts = line.split(",")
                        if len(parts) >= 5:
                            raw_alt = float(parts[1])
                            raw_x = float(parts[2])
                            raw_y = float(parts[3])
                            raw_z = float(parts[4])
                            updated = True
                            
                    # パターン2: "Accel: [X, Y, Z]" の形式で来た場合
                    match_acc = re.search(r"Accel:\s*\[([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)\]", line)
                    if match_acc:
                        raw_x = float(match_acc.group(1))
                        raw_y = float(match_acc.group(2))
                        raw_z = float(match_acc.group(3))
                        updated = True
                        
                    # パターン3: "Alt : Z m" の形式で来た場合
                    match_alt = re.search(r"Alt\s*:\s*([\d\.-]+)", line)
                    if match_alt:
                        raw_alt = float(match_alt.group(1))
                        updated = True

                    # データのどれかが更新されていれば、計算してグラフとCSVに反映
                    if updated:
                        # 常に最新のrawデータからオフセットを引いてゼロ基準にする
                        calib_x = raw_x - offset_x
                        calib_y = raw_y - offset_y
                        calib_z = raw_z - offset_z
                        calib_alt = raw_alt - offset_alt
                        
                        times_acc.append(current_t)
                        acc_x.append(calib_x)
                        acc_y.append(calib_y)
                        acc_z.append(calib_z)
                        
                        times_alt.append(current_t)
                        alts.append(calib_alt)
                        
                        # ★ここで確実にCSVに1行書き込む！
                        writer.writerow([timestamp, round(calib_x, 2), round(calib_y, 2), round(calib_z, 2), round(calib_alt, 2)])
                        csvfile.flush() # 即座にファイルに保存
                            
                time.sleep(0.001)
                
    except Exception as e:
        print(f"\n【通信エラー】{e}")
        is_running = False

# ==========================================
# グラフ描画の設定
# ==========================================
fig = plt.figure(figsize=(10, 7))
fig.canvas.manager.set_window_title('Flight Telemetry')

fig.canvas.mpl_connect('key_press_event', on_key_press)

ax_acc = fig.add_subplot(2, 1, 1) 
ax_alt = fig.add_subplot(2, 1, 2) 

# test_serial.py の下の方にある update_graph 関数を上書きしてください

def update_graph(frame):
    # --- 角度グラフの更新 ---
    ax_acc.cla()
    ax_acc.set_title("Attitude Angles (Roll & Pitch)", fontsize=14)
    ax_acc.set_ylabel("Degrees (°)")
    ax_acc.grid(True, linestyle='--', alpha=0.7)
    
    # ★追加: 縦軸のメモリを -90度 から +90度 に完全固定する！
    ax_acc.set_ylim(-90, 90)
    
    if len(times_acc) > 0:
        ax_acc.plot(times_acc, acc_x, label="Roll (X)", color="#FF5733", linewidth=2.0)
        ax_acc.plot(times_acc, acc_y, label="Pitch (Y)", color="#33FF57", linewidth=2.0)
        
        # ★変更: Z軸(Yaw)はグラフを狂わせるのでコメントアウトして非表示にする
        # ax_acc.plot(times_acc, acc_z, label="Yaw (Z)", color="#3357FF", linewidth=1.5)
        
        ax_acc.legend(loc="upper left")
        
    # --- 高度グラフの更新 (ここはそのまま) ---
    ax_alt.cla()
    ax_alt.set_title("Relative Altitude", fontsize=14)
    ax_alt.set_xlabel("Time (Seconds)")
    ax_alt.set_ylabel("Meters (m)")
    ax_alt.grid(True, linestyle='--', alpha=0.7)
    
    # 高度も、もし見にくければ例えば -1m 〜 +3m に固定することもできます
    # ax_alt.set_ylim(-1, 3) 
    
    if len(times_alt) > 0:
        ax_alt.plot(times_alt, alts, label="Altitude", color="purple", linewidth=2.0)
        latest_alt = alts[-1]
        ax_alt.text(0.02, 0.90, f"Current Alt: {latest_alt:.2f} m", 
                    transform=ax_alt.transAxes, fontsize=12, fontweight='bold', color='purple')
        ax_alt.legend(loc="upper left")
# ==========================================
# メイン実行部
# ==========================================
if __name__ == "__main__":
    thread = threading.Thread(target=serial_thread, daemon=True)
    thread.start()

    print("グラフウィンドウを起動しています...")
    ani = animation.FuncAnimation(fig, update_graph, interval=100, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show() 
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        print("\nログ取得とグラフ描画を終了しました。")