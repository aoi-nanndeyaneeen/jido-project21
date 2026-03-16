import csv
import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    parser = argparse.ArgumentParser(description="3D Flight Data Plotter")
    parser.add_argument("csv_file", type=str, help="Path to the logged CSV file")
    args = parser.parse_args()

    times = []
    roll = []
    pitch = []
    yaw = []
    alt = []

    try:
        with open(args.csv_file, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    times.append(float(row["Time"]))
                    roll.append(float(row["Roll(deg)"]))
                    pitch.append(float(row["Pitch(deg)"]))
                    yaw.append(float(row["Yaw(deg)"]))
                    alt.append(float(row["Alt(m)"]))
                except (ValueError, KeyError):
                    continue
    except FileNotFoundError:
        print(f"エラー: ファイル '{args.csv_file}' が見つかりません。")
        sys.exit(1)
        
    if len(times) == 0:
        print("データがありませんでした。")
        sys.exit(1)

    # Convert to numpy arrays
    times = np.array(times)
    roll = np.array(roll)
    pitch = np.array(pitch)
    yaw = np.array(yaw)
    alt = np.array(alt)

    print(f"読み込み完了: {len(times)} 件のデータレコード")

    # ==========================================
    # 3D Plotting
    # ==========================================
    fig = plt.figure(figsize=(10, 8))
    fig.canvas.manager.set_window_title('3D Flight Telemetry Plot')
    ax = fig.add_subplot(111, projection='3d')

    # X: Time, Y: Roll, Z: Pitch (Or change to whatever makes sense for your analysis!)
    # GPSがないため、ここでは 時間 x Roll x Pitch を3D空間にマッピングする例
    # 実際の3D軌跡(Trajectory)を描画する場合は、カルマンフィルタ等で速度・位置に変換したX,Y,Zが必要です。
    
    # 描画オプション1: 高度(Alt)と姿勢(Roll/Pitch)の関係を3Dで見る
    scatter = ax.scatter(roll, pitch, alt, c=times, cmap='viridis', s=10, alpha=0.8)
    ax.plot(roll, pitch, alt, color='gray', alpha=0.5, linewidth=1)

    ax.set_title("3D Attitude & Altitude Plot\n(Color = Time)")
    ax.set_xlabel("Roll (deg)")
    ax.set_ylabel("Pitch (deg)")
    ax.set_zlabel("Altitude (m)")

    # タイムラインのカラーバーを追加
    cbar = fig.colorbar(scatter, ax=ax, pad=0.1, shrink=0.7)
    cbar.set_label("Time (s)")

    print("グラフを描画しました。ウィンドウ上でドラッグして回転できます。")
    plt.show()

if __name__ == "__main__":
    main()
