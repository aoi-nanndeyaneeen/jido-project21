import csv
import matplotlib.pyplot as plt

# ★ 読み込むCSVファイル名に合わせて変更してください
FILENAME = "flight_data_20260304_143655.csv"

times = []
roll, pitch, yaw = [], [], []
alt = []

print(f"{FILENAME} を読み込んでいます...")

try:
    with open(FILENAME, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # データの読み込み
            times.append(row['Time'])
            roll.append(float(row['Roll_X(deg)']))
            pitch.append(float(row['Pitch_Y(deg)']))
            yaw.append(float(row['Yaw_Z(deg)']))
            alt.append(float(row['Altitude(m)']))

    # === グラフの描画設定 ===
    fig, (ax_acc, ax_alt) = plt.subplots(2, 1, figsize=(12, 8))
    fig.canvas.manager.set_window_title('Flight Data Analysis')

    # X軸のラベル（時間）が多すぎると重なるので、適度に間引くためのインデックス
    x_indices = range(len(times))

    # --- 上段：姿勢（角度）グラフ ---
    ax_acc.plot(x_indices, roll, label="Roll (X)", color="#FF5733", linewidth=2)
    ax_acc.plot(x_indices, pitch, label="Pitch (Y)", color="#33FF57", linewidth=2)
    ax_acc.plot(x_indices, yaw, label="Yaw (Z)", color="#3357FF", linewidth=2)
    
    ax_acc.set_title("Attitude Angles (Roll / Pitch / Yaw)", fontsize=14)
    ax_acc.set_ylabel("Degrees (°)", fontsize=12)
    ax_acc.grid(True, linestyle='--', alpha=0.7)
    ax_acc.legend(loc="upper right")
    # 0度のラインを強調
    ax_acc.axhline(0, color='black', linewidth=1, linestyle='-')

    # --- 下段：高度グラフ ---
    ax_alt.plot(x_indices, alt, label="Altitude", color="purple", linewidth=2)
    
    ax_alt.set_title("Relative Altitude", fontsize=14)
    ax_alt.set_ylabel("Meters (m)", fontsize=12)
    ax_alt.set_xlabel("Data Points (Time ->)", fontsize=12)
    ax_alt.grid(True, linestyle='--', alpha=0.7)
    ax_alt.legend(loc="upper right")
    # 0mのラインを強調
    ax_alt.axhline(0, color='black', linewidth=1, linestyle='-')

    plt.tight_layout()
    print("グラフを描画しました！ウィンドウ下部の虫眼鏡マークで拡大できます。")
    plt.show()

except FileNotFoundError:
    print(f"エラー: '{FILENAME}' が見つかりません。ファイル名が正しいか確認してください。")
except Exception as e:
    print(f"エラーが発生しました: {e}")