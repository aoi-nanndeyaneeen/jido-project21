import serial
import csv
import time
import re
import argparse
import sys
import os

def main():
    parser = argparse.ArgumentParser(description="Ground Receiver Serial Logger")
    parser.add_argument("--port", "-p", type=str, default="COM3", help="Serial port name (e.g., COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--output", "-o", type=str, default="flight_log", help="Output CSV filename prefix")
    args = parser.parse_args()

    # Create logs directory if it doesn't exist
    log_dir = os.path.join(os.path.dirname(__file__), "logs")
    os.makedirs(log_dir, exist_ok=True)
    
    filename = os.path.join(log_dir, time.strftime(f"{args.output}_%Y%m%d_%H%M%S.csv"))
    
    # ANSI escape sequence remover (for color codes and clear screen commands)
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

    print(f"[{args.port} - {args.baud}bps] 接続を待機中...")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except Exception as e:
        print(f"エラー: シリアルポート {args.port} を開けません。({e})")
        print("デバイスマネージャー等で正しいCOMポートを確認し、'-p COM数字' オプションを付けて実行してください。")
        sys.exit(1)

    print(f"ログの記録を開始します:\n -> {filename}")
    print("終了するには Ctrl+C を押してください")

    with open(filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Roll(deg)", "Pitch(deg)", "Yaw(deg)", "Ax(g)", "Ay(g)", "Az(g)", "Alt(m)"])

        current_data = {"Time": 0, "Roll": 0, "Pitch": 0, "Yaw": 0, "Ax": 0, "Ay": 0, "Az": 0, "Alt": 0}
        start_time = time.time()
        
        try:
            while True:
                line = ser.readline()
                if not line:
                    continue
                
                try:
                    text = line.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue
                    
                # エスケープシーケンスを除去
                clean_text = ansi_escape.sub('', text)
                
                # Roll_Ang:+10.00,Pitch_Ang: -5.00,Yaw_Ang: +2.00
                if "Roll_Ang:" in clean_text:
                    m = re.search(r'Roll_Ang:\s*([+-]?[\d.]+).*Pitch_Ang:\s*([+-]?[\d.]+).*Yaw_Ang:\s*([+-]?[\d.]+)', clean_text)
                    if m:
                        current_data["Roll"] = float(m.group(1))
                        current_data["Pitch"] = float(m.group(2))
                        current_data["Yaw"] = float(m.group(3))
                        
                # Acc: ax=+0.1234 ay=-0.1234 az=+0.9876 [g]
                elif "Acc: ax=" in clean_text:
                    m = re.search(r'ax=\s*([+-]?[\d.]+).*ay=\s*([+-]?[\d.]+).*az=\s*([+-]?[\d.]+)', clean_text)
                    if m:
                        current_data["Ax"] = float(m.group(1))
                        current_data["Ay"] = float(m.group(2))
                        current_data["Az"] = float(m.group(3))
                        
                # Alt: +50.23 m
                elif "Alt:" in clean_text and "m" in clean_text:
                    m = re.search(r'Alt:\s*([+-]?[\d.]+)', clean_text)
                    if m:
                        current_data["Alt"] = float(m.group(1))
                        
                        # Altが機体から来る最後の行想定なので、ここでCSVに1行書き込み
                        current_data["Time"] = round(time.time() - start_time, 3)
                        writer.writerow([
                            current_data["Time"], current_data["Roll"], current_data["Pitch"], 
                            current_data["Yaw"], current_data["Ax"], current_data["Ay"], 
                            current_data["Az"], current_data["Alt"]
                        ])
                        f.flush() # 途中で強制終了されても消えないように保存
                        
        except KeyboardInterrupt:
            print(f"\n記録を終了しました。ログは '{filename}' に保存されています。")
        finally:
            ser.close()

if __name__ == "__main__":
    main()
