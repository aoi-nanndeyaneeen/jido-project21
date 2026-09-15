"""
ble_monitor.py  -  log_recorder (XIAO ESP32C3) 経由で、BLE だけで完結する
                    デバッグ用ツール。

    機体 (drone_s5) ──UART──> log_recorder XIAO ──BLE 125Hz Notify──> ここ (見る)
    機体 (drone_s5) <──UART── log_recorder XIAO <──BLE Write──────── ここ (指令)

==========================================================================
これは何か
==========================================================================
1. Arduino IDE のシリアルモニタみたいに、機体状態を流しっぱなしで見る
   (console.py の画面は BLE の最新値を1行だけ上書き表示するが、値の
   推移をそのまま目で追いたいときはそれでは物足りない)。
2. PID reset / IMU再キャリブレーション / デバイス確認 (I2C再走査) を
   BLE 経由で送る (P / k / i キー)。

★ 2. は完全にデバッグ用限定。機体の操縦 (離着陸・速度・高度など) は
  ここからは絶対にできない。プロトコル (LogLinkProto.h の ActReq) に
  action と action_seq の2byteしかフィールドが無く、操縦系を送ろうにも
  送る場所が無い、という構造で保証してある。操縦は今まで通り
  console.py / main.py (IM920) だけが担う。

★ 新しい無線プロトコルを画面表示のために足したわけではない。BLE 経由で
  既に受け取っている FlightLog::Rec (bin2csv.py が解釈しているのと同じ
  バイナリ) を文字列にしているだけ。指令のほうは log_recorder 側に
  新設した Write 用 characteristic を使う (詳細は core/ble_tap.py)。

★ 地上局 (IM920 / COM ポート) には一切触らない。console.py / main.py と
  同時に起動しても構わない (BLE Notify は複数クライアントが繋がっても
  平気)。ただし操縦の代わりにはならない (見る・単発指令を送るだけ)。

==========================================================================
使い方
==========================================================================
    cd position_estimator/src
    python ble_monitor.py                  # 既定の名前 (S5-LogBLE) を自動で探す
    python ble_monitor.py --name S5-LogBLE
    python ble_monitor.py --save            # ついでに .BIN も残す (既定は残さない)
    python ble_monitor.py --no-keys         # P/k/i を無効化し、表示専用にする

キー:
    P   PID リセット (単押し)
    k   IMU 再キャリブレーション (2回押し。機体は非アーム中のみ実行する)
    i   デバイス確認 / I2C再走査 (機体は非アーム中のみ実行する)
    q   終了
"""
#普通にlog取ってなくてもBLE経由で状態が見たい！！！
#デバイスチェックはもっと正確に状態がわからないかな？
import argparse
import time
from pathlib import Path

from core.ble_tap import BleTap, ACT_PID_RESET, ACT_IMU_CAL, ACT_SELFTEST
from core.keyreader import KeyReader
from utils.config import LOG_DIR

# 'k' (IMU校正) を確定させるまでの2回目待ち時間 [s]。console.py と揃えてある。
CONFIRM_S = 3.0
# キー入力のポーリング間隔。人間の反応速度に対して十分速く、CPUも食わない。
POLL_S = 0.05


def main():
    ap = argparse.ArgumentParser(
        description="BLE 経由で機体ログを見て、デバッグ指令 (PID reset/"
                    "IMU校正/デバイス確認) を送るツール")
    ap.add_argument("--name", default="S5-LogBLE", help="BLE デバイス名")
    ap.add_argument("--save", action="store_true",
                    help="表示だけでなく .BIN にも保存する (既定: 保存しない)")
    ap.add_argument("--log-dir", default=str(LOG_DIR),
                    help="--save のときの保存先 (既定: position_estimator/src/logs)")
    ap.add_argument("--no-keys", action="store_true",
                    help="P/k/i を無効化し、表示専用にする (誤操作防止)")
    args = ap.parse_args()

    outdir = Path(args.log_dir) if args.save else Path(".ble_monitor_tmp")

    tap = BleTap(outdir, name=args.name, on_event=print, verbose=True)
    if not tap.start():
        print("[ble_monitor] 起動に失敗しました (上のメッセージを参照)")
        return

    print(f"[ble_monitor] '{args.name}' を探しています... (Ctrl+C または q で終了)")
    if not args.no_keys:
        print("[ble_monitor] P=PIDリセット  k=IMU校正(2回押し)  i=デバイス確認  q=終了")

    t_k_armed = 0.0   # 'k' の1回目を押した時刻 (0 = 未確定)

    def fire(action, label):
        seq = tap.next_action_seq()
        if tap.fire_action(action, seq):
            print(f"[ble_monitor] {label} を送信 (action_seq={seq})")
        else:
            print(f"[ble_monitor] {label} を送れませんでした (BLE未接続)")

    try:
        with KeyReader() as kr:
            while True:
                if not args.no_keys:
                    ch = kr.get()
                    now = time.time()
                    if ch == "q":
                        break
                    elif ch == "P":
                        fire(ACT_PID_RESET, "PID reset")
                    elif ch == "k":
                        if now - t_k_armed < CONFIRM_S:
                            fire(ACT_IMU_CAL, "IMU校正")
                            t_k_armed = 0.0
                        else:
                            t_k_armed = now
                            rec = tap.state()
                            armed = rec.get("armed")
                            warn = ("  ※テレメトリはアーム中と表示しています。"
                                    "拒否されます" if armed else "")
                            print(f"[ble_monitor] IMU校正? 機体を水平な床に置いて"
                                  f"静止させてから {CONFIRM_S:.0f} 秒以内に"
                                  f"もう一度 k を押してください{warn}")
                    elif ch == "i":
                        fire(ACT_SELFTEST, "デバイス確認(I2C再走査)")
                    time.sleep(POLL_S)
                else:
                    # --no-keys のときは何も読まずにただ待つ (表示専用)。
                    time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        tap.stop()
        if not args.save:
            # --save 無しのときは一時 .BIN を残さない (中身は画面に出し切っている)。
            for f in outdir.glob("LOG*.BIN"):
                try:
                    f.unlink()
                except OSError:
                    pass
            try:
                outdir.rmdir()
            except OSError:
                pass
        print("[ble_monitor] 終了しました")


if __name__ == "__main__":
    main()
