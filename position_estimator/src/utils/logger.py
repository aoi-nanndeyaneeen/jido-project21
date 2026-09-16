# CSVログ記録クラス
#
# ---------------------------------------------------------------------------
# 3つのログの役割分担と、突き合わせ方
# ---------------------------------------------------------------------------
#   flight_*.csv    カメラスレッド (30〜60Hz)  検知と幾何。何を機体だと判定したか
#   mission_*.csv   メインループ   (5Hz)       制御と突き合わせ。何を送り、機体が
#                                              どう認識していたか
#   s5_link_*.csv   受信スレッド   (12〜15Hz)  機体テレメトリの生ログ
#
# ★ 3つとも先頭列に Epoch_s (time.time() の実数) を持つ。これが唯一の
#   突き合わせキー。以前は機体側の millis() しか無く、カメラログと
#   テレメトリを時刻で並べられずに解析が推測まじりになっていた。
#   列を足すときも Epoch_s だけは必ず先頭に置くこと。

import csv
import datetime
import time
from pathlib import Path


class CsvLogger:
    """ディレクトリ作成・ファイルオープン・ヘッダー書き込みをまとめた共通CSVロガー基底クラス"""

    def __init__(self, path: Path, header: list, mode: str = "w"):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        write_header = mode == "w" or not path.exists()
        self._fh = open(path, mode=mode, newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        if write_header:
            self._writer.writerow(header)

    @staticmethod
    def _stamp():
        """(epoch秒, 人間が読む時刻) を返す。両方を同じ瞬間から作る。"""
        now = time.time()
        hms = datetime.datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3]
        return round(now, 3), hms

    def close(self):
        self._fh.close()


def _f(v, nd=3):
    """None を空欄に、それ以外を丸める。CSV に 'None' と書かせない。"""
    if v is None:
        return ""
    return round(float(v), nd)


class FlightLogger(CsvLogger):
    """
    カメラが1フレームごとに「何を見て、何を機体だと決めたか」。

    ★ 候補の個数と採用画素を残すのが要点。位置だけ見ても、窓の反射を掴んだのか
      本当に機体を見失ったのかが後から区別できない。
    """

    HEADER = ["Epoch_s", "Time", "Detected",
              "Pos_X(m)", "Pos_Y(m)", "Pos_Z(m)", "Residual(m)",
              "N_Cand1", "N_Cand2", "U1", "V1", "U2", "V2",
              "Pair_Rejected", "In_Dummy",
              # ★ 2026-09-16: 両カメラで点滅確認できているのに位置が出ない
              #   ときの理由 (residual / gate / reach) と、最も惜しかった
              #   ペアの三角測量結果。これが無いと「基準5点の座標が
              #   フィールド寸法と食い違っていた」ような不具合をログから
              #   切り分けられない。
              "Reject_Why", "Reject_X(m)", "Reject_Y(m)", "Reject_Z(m)",
              "Reject_Residual(m)",
              # 各カメラの最良トラックのロックインスコア (6Hz成分の割合, 0-1) と
              # 変調深さ (輝度の標準偏差)。黄色 (確定) の条件は
              # score>=BLINK_MIN_SCORE かつ depth>=BLINK_MIN_DEPTH。
              "Blink1_Score", "Blink1_Depth", "Blink2_Score", "Blink2_Depth"]

    def __init__(self, log_path: Path):
        super().__init__(log_path, self.HEADER, mode="a")

    def write(self, P_vec, residual, n_cand1, n_cand2, uv1, uv2,
              pair_rejected, in_dummy, reject=None,
              blink1=(0.0, 0.0), blink2=(0.0, 0.0)):
        """毎フレーム呼ぶ。P_vec=None なら未検知として記録。
        reject は PairSelector.last_reject = (理由, P, residual) か None。
        blink1/blink2 は BlinkTracker.best() の (score, depth)。"""
        epoch, hms = self._stamp()
        row = [epoch, hms, int(P_vec is not None)]
        if P_vec is not None:
            row += [_f(P_vec[0]), _f(P_vec[1]), _f(P_vec[2])]
        else:
            row += ["", "", ""]
        row += [_f(residual),
                int(n_cand1), int(n_cand2),
                _f(uv1[0], 1) if uv1 else "", _f(uv1[1], 1) if uv1 else "",
                _f(uv2[0], 1) if uv2 else "", _f(uv2[1], 1) if uv2 else "",
                int(bool(pair_rejected)), int(bool(in_dummy))]
        if reject is not None:
            why, Pr, rr = reject
            row += [why, _f(Pr[0]), _f(Pr[1]), _f(Pr[2]), _f(rr)]
        else:
            row += ["", "", "", "", ""]
        row += [_f(blink1[0]), _f(blink1[1], 1), _f(blink2[0]), _f(blink2[1], 1)]
        self._writer.writerow(row)
        self._fh.flush()


class MissionLogger(CsvLogger):
    """
    指令を出した瞬間ごとに「PCの判断・カメラの見立て・機体の言い分」を1行に並べる。

    ★ このログの主眼は Diff_*。機体はフロー(光学式)の積分で自分の位置を
      推定しており、これは必ず流れていく。カメラの絶対位置と並べて初めて
      「どれだけ流れたか」が測れる。機体単独のログでは絶対に分からない。

    ★ 原点合わせと Diff_* の計算そのものは core/mission.py が持っている
      (2026-09-14〜: 同じ値を使って機体の自己位置を定期的に補正するため、
      ログ専用にここでもう一度計算すると2つの基準がずれかねない)。
      ここはその結果 (snapshot()["diff"]) を書くだけ。
    """

    HEADER = [
        "Epoch_s", "Time",
        # PC 側のミッション状態 (Phase=送信時 / Phase_Next=送信後。違う行が遷移)
        "Phase", "Phase_Next", "WP_Idx", "Returning",
        "Tgt_X(m)", "Tgt_Y(m)", "Tgt_Z(m)", "Dist_H(m)",
        # カメラの見立て (これを真値の基準として扱う)
        "Cam_X(m)", "Cam_Y(m)", "Cam_Z(m)", "Pos_Valid", "In_Dummy", "Residual(m)",
        "Yaw_deg", "Yaw_Valid", "Yaw_Src",
        # 機体自身のyaw (アーム基準の相対方位) と、PCが仮定しているyawとの差。
        # Yaw_Src=fixed のときにこれが 0 から離れていたら、機首はもう
        # 「フィールド奥を向いている」という前提から外れている
        # (2026-09-14: 複数回リトライの間に手動操作で30度以上ズレた事故)。
        "Yaw_Device_deg", "Yaw_Mismatch_deg",
        # PC が実際に送った指令
        "Req", "Cmd_Vx(m/s)", "Cmd_Vy(m/s)", "Cmd_Alt(m)", "Flags",
        # 機体の状態
        "Mode", "Armed", "Guided", "Cmd_Fresh", "Landed", "Airborne",
        "Range_H(m)", "Alt_Hold(m)", "Tel_Age(s)",
        # 機体自身の水平位置推定と、カメラとの食い違い
        "Fh_PosN(m)", "Fh_PosE(m)", "Fh_HoldN(m)", "Fh_HoldE(m)",
        "Drone_X(m)", "Drone_Y(m)", "Diff_X(m)", "Diff_Y(m)", "Diff_Norm(m)",
        "Aligned",
        # この行で地上補正 (mission.py) を送ったか。送った場合の絶対目標値
        "Pos_Corr_Sent", "Pos_Corr_N(m)", "Pos_Corr_E(m)",
        "Fh_Vxt(m/s)", "Fh_Vyt(m/s)", "Fh_Vxc(m/s)", "Fh_Vyc(m/s)",
        "Flow_OK", "Range_Valid", "Bad",
        "Event",
    ]

    # 機体モード番号 -> 名前 (s5_link.py の docstring と同じ対応)
    MODE_NAME = {0: "RATE", 1: "ANGLE", 2: "GUIDED", 3: "POSHOLD", 4: "ALTHOLD"}

    def __init__(self, log_path: Path):
        super().__init__(log_path, self.HEADER, mode="w")

    def write(self, mission_snap, cam, tel, yaw, event=""):
        """
        Args:
            mission_snap: WaypointMission.snapshot()
                          ("diff" = (diff_x,diff_y,diff_norm,drone_x,drone_y) か None、
                           "aligned" = 原点合わせが済んでいるか、
                           "corr" = この行で送った補正 (n_m,e_m) か None)
            cam : dict  x,y,z,valid,in_dummy,residual
            tel : dict  機体テレメトリ (S5Link.state()) + age
            yaw : dict  deg,valid,src
        """
        epoch, hms = self._stamp()
        m, t = mission_snap, tel

        cam_x, cam_y = cam.get("x"), cam.get("y")
        fh_n, fh_e = t.get("fh_posn"), t.get("fh_pose")

        diff = m.get("diff")
        dif_x, dif_y, dif_n, drone_x, drone_y = diff if diff else (None,) * 5
        corr = m.get("corr")

        mode = t.get("mode")
        mode_name = self.MODE_NAME.get(int(mode), str(mode)) if mode is not None else ""

        # 機体自身のyaw (アーム基準の相対方位) と、PC想定yawとの差。
        # yaw_src="camera" のときは別物 (機体側は絶対方位を持たない) なので
        # 比較しない。fixed のときだけ意味がある。
        dev_yaw = t.get("yaw")
        yaw_mismatch = None
        if dev_yaw is not None and yaw.get("src") != "camera" and yaw.get("deg") is not None:
            yaw_mismatch = dev_yaw - yaw["deg"]

        self._writer.writerow([
            epoch, hms,
            m["phase"], m["phase_next"], m["wp_idx"], int(m["returning"]),
            _f(m["tgt"][0]) if m["tgt"] else "",
            _f(m["tgt"][1]) if m["tgt"] else "",
            _f(m["tgt"][2]) if m["tgt"] else "",
            _f(m["dist_h"]),
            _f(cam_x), _f(cam_y), _f(cam.get("z")),
            int(bool(cam.get("valid"))), int(bool(cam.get("in_dummy"))),
            _f(cam.get("residual")),
            _f(yaw.get("deg"), 2), int(bool(yaw.get("valid"))), yaw.get("src", ""),
            _f(dev_yaw, 1), _f(yaw_mismatch, 1),
            m["req"], _f(m["vx"]), _f(m["vy"]), _f(m["alt"]), m["flags"],
            mode_name,
            int(bool(t.get("armed"))), int(bool(t.get("guided"))),
            int(bool(t.get("cmd_fresh"))), int(bool(t.get("landed"))),
            int(bool(t.get("airborne"))),
            _f(t.get("range_h")), _f(t.get("alt_hold")), _f(t.get("age")),
            _f(fh_n), _f(fh_e), _f(t.get("fh_holdn")), _f(t.get("fh_holde")),
            _f(drone_x), _f(drone_y), _f(dif_x), _f(dif_y), _f(dif_n),
            int(bool(m.get("aligned"))),
            int(corr is not None), _f(corr[0]) if corr else "", _f(corr[1]) if corr else "",
            _f(t.get("fh_vxt")), _f(t.get("fh_vyt")),
            _f(t.get("fh_vxc")), _f(t.get("fh_vyc")),
            int(bool(t.get("flow_ok"))), int(bool(t.get("range_valid"))),
            _f(t.get("bad"), 0),
            event,
        ])
        self._fh.flush()


class PerformanceLogger(CsvLogger):
    HEADER = ["Epoch_s", "Time", "Kind", "Loop_ms", "Cam1_ms", "Cam2_ms",
              "FrameAge1_ms", "FrameAge2_ms", "ReaderFPS1", "ReaderFPS2",
              "ReaderMs1", "ReaderMs2", "ReadErrors1", "ReadErrors2",
              "Display_ms", "RC_ms", "Velocity_ms", "Graph_ms"]

    def __init__(self, log_path: Path):
        super().__init__(log_path, self.HEADER, mode="w")

    def write(self, kind, values):
        epoch, hms = self._stamp()
        row = [epoch, hms, kind]
        row.extend(round(values.get(name, 0.0), 3) for name in self.HEADER[3:])
        self._writer.writerow(row)
        self._fh.flush()
