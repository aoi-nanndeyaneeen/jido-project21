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
              "Pair_Rejected", "In_Dummy"]

    def __init__(self, log_path: Path):
        super().__init__(log_path, self.HEADER, mode="a")

    def write(self, P_vec, residual, n_cand1, n_cand2, uv1, uv2,
              pair_rejected, in_dummy):
        """毎フレーム呼ぶ。P_vec=None なら未検知として記録"""
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
        self._writer.writerow(row)
        self._fh.flush()


class MissionLogger(CsvLogger):
    """
    指令を出した瞬間ごとに「PCの判断・カメラの見立て・機体の言い分」を1行に並べる。

    ★ このログの主眼は Diff_*。機体はフロー(光学式)の積分で自分の位置を
      推定しており、これは必ず流れていく。カメラの絶対位置と並べて初めて
      「どれだけ流れたか」が測れる。機体単独のログでは絶対に分からない。

    ★ 機体の pos_n/pos_e は「フロー保持に入った場所を原点とする相対位置」。
      フィールド座標とは原点が違うので、両方が同時に有効になった最初の
      瞬間のオフセットを覚えて引く (set_align)。それ以降の差が「ずれ」。
      生値 (Fh_PosN/Fh_PosE) も残してあるので、後から別の基準で取り直せる。
    """

    HEADER = [
        "Epoch_s", "Time",
        # PC 側のミッション状態 (Phase=送信時 / Phase_Next=送信後。違う行が遷移)
        "Phase", "Phase_Next", "WP_Idx", "Returning",
        "Tgt_X(m)", "Tgt_Y(m)", "Tgt_Z(m)", "Dist_H(m)",
        # カメラの見立て (これを真値の基準として扱う)
        "Cam_X(m)", "Cam_Y(m)", "Cam_Z(m)", "Pos_Valid", "In_Dummy", "Residual(m)",
        "Yaw_deg", "Yaw_Valid", "Yaw_Src",
        # PC が実際に送った指令
        "Req", "Cmd_Vx(m/s)", "Cmd_Vy(m/s)", "Cmd_Alt(m)", "Flags",
        # 機体の状態
        "Mode", "Armed", "Guided", "Cmd_Fresh", "Landed", "Airborne",
        "Range_H(m)", "Alt_Hold(m)", "Tel_Age(s)",
        # 機体自身の水平位置推定と、カメラとの食い違い
        "Fh_PosN(m)", "Fh_PosE(m)", "Fh_HoldN(m)", "Fh_HoldE(m)",
        "Drone_X(m)", "Drone_Y(m)", "Diff_X(m)", "Diff_Y(m)", "Diff_Norm(m)",
        "Fh_Vxt(m/s)", "Fh_Vyt(m/s)", "Fh_Vxc(m/s)", "Fh_Vyc(m/s)",
        "Flow_OK", "Range_Valid", "Bad",
        "Event",
    ]

    # 機体モード番号 -> 名前 (s5_link.py の docstring と同じ対応)
    MODE_NAME = {0: "RATE", 1: "ANGLE", 2: "GUIDED", 3: "POSHOLD", 4: "ALTHOLD"}

    def __init__(self, log_path: Path):
        super().__init__(log_path, self.HEADER, mode="w")
        self._align = None      # (dx, dy) フィールド座標 - 機体座標

    def set_align(self, cam_xy, fh_ne):
        """
        カメラと機体フローの原点合わせ。両方が同時に有効な最初の1回だけ呼ぶ。

        機体の n(北) をフィールドの +y(奥)、e(東) を +x(右) に対応させる。
        これは「機首をフィールド奥へ向けて置く」という運用前提そのもの
        (config の YAW_INITIAL_ALIGN_DEG=0)。前提が崩れていれば Diff_* に
        回転ずれとして現れる。
        """
        self._align = (cam_xy[0] - fh_ne[1], cam_xy[1] - fh_ne[0])

    @property
    def aligned(self):
        return self._align is not None

    def write(self, mission_snap, cam, tel, yaw, event=""):
        """
        Args:
            mission_snap: WaypointMission.snapshot()
            cam : dict  x,y,z,valid,in_dummy,residual
            tel : dict  機体テレメトリ (S5Link.state()) + age
            yaw : dict  deg,valid,src
        """
        epoch, hms = self._stamp()
        m, t = mission_snap, tel

        cam_x, cam_y = cam.get("x"), cam.get("y")
        fh_n, fh_e = t.get("fh_posn"), t.get("fh_pose")

        # ---- 機体の自己位置をフィールド座標へ写して差を取る ----------
        drone_x = drone_y = dif_x = dif_y = dif_n = None
        if self._align is not None and fh_n is not None and fh_e is not None:
            drone_x = fh_e + self._align[0]
            drone_y = fh_n + self._align[1]
            if cam.get("valid") and cam_x is not None:
                dif_x = cam_x - drone_x
                dif_y = cam_y - drone_y
                dif_n = (dif_x ** 2 + dif_y ** 2) ** 0.5

        mode = t.get("mode")
        mode_name = self.MODE_NAME.get(int(mode), str(mode)) if mode is not None else ""

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
            m["req"], _f(m["vx"]), _f(m["vy"]), _f(m["alt"]), m["flags"],
            mode_name,
            int(bool(t.get("armed"))), int(bool(t.get("guided"))),
            int(bool(t.get("cmd_fresh"))), int(bool(t.get("landed"))),
            int(bool(t.get("airborne"))),
            _f(t.get("range_h")), _f(t.get("alt_hold")), _f(t.get("age")),
            _f(fh_n), _f(fh_e), _f(t.get("fh_holdn")), _f(t.get("fh_holde")),
            _f(drone_x), _f(drone_y), _f(dif_x), _f(dif_y), _f(dif_n),
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
