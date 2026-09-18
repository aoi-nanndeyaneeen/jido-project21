# 定数、パス、フィールド設定などをまとめるモジュール

import json
from pathlib import Path
import numpy as np

# ==========================================
# パス設定
# ==========================================
# ※ 歴史的経緯で名前と実体がずれている。既存の logs/ calib/ の場所を
#    変えると保存済みキャリブが読めなくなるため、名前はそのままにしてある。
SRC_DIR  = Path(__file__).parent      # 実体は src/utils/
ROOT_DIR = SRC_DIR.parent             # 実体は src/
LOG_DIR  = ROOT_DIR / "logs"          # src/logs/   （calib は src/calib/）

# position_estimator/ 直下。camera_server.py と detection_params.json がある場所。
PROJECT_DIR = SRC_DIR.parent.parent

# ==========================================
# カメラ設定
# ==========================================
# ラズパイの mjpg-streamer などのネットワークストリームURLを指定します。
# ローカルのUSBカメラIDを直接指定することも可能です（例: 0, 1）。
CAMERA_1_URL = 1                    # ラップトップUSBカメラ
# Camera2の接続先。通常はRPI、LaptopへUSB直結する一時運用ではUSBにする。
CAMERA2_SOURCE = "RPI"             # "USB" または "RPI"
CAMERA_2_URL = 2                    # USB時のCamera2デバイス番号
RPI_HOST     = "192.168.11.13"      # ラズパイIP
RPI_PORT     = 5555  # カメラ2 (ラズパイ接続)
# ★ 本番は 1 分の準備時間に 2 人が同時に camera_server.py と main.py を起動する。
#   どちらが先でもよいように、PC 側がこの秒数だけ RPi の立ち上がりを待つ。
#   (2026-09-17 まで: 3秒のプローブ1回きりで、外れると Camera2 がダミーに
#    固定され、main.py を再起動するしか無かった。init/camera_setup.py 冒頭)
RPI_WAIT_S   = 60.0

CAMERA_W = 1280   # 要求解像度 (幅)
CAMERA_H =  720   # 要求解像度 (高さ)

CAMERA_FPS = 60   # 要求FPS（ハードウェアが非対応の場合は自動で上限にフォールバック）

# ==========================================
# カメラの自動制御
# ==========================================
# ★ オートフォーカスは必ず無効にすること。
#    フォーカスが動くと焦点距離そのものが変わり、
#    チェッカーボードで測った内部パラメータ K が無効になる。
#    ノイズの問題ではなく幾何精度の根幹。対象は15〜60m先なので無限遠固定でよい。
CAMERA_AUTOFOCUS   = False
CAMERA_FOCUS_VALUE = 0      # 0 = 無限遠 (C920の場合)

# 露出・ホワイトバランスは「キャリブレーションまでオート → 完了時点の値で固定」。
# 会場の明るさに自動で合わせつつ、競技中は変動しない。当日の操作は不要。
LOCK_EXPOSURE_AFTER_CALIB = True

# ---- 追跡用の露出 (bright 検知モードで使う) --------------------------------
# 機体のLEDだけを白飛びさせるため、キャリブレーション完了後に露出を
# 「自動が選んだ値」ではなく、明示した暗い値へ落とす。
#
#   None  : 従来どおり。自動露出が選んだ値でそのまま固定する
#   数値  : その値を CAP_PROP_EXPOSURE に設定する
#
# ★ 値の意味はドライバ依存。Windows の DirectShow(UVC) では
#   概ね 2 のべき乗の負値で、-6 が約 1/64 秒、小さいほど暗い。
#   当日その場で -4 から下げていき、「LED以外に白飛びが無い」ところで止める。
#   暗くしすぎると LED が小さく写って bright_min_area_px を割る。
#
# ★ キャリブレーション (5点クリック) は明るい状態で先に済ませること。
#   暗くすると隅や支柱が見えなくなってクリックできない。
#
# ★ 2026-09-15: None -> -6。Camera1 を直接開いて実測した結果:
#     自動 (-4): 16fps / 常に白飛びしている画素 7320 / 画面平均輝度 84
#     -5       : 30fps / 1521 / 49
#     -6       : 30fps /  715 / 24
#     -7       : 30fps /  498 / 11
#   None だと自動露出が室内で選んだ -4 のまま固定され、16fps + 長露出で
#   点滅の振幅が潰れ、地上で静止した機体でも Camera1 が検知を落とし続けた。
#
# ★ 2026-09-16: -6 -> -5。実測フィールド (6m x 9m) の対角奥 (Camera1から
#   約10〜14m) で Camera1 だけ検知できず、Camera2 (RPi, 自動露出のまま)
#   はできる件を調査 (camera_server.py 2026-09-16のコメント参照)。
#   LEDの実光量は距離の2乗で落ちるため、-6 まで絞ると近距離でしか
#   bright_threshold(230) を超えなくなっていた。-5 で約2倍明るくして
#   奥まで届かせる。近距離で拾うクラッタが増える分は、static bright mask
#   と 6Hz 点滅ロックイン (core/blink.py) 側で弾く前提
#   (= 「点滅で確定する」を最終フィルタとして信用する)。
#   これでも奥が拾えない/近距離の誤検知が増えすぎるなら次は -4 を試すこと。
TRACKING_EXPOSURE = -5

# α6400 は USB 出力だと OpenCV から露出制御できない場合が多い。
# その場合はカメラ本体を M モード + MF に設定しておくこと（準備時間は増えない）。

# ==========================================
# シリアル通信設定 (飛行コントローラへの送信)
# 使用しない場合は SERIAL_ENABLED = False にしてください
# ==========================================
SERIAL_ENABLED = False
SERIAL_PORT    = "COM7"
SERIAL_BAUD    = 115200

# ==========================================
# フィールド寸法
# ==========================================
# 座標系: フィールド中央が原点 (0, 0, 0)、床が z=0。
#   x = 幅方向   (-FIELD_W/2 〜 +FIELD_W/2)
#   y = 奥行方向 (-FIELD_D/2 〜 +FIELD_D/2)
#   z = 高さ（上が正）
# カメラ2台は手前側 (y = -FIELD_D/2 の辺) の2隅に設置する。
FIELD_PROFILE = "large"            # "small" / "middle" / "large"
FIELD_PROFILES = {
    "small":  (1.4, 1.4),           # 一時運用: 1.4m x 1.4m
    "middle": (1.8, 2.6),           # 横1.8m x 奥行2.6m
    "large":  (6.0, 9.0),           # 通常運用: 横6m x 縦9m (2026-09-16 実測フィールドに変更)
}
FIELD_W, FIELD_D = FIELD_PROFILES[FIELD_PROFILE]

_HW = FIELD_W / 2.0   # 13.0
_HD = FIELD_D / 2.0   # 21.0

# ==========================================
# キャリブレーション基準5点 (3D座標)
# ==========================================
# ★ 当日、会場の画角を見てから切り替えられるようプリセット化してある。
#    CALIB_PRESET を書き換えるだけでよい。
#
# クリック順序は全プリセット共通:
#     1) 手前左  2) 手前右  3) 奥右  4) 奥左  5) 4番(奥左)の点の真上（高さ CALIB_POLE_H）
#   ★ 2026-09-16: 一時的に「手前右→手前左」始まりにしていたが、元の
#     「手前左→手前右」始まりに戻した。順序を変えたら FIELD_POINT_COORDS /
#     CALIB_POINT_LABELS / _make_preset / calibration_flow.py の案内文を全部そろえること。
#
# 【5点目について】
#   4点が床の同一平面上にあるため、カメラの姿勢とスケールを決めているのは
#   実質この5点目だけ。ここが数pxずれると全体が傾く。最重要の点。
#   ・目印になる垂直構造物（ネット支柱・ゴールポスト・壁のライン）に合わせるか、
#     既知の長さの棒を立てること。空中の何もない点をクリックしてはいけない。
#   ・棒は「遠い隅」より「近い隅」に立てた方が精度が出る。
#     高さ点は実際に測れる高さの棒を使うこと。
#     近い隅が画角に入るなら CALIB_POLE_AT_NEAR = True にする。

CALIB_POLE_H       = 0.2     # 参考値。実際の5点目のz座標に合わせる。
CALIB_POLE_AT_NEAR = False   # True: 手前左(1番)の真上 / False: 奥左(4番)の真上


def _make_preset(half_w: float, half_d: float, pole_h: float, pole_near: bool):
    """4隅 + 高さ点 の5点を生成する。"""
    p1 = [-half_w, -half_d, 0.0]   # 1: 手前左
    p2 = [ half_w, -half_d, 0.0]   # 2: 手前右
    p3 = [ half_w,  half_d, 0.0]   # 3: 奥右
    p4 = [-half_w,  half_d, 0.0]   # 4: 奥左
    base = p1 if pole_near else p4
    p5 = [base[0], base[1], pole_h]
    return np.array([p1, p2, p3, p4, p5], dtype=np.float32)


CALIB_PRESETS = {
    # --- フィールド4隅。最も精度が出るが、水平画角90°近くが必要 ---
    #     (角から対角に6m下がれば約72°で収まる。config下部の画角メモ参照)
    "corners":   _make_preset(_HW, _HD, CALIB_POLE_H, CALIB_POLE_AT_NEAR),

    # --- 画角に4隅が入らない場合。フィールドを縮めた相似矩形 ---
    #     隅にこだわる必要はない。大事なのは画像内で広く散っていること。
    "inner_85":  _make_preset(_HW * 0.85, _HD * 0.85, CALIB_POLE_H, CALIB_POLE_AT_NEAR),
    "inner_70":  _make_preset(_HW * 0.70, _HD * 0.70, CALIB_POLE_H, CALIB_POLE_AT_NEAR),

    # --- 会場のコートライン（寸法が規格で決まっているので座標が正確に分かる）---
    #     ※中央寄りに点が集中するため外部パラメータの精度は落ちる。最後の手段。
    "basketball": _make_preset(15.0 / 2, 28.0 / 2, CALIB_POLE_H, CALIB_POLE_AT_NEAR),
    "volleyball": _make_preset( 9.0 / 2, 18.0 / 2, CALIB_POLE_H, CALIB_POLE_AT_NEAR),

    # --- 当日その場で実測した任意の5点を直接書く場合はここを編集 ---
    # 順序: 手前左, 手前右, 奥右, 奥左, 高さ点
    "custom": np.array([
        [-13.0, -21.0, 0.0],
        [ 13.0, -21.0, 0.0],
        [ 13.0,  21.0, 0.0],
        [-13.0,  21.0, 0.0],
        [-13.0,  21.0, 0.2],
    ], dtype=np.float32),
}

# ★ 5点の座標は、使用するプロファイルのリストを直接編集する。
#    順序: 手前左, 手前右, 奥右, 奥左, 高さ点 (奥左の真上)
#    ★ 4隅の x/y は FIELD_PROFILES の寸法と一致していないと起動時に止まる
#      (下のチェック)。寸法を変えたらここも直す。5点目の z は当日立てる
#      棒の実測高さ。
FIELD_POINT_COORDS = {
    "small": [
        [-0.7, -0.7, 0.0],
        [ 0.7, -0.7, 0.0],
        [ 0.7,  0.7, 0.0],
        [-0.7,  0.7, 0.0],
        [-0.7,  0.7, 0.2],
    ],
    "middle": [
        [-0.9, -1.3, 0.0],
        [ 0.9, -1.3, 0.0],
        [ 0.9,  1.3, 0.0],
        [-0.9,  1.3, 0.0],
        [-0.9,  1.3, 0.82],
    ],
    # 2026-09-16: 6m x 9m。高さ点は奥左の 2.0m 上。
    "large": [
        [-3.0, -4.5, 0.0],
        [ 3.0, -4.5, 0.0],
        [ 3.0,  4.5, 0.0],
        [-3.0,  4.5, 0.0],
        [-3.0,  4.5, 2.0],
    ],
}

CALIB_PRESET = f"{FIELD_PROFILE}_coordinates"

FIELD_POINTS = np.array(FIELD_POINT_COORDS[FIELD_PROFILE], dtype=np.float32)

# 4隅の座標が FIELD_PROFILES の寸法と食い違っていたら起動時に止める。
# (寸法だけ直して基準点を直し忘れる事故の再発防止。上の 2026-09-16 参照)
for _i, (_px, _py) in enumerate(((-_HW, -_HD), (_HW, -_HD), (_HW, _HD), (-_HW, _HD))):
    if abs(FIELD_POINTS[_i][0] - _px) > 1e-3 or abs(FIELD_POINTS[_i][1] - _py) > 1e-3:
        raise ValueError(
            f"[config] FIELD_POINT_COORDS['{FIELD_PROFILE}'] の {_i + 1} 点目 "
            f"({FIELD_POINTS[_i][0]:+.2f}, {FIELD_POINTS[_i][1]:+.2f}) が "
            f"FIELD_PROFILES の寸法 ({_px:+.2f}, {_py:+.2f}) と一致しません。"
            "フィールド寸法を変えたら基準5点も直してください。")

# クリック時に画面へ表示するラベル（順序ミスを防ぐ）
CALIB_POINT_LABELS = [
    "1: 手前左 (near-left)",
    "2: 手前右 (near-right)",
    "3: 奥右   (far-right)",
    "4: 奥左   (far-left)",
    f"5: 4番(奥左)の真上 {float(FIELD_POINTS[4][2]):.1f}m",
]

# ==========================================
# キャリブレーション品質の判定しきい値
# ==========================================
# 再投影誤差 [px]: solvePnP の答えで3D点を画像に投影し直し、
#                  クリック位置とどれだけずれたかを測ったもの。
#                  大きい場合はクリック順序ミス／座標入力ミスをまず疑う。
REPROJ_WARN_PX = 2.0    # これを超えたら黄色警告（精度は期待できない）
REPROJ_FAIL_PX = 5.0    # これを超えたらやり直し推奨（赤）

# 両カメラで同じ5点を三角測量して config の3D座標と比べたときの誤差 [m]。
# 再投影誤差は各カメラ内部の辻褄しか見ないが、こちらは2台の相対関係を検証する。
TRIANG_WARN_M = 0.30
TRIANG_FAIL_M = 1.00

# ==========================================
# 内部パラメータ (K, distCoeffs)
# ==========================================
# calib/intrinsics_<label>.json に、tools/calibrate_intrinsics.py で
# 事前に測定した値を保存しておく。当日は読み込むだけ（撮影は不要）。
#
# ファイルが無い場合は焦点距離を画角から概算するが、これはあくまで暫定値。
# 必ず事前にチェッカーボードで実測すること。
USE_MEASURED_INTRINSICS = True

# 実測値が無い場合のフォールバック用の公称水平画角 [度]
# （approx_camera_matrix() が使用。従来は focal=width 決め打ちで
#   水平画角53°相当という実機とかけ離れた値だった）
#
# ★ カメラの割り当て（間違えやすいので明記）
#     Camera1 = ラップトップ直結 (CAMERA_1_URL)  → Sony α6400 + 16mm
#     Camera2 = ラズパイ経由    (RPI_HOST)      → Logicool C920
#   Camera2 の内部パラメータも「PCに直結して」測定してよい。
#   読み込むのはPC側 (core/remote_camera.py) なので、
#   同じ解像度で撮れていれば経路は問わない。
FALLBACK_HFOV_DEG = {
    "Camera1": 72.9,   # Sony α6400 + 16mm  ※2026-08-16 実測値（公称72.6°とほぼ一致）
    "Camera2": 78.2,   # Logicool C920      ※2026-08-16 実測値
                       #   「90°」は対角の公称値で、水平は78°前後だった
}
FALLBACK_HFOV_DEFAULT = 78.0

# ==========================================
# 【メモ】画角と設置位置
# ==========================================
# フィールド42m×26mの4隅を1台で収めるのに必要な水平画角:
#     角ちょうど          90.0°
#     対角に3m後退        約81°
#     対角に6m後退        約72.3°   ← α6400+16mm (72.6°) がぎりぎり収まる
#     対角に8m後退        約67.7°   （マージン5°）
# 後退方向は「角から対角線方向」が最も効率的。
# 参考: 奥行方向のみなら10m、横方向のみなら15m、手前辺の中央だと20m必要。
# 光軸は対角線方向（角から約47°）に向ける。

# ==========================================
# 位置推定フィルタ
# ==========================================
# 2本のレイの最近接距離がこの値を超えたら外れ値として破棄 [m]
# ★ これは「誤検知ペアを弾く主力のゲート」。フィールドより大きい値にすると
#    窓の反射と機体を組ませたペアまで通ってしまい、ゲートとして機能しない。
#    キャリブ完了時に表示される三角測量誤差より少し大きい値にすること。
MAX_RESIDUAL_BY_PROFILE = {
    "small":  0.40,
    "middle": 0.60,
    "large":  2.00,
}
MAX_RESIDUAL_M = MAX_RESIDUAL_BY_PROFILE[FIELD_PROFILE]

# ==========================================
# 3D空間ゲート (Phase B)
# ==========================================
# 三角測量した候補ペアが物理的にありえる位置にあるかを判定する。
# 審査員席はフィールド外(y > FIELD_D/2)にあるため、正しくペアリングされれば
# ここで自動的に落ちる。誤ったペアリングは residual が跳ね上がって落ちる。
# 余裕はフィールド寸法に比例させる。1.8m四方の室内で±2mも余裕をとると
# 窓や壁がゲート内に入ってしまい、ゲートとして機能しない。
GATE_MARGIN_M = min(2.0, max(0.3, 0.3 * min(FIELD_W, FIELD_D)))
GATE_X = (-_HW - GATE_MARGIN_M, _HW + GATE_MARGIN_M)
GATE_Y = (-_HD - GATE_MARGIN_M, _HD + GATE_MARGIN_M)
# 高度 [m]。下限は MISSION_TAKEOFF_ALT_M より必ず低くすること
# （0.5m ホバリングを 0.5m 下限で切ると機体そのものが弾かれる）。
# ★ 下限を床より上 (旧 0.15m) にすると、離陸前に床に置いた機体の LED
#   (高さ数cm) が「フィールド外」で必ず棄却され、位置が一切出ない
#   (2026-09-15、両カメラで点滅確認済みなのに 3263 フレーム全棄却)。
#   床面の鏡像反射は z≈-高度 に三角測量されるので、-0.10m でも浮上後は落ちる。
# ★ 2026-09-16: この修正は long/large 判定を "!= large" に限定していたため、
#   FIELD_PROFILE="large" (このプロファイルを6m×9mの実測フィールドに転用)
#   では旧 0.5m 下限のままで再発した。プロファイルに関係なく同じ下限にする。
GATE_Z = (-0.10, 10.0)

# ==========================================
# ジオフェンス (ミッションが「逸脱した」と判断する境界)
# ==========================================
# ★ 3Dゲート (GATE_*) と必ずセットで考えること。両者を別々の場所で
#   別々に計算していたせいで、2026-09-14 に次の事故が起きた:
#     ゲート ±1.84m / フェンス ±1.5m → その隙間 (1.5〜1.84m) に落ちた
#     誤検知は「追跡としては成立するが、ミッションは即中断」になる。
#     地上に置いたままの機体が1フレームの誤検知で離陸前に中断された。
#
# 不変条件: フェンスはゲートより内側であること。
#   ゲートの外は検知そのものが棄却されるので、フェンスを外に置くと
#   永久に発火しない (代わりに「自己位置ロスト」で降りることになり、
#   ログに残る理由が実態とずれる)。
MISSION_FENCE_X = _HW + 0.2
MISSION_FENCE_Y = _HD + 0.2
# ★ 上昇旋回の到達高度 (COMP_CLIMB_ALT_M = 2.2m) より十分高くすること。
#   ここを割ると機動の途中でジオフェンスが働き、自動着陸に落ちる。
# ★ 2026-09-18: 3.0 -> 4.0。上昇旋回の到達高度 (COMP_CLIMB_ALT_M=3.6) より高くないと
#   機動の途中でジオフェンスが働いて着陸に落ちる。機体側の測距上限 RANGE_MAX_M=4.2 未満。
MISSION_FENCE_Z = 4.0

# 逸脱がこの秒数続いたら中断する。1フレームの誤検知で落とさないため。
#  ★ 代償: 本物の逸脱にはこの秒数だけ反応が遅れる。指令速度の上限が
#    0.4m/s なので、0.5秒なら最大 0.2m 余分に出る。フェンスの外に
#    それだけの余地があることを確認してから伸ばすこと。
MISSION_FENCE_GRACE_S = 0.5

for _n, _f, _g in (("X", MISSION_FENCE_X, GATE_X[1]),
                   ("Y", MISSION_FENCE_Y, GATE_Y[1]),
                   ("Z", MISSION_FENCE_Z, GATE_Z[1])):
    if _f > _g:
        print(f"[config] [WARN] ジオフェンス{_n} ({_f:.2f}m) が 3Dゲート "
              f"({_g:.2f}m) より外にあります。この軸のフェンスは発火しません。")

# ==========================================
# 高度センサ (機体の下向き距離センサ)
# ==========================================
# 単純な「4m以下はセンサ / 4m超はカメラ」の切り替えは、
#   (1) 範囲外なのかセンサ故障なのか区別できない
#   (2) 切り替えの瞬間に z が跳び、境界付近で毎フレーム切り替わる
# ため、カルマンフィルタの観測として両方入れて重み付けさせる。
ALT_SENSOR_ENABLED   = True
ALT_SENSOR_MAX_M     = 4.0    # センサの測定上限
ALT_VALID_MIN_M      = 0.2    # これ未満は無効
ALT_VALID_MAX_M      = 3.8    # 上限にマージンをとった有効範囲の上端
ALT_RECOVER_MAX_M    = 3.5    # 一度無効になった後、復帰を許す値（ヒステリシス）
ALT_CAMERA_AGREE_M   = 1.5    # カメラのzとこれ以上食い違ったらセンサを信じない
ALT_SENSOR_SIGMA_M   = 0.05   # センサ有効時の観測ノイズ（カメラの1/10）
ALT_CAMERA_SIGMA_M   = 0.50   # カメラ由来 z の観測ノイズ
ALT_TILT_COMPENSATE  = True   # 実高度 = 測定値 × cos(roll) × cos(pitch)

# ==========================================
# トラッキング (Phase C)
# ==========================================
TRACK_CONFIRM_M       = 3      # 確立に必要な検知数 (M-of-N の M)
TRACK_CONFIRM_N       = 5      # 直近Nフレーム中     (M-of-N の N)
TRACK_COAST_SEC       = 1.5    # 検知が途切れても予測で維持する時間 [s]
                               # ドローンは急に動けないので長めでよい
TRACK_MAX_SPEED_MPS   = 15.0   # ゲート半径の算出に使う最大速度 [m/s]
TRACK_TEMPLATE_PX     = 24     # 外観相関に使うテンプレートの一辺 [px]
TRACK_SEARCH_PX       = 40     # 予測位置まわりの探索半径 [px]
TRACK_MATCH_MIN_SCORE = 0.55   # 正規化相関のしきい値

# ==========================================
# ヨー方位推定 (YAW_HANDOFF.md)
# ==========================================
# クアッドのIMUは6軸（磁気センサなし）でヨーが絶対方位を持たない。
# カメラが測るフィールド座標系のΔvと、機体が測る機体座標系のΔvを
# 照合してヨーを求め、機体へ返す。
#
# 【座標系の定義 — YAW_HANDOFF.md §8 の確定事項】
#
#   フィールド座標系: x=幅(右), y=奥行(奥), z=上。右手系 (x × y = +z)
#   機体座標系      : FRD (前・右・下)。ヨーは「右へ首を振るのが +」
#                     (flight_controller/include/quad/QuadConfig.h §1)
#
#   ヨー角の定義:  psi = atan2(nx, ny)     n = 機首方向の水平単位ベクトル
#
#     psi =   0 deg → 機首がフィールド奥 (+y) を向いている
#     psi = +90 deg → 機首がフィールド右 (+x)
#     psi = 180 deg → 機首が手前 (-y、カメラ側)
#
#   +y を北とみなした方位角そのもの。右回り（上から見て時計回り）が正で、
#   機体側の「右へ首を振るのが +」と符号が一致する。
#   ★ atan2(y, x) ではなく atan2(x, y) であることに注意。逆にすると
#     回転方向が反転し、位置制御ループが正帰還になる。
YAW_ENABLED = True

# 機体から届くΔvの座標系: FRD の (前, 右)。
# IMUは FLU を返すので、機体側で FRD に変換して送ること（機体側の責務）。

# 初期アラインメント: アーム時に機体を既知方向へ向けて置き、その方位を与える。
# これによりカメラ側は「絶対値をゼロから探す」必要がなくなり、
# 90度ずれて飛んでいくリスクがほぼ消える。
# ★ 2026-09-17: 0 (フィールド奥 +y) -> 90 (フィールド右 +x)。
#   8の字は機首の左右に円を2つ並べるので、**機首に直交する向きに 4R** 要る。
#   R=1.5m なら 6.0m。機首が +y だと 4R は x 方向 = 幅 6m ちょうどで、
#   ドリフトぶんだけ確実にはみ出す。機首を +x にすれば 4R は y 方向 (9m) に
#   取れて収まる (機体側フェンス y±4.2 に対して ±3.0)。
#   → **機首をフィールド右 (+x) へ向けて置いてアームする**。
#   起動時に core/program.check_geometry() が実寸で検算し、はみ出すなら警告する。
YAW_INITIAL_ALIGN_DEG  = 90.0   # 90 = 機首をフィールド右(+x)へ向けて置く
YAW_SEND_INITIAL_ALIGN = True   # 起動時に初期方位を機体へ送るか

# 速度・Δvの窓
YAW_VEL_WINDOW_S   = 1.0    # 速度を最小二乗で求める窓 [s]
YAW_DV_WINDOW_S    = 1.0    # Δvを取る間隔 [s]
YAW_MEASURE_HZ     = 5.0    # PC内部で照合を試みる頻度 [Hz]
YAW_SEND_HZ        = 1.0    # 機体へ送る頻度 [Hz]（速くしても通信ジッタが乗るだけ）

# ゲート（これを満たさない窓は捨てる）
#  ★ 2026-09-15: 元は 1.0 だったが、MISSION_YAW_MODE="fixed" 運用での
#    巡航速度上限 (mission.py MAX_VEL=0.4m/s) 以下では窓内Δvが1.0m/sへ
#    絶対に届かず、ヨー推定が一度も収束しない状態だった (全飛行ログで
#    Yaw_Src=camera が0件だったことで確認済み)。0.4m/s巡航でも届く値へ
#    下げる。ノイズとの分離は YAW_DV_RATIO_TOL と YAW_CONFIRM_N/
#    YAW_CONFIRM_TOL_DEG (連続5回±20度一致) 側で担保する。
YAW_MIN_DV_CAMERA    = 0.2   # カメラ側Δvの下限 [m/s]。小さいと方向が純ノイズ
YAW_MIN_DV_BODY      = 0.2   # 機体側Δvの下限 [m/s]
YAW_DV_RATIO_TOL     = 0.5   # 大きさの食い違い許容比。超えたらどちらか異常
YAW_MAX_TURN_DEG     = 20.0  # 窓の間に機体がこれ以上首を振っていたら捨てる
                             # （機体座標系が回ると1秒ぶんのΔv合成が無意味になる）

# 収束判定
YAW_AVG_COUNT      = 10     # 円形平均に使う直近測定数
YAW_CONFIRM_N      = 5      # 連続何回一致したら yaw_valid=1 を立てるか
YAW_CONFIRM_TOL_DEG = 20.0  # 「一致」とみなす角度差 [deg]
YAW_HOLD_SEC       = 5.0    # 最後の有効測定からこの時間を過ぎたら valid を落とす

# 進入フェーズ（競技開始直後の直進機動）
# 純前進なので dvy_body ≈ 0 とみなせ、カメラΔvの方位がそのまま機首方位になる。
# S/Nが高くゲートを確実に通るため、最初の絶対ヨー確定に最適。
YAW_ENTRY_MODE_ENABLED = True
YAW_ENTRY_MAX_DVY_BODY = 0.4   # |dvy_body| がこれ未満なら「ほぼ純前進」とみなす [m/s]

# ==========================================
# 地上局リンク（機体との双方向通信: 操縦指令 + テレメトリ）
# ==========================================
# ★ 2026-09-17: IM920 から BLE へ移した。どちらを使うかはここ 1 行で決まる。
#   "ble"   : core/ble_link.py。log_recorder (XIAO ESP32C3) と BLE でつなぐ。
#             機体125Hzログ (下の BLE_LOG_*) と同じ BLE 接続を共有する。
#   "im920" : core/s5_link.py。地上局 XIAO (ground_receiver の env:xiao_s5_log)
#             を USB で開き、IM920 経由で機体とつなぐ (旧経路)。
# ★ IM920 に戻すときは機体側もそろえること:
#   flight_controller/include/quad/S5Features.h の GROUND_LINK = GroundLink::IM920
#   にして焼き直す (片方だけ変えると、機体は指令を 1 つも受け取らない =
#   GUIDED に入らない。飛行中なら 1 秒でホールド -> 4 秒で自動着陸)。
GROUND_LINK_BACKEND = "ble"
#
# ★ im920 のとき: ground_receiver/tools/s5_logger.py を同時に起動しないこと。
#   USB シリアルは1プロセスしか開けない。テレメトリの CSV 保存は
#   S5Link 側がやる（logs/s5_link_*.csv）。BLE でも同じ名前・同じ列で残る。
GROUND_LINK_ENABLED = True
GROUND_LINK_PORT    = None    # im920 のみ。None = VID:PID (303A:1001 / 旧RP2040 2E8A:000A) で自動検出

# ==========================================
# core/ble_tap.py が log_recorder (XIAO ESP32C3, BLE) から機体125Hzログを
# 受けて LOGnnnn.BIN に保存する。
#
# ★ bleak が未導入 / log_recorder の電源が入っていないだけなら、
#   黙って諦めて追跡・ミッションはそのまま続行する (BleTap.start() 参照)。
#   「付けっぱなしで害がない」設計なので、既定で有効にしてある。
# ★ GROUND_LINK_BACKEND = "ble" のときは、ここが False でも地上局リンクの
#   ために BLE 接続は張る (その場合も .BIN は残る。BleTap は常に保存するため)。
BLE_LOG_ENABLED = True
BLE_LOG_NAME    = "S5-LogBLE"  # log_recorder/src/main.cpp の BLE_DEVICE_NAME と一致させる

# ==========================================
# ウェイポイント飛行ミッション
# ==========================================
# フィールド座標 [m]（x=右, y=奥, z=上。中央が原点）。
# ★ FIELD_PROFILE と必ず整合させること。small (1.4m x 1.4m) のまま
#   large 用の座標を入れると、離陸した瞬間にジオフェンスへ突っ込む。
#
# 高さは「対地高度」として機体へ送る。床が平らである前提。
# ★ flight_controller の Quad::ALT_TARGET_M と同じ値にしておくこと。
#   GUIDED から抜けた瞬間 (スイッチ操作・リンク断・スティック介入) に
#   機体は POSHOLD へ落ち、そこでは ALT_TARGET_M を保持高度にする。
#   値が違うと、抜けた瞬間にその差だけ勝手に昇降する。
MISSION_TAKEOFF_ALT_M = 0.50   # 離陸してまず保持する高度 [m]


# ---- ミッション中の機首方位をどこから取るか ----------------------------
#  "fixed"  : カメラのヨー推定が収束していない間は YAW_INITIAL_ALIGN_DEG を
#             「正しい機首方位」として使う。収束したらカメラ側を優先する。
#             ★ 運用: 機首をフィールド奥(+y)へ向けて置いてアームし、
#               ミッション中は機首を回さない。これが前提。
#  "camera" : カメラのヨー推定が収束するまで水平移動しない (最も安全)。
#
#  ★ "camera" のままだと、ヨー推定は「機体が 1m/s 以上で動いている窓」が
#    無いと収束しない (YAW_MIN_DV_CAMERA)。つまり
#      動かない -> ヨーが決まらない -> 動かせない
#    のデッドロックになり、ウェイポイント飛行が永久に始まらない。
#    最初の正方形飛行は "fixed" で通し、ヨーの自動補正はそのあと。
MISSION_YAW_MODE = "fixed"

# カメラΔvのヨー推定を採用してよい、機体ジャイロヨーとの食い違いの上限 [deg]。
#  2026-09-15 20:49 に推定が 86deg ずれた値へ収束し、機体のヨーを書き換えて
#  機首を回し続けた。これを超える推定は捨てて機体ヨーで飛ぶ。
YAW_CAMERA_MAX_DISAGREE_DEG = 20.0

# "fixed" のまま (=カメラのヨー推定が未収束のまま) 巡航しているあいだの
# 速度上限 [m/s]。通常の MAX_VEL (mission.py, 0.4) より絞ることで、
# 初期アラインメントがズレていた場合の被害を抑えつつ、YawEstimator が
# 収束に必要なΔvをこの間に稼ぐ。収束して yaw_src="camera" になった
# 瞬間から通常の MAX_VEL に戻る。
MISSION_YAW_PROBE_VEL = 0.15

# ---- ミッションの自動開始 ---------------------------------------------
#  True : 機体が **アームされた瞬間** (THR_CUT 解除の立ち上がりエッジ) に
#         ミッションを待機状態 (Phase.ARMING) にする。以降は PC が REQ_HOLD を
#         送り続けるので、パイロットが SW_HOVER を GUIDED (up) に上げて
#         スロットルを 15% 以上にした瞬間に機体が GUIDED へ入り、離陸が始まる。
#         → 本番の手順から「[M] を押す」が消える。
#  False: [M] キーを押すまで何も送らない (段階確認・ベンチ用)。
#
#  ★ 2026-09-17 修正: 以前は「GUIDED フラグの立ち上がりエッジ」で start() して
#    いたが、これは原理的に発火しなかった。機体が GUIDED に入るには
#    「新鮮な上りコマンドが届いていること」が要る (quad/Guided.h の初回
#    エンゲージ条件) のに、PC は mission が IDLE の間 何も送らない。
#      PC が送らない -> 機体は GUIDED に入れない -> エッジが立たない -> PC は送らない
#    という止まり方で、結局毎回 [M] を押す必要があった。
#    アームのエッジで待機に入れば、PC は REQ_HOLD を送り始め、機体はスイッチを
#    上げた瞬間に GUIDED へ入れる。
#
#  ★ 安全: 待機中に送るのは REQ_HOLD (水平速度 0・現在高度の保持) だけ。
#    機体側は未アーム / SW_HOVER が下 / フロー不良 / スロットル 15% 未満の
#    どれかがあれば GUIDED に入らない (quad/Guided.h)。地上に置いたままの機体が
#    勝手に動くことはない。離陸の号令はあくまで「スイッチとスロットル」。
#  ★ 着陸後にもう一度飛ばすには、THR_CUT でディスアーム -> 再アーム。
#    (アームしっぱなしで再開すると、着陸直後にまた離陸してしまうため)
MISSION_AUTOSTART = True

# ---- 機体の自己位置に対するカメラ補正 ----------------------------------
#  床の模様が薄いとオプティカルフローが流れやすく、機体単独の位置推定
#  (フロー積分) が真値からじわじわずれる。カメラは絶対位置 (cm オーダー)
#  を持っているので、数秒に1回だけ機体の pos_n/pos_e を上書きして流れを消す。
#
#  ★ 姿勢・速度のクローズドループには絶対に混ぜない。IM920sL は往復
#    100〜200ms 遅れる。この遅れを含んだ位置を毎ループ (100Hz) 使うと
#    発振する。「たまに位置をこっそり書き換えるだけ」に留めること。
#    実際に効くのは静止保持中だけ (GUIDED 巡航中は機体側の hold が
#    毎ループ pos に追従するので実質無効。core/PosHold.h 参照)。
POS_CORR_ENABLED    = True
POS_CORR_PERIOD_S   = 2.0    # 補正を送る間隔 [s]
# 1回の補正で動かしてよい最大量 [m]。機体側の速度クランプ(FLOW_POS_VEL_LIM)
# は「反応の速さ」を絞るだけで、飛んでくる位置の値そのものは絞らない。
# 窓の反射のような、ゲートを1回だけすり抜けた誤検知1発で機体を
# 大きく動かさないための、PC側の最後の蓋。
POS_CORR_MAX_STEP_M = 0.4

# ---- 持続性チェック (2026-09-14 追加) -----------------------------------
# ★ 事故: align直後は0.02mだったズレが、次のチェックでは1.22mに見えた
#   ことがあった (窓の反射などゲートを1回だけすり抜けた誤検知1点)。
#   MAX_STEP_M でクランプはしていたが、"信じてよいズレか" 自体は
#   見ていなかった。ここで「直近の推定から大きく変わらない値が
#   一定時間続いた」ときだけ補正を送るようにする。
#   tracker.py の PixelJumpFilter (recovery_frames) や TRACK_CONFIRM_M と
#   同じ「M-of-N で確定させる」考え方。
POS_CORR_TRACK_TOL_M = 0.15  # この範囲に収まっていれば「同じズレ」とみなす
POS_CORR_CONFIRM_S   = 1.0   # この秒数、同じズレが続いたら補正してよいと判断する


# ==========================================================================
# 本番プログラム (4分間の飛行競技)  -- core/program.py が読む
# ==========================================================================
# 飛ぶ順番と時間配分。ルールブックの連続成功倍率 (1.5^n) があるので順番は固定で、
# **着陸が最も価値が高い** (800 x 1.2 x 1.5^4 = 4860点)。したがって途中の機動を
# 1つ捨ててでも時間内に降ろす、という設計にしてある。詳細は core/program.py 冒頭。
#
#   1 滑走路内離陸  100点   2 水平旋回 1000点   3 上昇旋回 1200点
#   4 8の字飛行    1400点   5 着陸      800点
#
# ★ 3分55秒 (235s) 以降の着陸は失敗。COMP_LAND_BY_S / COMP_FORCE_LAND_BY_S の
#   2段で、何が起きていても必ず着陸へ倒す。

# ---- 速度と旋回半径 ------------------------------------------------------
# 旋回半径 R = v / omega。ルールブックは「概ね1.5m以上」。
#  ★ 2026-09-16 までの実機は v=0.4m/s・omega=15deg/s (R=1.53m) で 1周 24秒。
#    この速度だと 水平旋回2周 + 上昇旋回5周 + 8の字2周 = 216秒 かかり、
#    移動と離着陸を足すと 3分に収まらない。v を上げて 1周 11.8秒 にする。
#  ★ 旋回に必要なリーン角は atan(v^2 / (R*g))。v=0.8/R=1.5 で 2.5度 しかなく、
#    機体側の上限 FLOW_MAX_LEAN=8度 に対して十分余裕がある (速度側の制約は
#    フロー速度の上限 FLOW_VEL_SANE=3.0m/s のほうがずっと遠い)。
#  ★ 実測の旋回半径は指令より 2〜3割 大きく出る (2026-09-16: 指令0.57m→実測0.7m)。
#    1.5m 指令なら実測 1.8〜2.0m で、ルールの「1.5m以上」は満たす側に外れる。
#  ★ 機体側 Quad::GUIDED_MAX_VEL がこの値以上であること (超えるとクランプされ、
#    指令より遅く回って半径が小さくなる = ルール違反側に外れる)。
#  ★ 2026-09-18 実測で見直し (flight_controller/scripts/analyze_circle.py):
#    完走した周の実半径は指令比 -8%〜+8% (LOG0042/0044/0057)。上の「2〜3割大きい」は
#    開ループ (Maneuver.h) 時代の話で、CircleTrack のクローズドループには当てはまらない。
#    R=1.5 指令だと最悪 1.38m でルールの 1.5m を割るので 2.0 にする。
#    半径を上げても損はしない: 飛行継続ボーナスは飛行時間そのものに付くので、
#    大きい円で時間を使っても、小さい円で回って余った時間を滞空に使っても同じ。
#  ★ 速度は 0.8 -> 0.7。軌道追従の目標速度は FF(V) + 位置補正(±0.30) を
#    FLOW_STICK_VEL=1.0 でクランプするので、V=0.8 だと補正余地が +0.2 しか無く、
#    飽和すると円の内側を通って実半径が縮む。
COMP_MANEUVER_SPEED = 0.7     # 機動中の前進速度 [m/s]
#  ★ 2.0 だと機動だけで 227s かかり 235s の締切に余裕が無くなる。1.8 なら最悪 -8% でも
#    1.66m で規定を満たし、想定合計 ~207s に収まる。
COMP_TURN_RADIUS_M  = 1.8     # 指令旋回半径 [m]
COMP_TURN_RIGHT     = True    # True = 右旋回から始める (8の字の1周目も)
COMP_CIRCLE_LAPS    = 2       # 水平旋回: 連続2周で1000点 (1周だと400点)
COMP_CLIMB_LAPS     = 2       # 上昇旋回: 低高度2周 -> 上昇1周 -> 高高度2周

COMP_CRUISE_ALT_M = MISSION_TAKEOFF_ALT_M   # 巡航・水平旋回・8の字の高度 [m]
# 上昇旋回の到達高度 [m]。ルール 6.10 は「**ポールの高さ (3m) 以上**を保ったまま2周」
# なので 3m を明確に超える必要がある (機体の一部でも領域を外すと認められない)。
# 低高度側は 3m 以下でなければならないので COMP_CRUISE_ALT_M も 3m 未満に保つこと。
#  ★★ ここは必ず機体側の測距上限 (QuadConfig.h の RANGE_MAX_M) より下にすること。
#    測れない高度を目標にすると「まだ低い」と信じて上昇し続ける (LOG0064 の天井張り付き)。
#    測距が 3.6m を返せないと分かったら COMP_ENABLE_CLIMB=False にして飛ばさない。
COMP_CLIMB_ALT_M  = 3.60

# ---- 本番 / 1/10 スケール ------------------------------------------------
# 半径と速度を同じ率で縮める。omega = v / R は変わらないので **1周の時間も
# 時間配分も本番と同じまま**、場所だけ 1/10 になる = 通し練習に使える。
#  ★ 縮むのは水平方向だけ。求心加速度は 1/10 になるのでリーン角も 1/10 になり、
#    本番より優しい条件になる。**「1/10 で飛べた」は本番の保証にはならない**。
#    確認できるのは順序・遷移・タイマー・締切・中断処理といった「段取り」だけ。
COMP_SCALE = 1.0              # 1.0 = 本番 / 0.1 = 1/10 (画面の [S] キーで切替)
# 1/10 のときの高度 (天井が低い場所で回すので、比例縮小はしない)
COMP_SMALL_CRUISE_ALT_M = 0.45
COMP_SMALL_CLIMB_ALT_M  = 0.90

# 上昇旋回をやるか ([C] キーで切替)。測距が 3m 超を返せないなら False。
#  ★ 失敗しても倍率チェーンは切れない (後続が1段ずれるだけ) ので「やってみる」は
#    合理的だが、**測れない高度を目標にするのは別の話** (上の注意)。
COMP_ENABLE_CLIMB = True

# ---- 場所 ---------------------------------------------------------------
# 機動の円の中心にしたい点 (フィールド座標 [m])。中心 (0,0) が四方の壁から
# 最も遠い。機動の開始地点はここから機首の向きで自動計算する (core/program.py)。
COMP_FIELD_CENTER = (0.0, 0.0)
# 機体側ジオフェンス (flight_controller QuadConfig.h の FENCE_E_LIM / FENCE_N_LIM)。
# ★ 起動時の幾何チェックがこの値と機動の軌跡を突き合わせる。片方だけ変えないこと。
#  ★★ 2026-09-19: 本番のミッションエリアは約22m四方。旧値 (2.7 / 4.2) は
#    6m x 9m の練習場のもので、R=1.8 の 8の字 (y ±3.6m) に対して余裕が 0.6m
#    しか無かった。フローのドリフトは 5周で 0.4m 出るので、**機動の途中で
#    フェンスに押し返されて円が崩れる** = ミッション不成立になる。
#    離陸地点 (離着陸エリア②) はフィールド中心から見て端の方にあるので、
#    狭い矩形にすると **離陸直後から境界に押し返される**。
#  ★ 機体側フェンスは shiftFrame() の後だけ効く (PosHold.h fenceOn())。
#    つまりカメラで自己位置を送っているときだけ。fly_nocam.py では最初から
#    効かないので、そのときの安全はパイロットの bail-out が受け持つ。
#  ★ 機体側 QuadConfig.h の FENCE_E_LIM(=X) / FENCE_N_LIM(=Y) と必ず同じ値に。
#    (座標の対応: 機体の N = フィールド y / 機体の E = フィールド x。
#     mission.py の corr_n = diff_y / corr_e = diff_x がその定義)
COMP_FENCE_X = 10.0
COMP_FENCE_Y = 10.0

# ---- 着陸の狙い ---------------------------------------------------------
# True  = 離陸地点へ帰投してから降りる (離着陸エリア内 400点)
# False = ミッションエリアでその場着陸 (飛行競技エリア内 300点)
#  ★ 既定は False。理由 (COMPETITION_OPEN_ISSUES.md G5):
#    - 自動着陸滑走路 (800点) は反対側の離着陸エリア①で、約30m の横断が要る。
#      端から狙っていない。
#    - 帰投は「機動のあとで一番ドリフトが溜まった状態で、5m 以上を位置ループで
#      移動する」段階で、失敗したときに巻き添えにするのが **着陸 1822点** と
#      いちばん高い。差は 400-300 の素点 = 倍率込み 506点。割に合わない。
#    - カメラ無しで飛ぶ場合 (fly_nocam.py) は帰投そのものができない。
#  ★ True に戻すと帰投 GOTO が復活し、着陸の素点表示も 400 になる。
COMP_RETURN_HOME = False

# ---- ミッションエリアへの進入 (カメラが無いとき) -------------------------
# カメラで自己位置が出ているときは、進入は普通に位置ループで飛ぶ (この値は無視)。
# 位置が出ていないとき (fly_nocam.py / カメラ落ち) は、**機首方向へこの秒数だけ
# 前進する**開ループ進入に切り替える。速度は COMP_MANEUVER_SPEED。
#  ★ これが無いと「離陸 -> その場ホバー -> その場で旋回」になり、ルール 6.7 の
#    「離着陸エリア②からミッションエリアに進入すること」が成立しない
#    (= 離陸 120点 と、その後ろの倍率チェーンが全部ずれる)。
#  ★ 距離 = COMP_MANEUVER_SPEED x この秒数。0.7 x 9 = 6.3m。
#    離着陸エリア②の滑走路からミッションエリアの内側までを見て当日調整する。
#    0 にすると開ループ進入をしない (その場ホバーのまま)。
COMP_ENTRY_DR_S = 9.0

# ---- 時間配分 [s] --------------------------------------------------------
# budget = その段階に許す時間。超えたら打ち切って次へ進む (粘ると後続と着陸を
# 巻き添えにする)。想定値は 2026-09-16 までの実測 (WP移動 ~1mで6秒 / 離陸3.9秒 /
# 着陸3.6秒) と、上の速度から出した機動の所要時間から。
COMP_BUDGET_S = {
    "takeoff":    20.0,   # 想定  5s
    "goto_first": 25.0,   # 想定 12s  離陸地点 -> ミッションエリア (4〜5m)
    "circle":     45.0,   # 想定 34s  (2周 32.3s + 開始の間)
    "goto":       20.0,   # 想定  8s  機動どうしの間 (ドリフト 0.5〜1m ぶん)
    "climb":      95.0,   # 想定 83s  (5周 80.8s + 開始の間)
    "figure8":    45.0,   # 想定 34s
    "goto_home":  30.0,   # 想定 12s  ミッションエリア -> 離着陸エリア
                          #  (COMP_RETURN_HOME=True のときだけ使う)
    # 機動が全部終わったあと、着陸の締切まで **その場でホバーし続ける** 段階。
    #  ★ 飛行継続ボーナス = 飛行時間 x2 x 成功ミッション数。5ミッションなら
    #    10点/秒。機動が予定より早く終わったぶんをここで使い切る。
    #    実際に終わるのは COMP_LAND_BY_S の締切なので、budget は「そこまで
    #    絶対に届く長さ」にしておけばよい。
    "loiter":    240.0,
    "land":       40.0,   # 想定 11s  (降下 6s + 静止判定 5s)
}
# ★ 2026-09-18: 飛行継続ボーナス (飛行時間 x2 x ミッション数 = 5ミッションなら 10点/秒)
#   があるので、早く降りるほど損。接地は 235s まで許されるので、そこから逆算する。
COMP_TARGET_S = 215.0        # 全部終わらせたい時刻 [s] (想定合計は ~207s)
# 何をしていてもこの時刻には打ち切って着陸へ入る [s]。
#  ★ 2026-09-19: 帰投をやめた (COMP_RETURN_HOME=False) ので、ここからは
#    「降下 6s + 静止 5s」だけ。210 + 11 = 221s で静止まで終わり、規定の
#    235s に 14s の余裕がある。早く降りるほど継続ボーナスを捨てるので、
#    余裕を食い潰さない範囲でできるだけ遅くしてある。
COMP_LAND_BY_S = 210.0
# ここを過ぎたら、何をしていても (帰投の途中でも) **その場で** 降りる [s]。
#  飛んだまま 3分55秒 を迎えると着陸点は 0 になり、後続の倍率ごと失う。
COMP_FORCE_LAND_BY_S = 218.0
# ルールブックの絶対締切 [s]。表示と警告にのみ使う。
COMP_RULE_DEADLINE_S = 235.0

COMP_SETTLE_S      = 1.0     # 機動の開始地点に着いてから静止する時間 [s]
COMP_LAND_SETTLE_S = 0.5     # 着陸地点に着いてから降下を始めるまで [s]
COMP_LAND_STILL_S  = 5.0     # 接地後に静止を確認する時間 [s] (ルール: 5秒以上)
# 静止判定でこれ以上動いたら数え直す [m]。接地の跳ね返りとカメラのノイズを
# 区別する閾値。三角測量の誤差 (TRIANG_WARN_M=0.30) より少し内側に取る。
COMP_LAND_STILL_TOL_M = 0.25

# 想定所要時間 (program_summary の表示と合計の算出に使うだけ。制限ではない)
COMP_EXPECT_TAKEOFF_S   = 5.0
COMP_EXPECT_GOTO_S      = 10.0
COMP_EXPECT_LAND_S      = 6.0
COMP_MANEUVER_START_S   = 2.0   # 機動の開始要求が機体に通るまでの間

# ---- 旧テスト用 (本番プログラムを使わないとき) ----------------------------
# COMP_ENABLED=False にすると、従来どおり MISSION_WAYPOINTS を巡回して帰投する。
# Phase 4〜6 の段階確認に戻したいときに使う。
COMP_ENABLED = True
MISSION_WAYPOINTS = [
    ( 0.5, -0.5, MISSION_TAKEOFF_ALT_M),
    ( 0.5,  0.5, MISSION_TAKEOFF_ALT_M),
    (-0.5,  0.5, MISSION_TAKEOFF_ALT_M),
    (-0.5, -0.5, MISSION_TAKEOFF_ALT_M),
]
# 保持試験モード。True だと 1点目に到達したあと帰投・着陸せずその場に留まり続ける
# (到達半径の出入りと滞在秒数をミッションログのイベントに残す)。COMP_ENABLED=False
# のときだけ効く。
MISSION_HOLD_AT_FIRST_WP = False


# ==========================================
# ダミー飛行（カメラ未検出フォールバック）
# ==========================================
DUMMY_FALLBACK_FRAMES = 30    # 何フレーム連続未検出でダミーに切り替えるか
DUMMY_ORBIT_RADIUS    = 4.0   # 旋回半径 [m]
DUMMY_ORBIT_ALT       = 3.0   # 飛行高度 [m]
DUMMY_ORBIT_PERIOD    = 10.0  # 1周の時間 [s]

# ==========================================
# 3D表示の固定範囲（フィールド寸法に追従）
# ==========================================
VIEW_MARGIN_M = max(0.2, min(FIELD_W, FIELD_D) * 0.08)
VIEW_X = (-_HW - VIEW_MARGIN_M, _HW + VIEW_MARGIN_M)
VIEW_Y = (-_HD - VIEW_MARGIN_M, _HD + VIEW_MARGIN_M)
VIEW_Z = (  0.0, 12.0)

# ==========================================
# 表示ウィンドウサイズ
# ==========================================
# 2880x1920 ディスプレイ向けデフォルト値。
# カメラウィンドウ2枚＋ダッシュボードが並ぶように調整してください。
DISP_W = 1280
DISP_H =  720

VELOCITY_W = 1000
VELOCITY_H =  500

# 速度推定フレーム数（大きいほど滑らか、小さいほど即応性が高い）
VELOCITY_SMOOTH_FRAMES = 5

# ==========================================
# 検知パラメータ（detection_params.json から読み込み）
# ==========================================
# ★ PC側とRPi側 (camera_server.py) が同じファイルを読む。
#    以前は両者に別々の値がハードコードされていてずれていた。
_DETECTION_JSON = PROJECT_DIR / "detection_params.json"

_DEFAULT_DETECTION = {
    "diff_threshold": 12,
    "min_area_px": 40,
    "max_area_px": 20000,
    "blur_kernel": 5,
    "morph_kernel": 5,
    "max_candidates": 8,
    "use_background_subtractor": True,
    "bg_history": 500,
    "bg_var_threshold": 24.0,
    "bg_learning_rate": 0.0005,
    "vibration_reject_ratio": 0.06,
    # 検知方式: "motion" / "bright" / "bright_or_motion"
    "detect_mode": "motion",
    "bright_threshold": 230,
    "bright_min_area_px": 4,
    "bright_max_area_px": 4000,
    "static_bright_mask": True,
    "static_mask_learn_frames": 120,
    "static_mask_ratio": 0.9,
    "static_mask_dilate_px": 7,
    "flicker_window_sec": 0.4,
    "flicker_threshold": 50,
    "flicker_bg_alpha": 0.1,
    "flicker_bg_threshold": 20,
}


def load_detection_params(path: Path = _DETECTION_JSON) -> dict:
    """detection_params.json を読む。無い/壊れている場合は既定値で続行する。

    戻り値は共通パラメータ。カメラ別の上書き ("camera_overrides") は
    DETECTION_CAMERA_OVERRIDES に分けて持ち、detection_params_for() で合成する。
    """
    params = dict(_DEFAULT_DETECTION)
    try:
        with open(path, encoding="utf-8") as f:
            loaded = json.load(f)
        # "_comment*" キーは無視する
        params.update({k: v for k, v in loaded.items() if not k.startswith("_")})
    except FileNotFoundError:
        print(f"[config] {path.name} が見つかりません。既定値を使用します。")
    except (json.JSONDecodeError, OSError) as e:
        print(f"[config] {path.name} の読込に失敗しました（既定値を使用）: {e}")
    return params


DETECTION = load_detection_params()

# ★ カメラ別の上書き。Camera1 (α6400, 遠方の LED が数 px) と Camera2 (RPi) は
#   レンズも露出も違うので、しきい値・ぼかし・面積の係数を揃えないほうがよい
#   (2026-09-16: 10m 先の機体を Camera1 だけ検知できなかった)。
#   RPi 側 camera_server.py はこのキーを読まないので Camera2 には影響しない。
DETECTION_CAMERA_OVERRIDES = DETECTION.pop("camera_overrides", {}) or {}


def detection_params_for(label: str) -> dict:
    """共通パラメータに、そのカメラ用の上書きを重ねたものを返す。"""
    params = dict(DETECTION)
    params.update({k: v for k, v in DETECTION_CAMERA_OVERRIDES.get(label, {}).items()
                   if not k.startswith("_")})
    return params

# 既存モジュールが参照している名前（後方互換）
DIFF_THRESHOLD = DETECTION["diff_threshold"]
MIN_AREA_PX    = DETECTION["min_area_px"]
MAX_AREA_PX    = DETECTION["max_area_px"]
BLUR_KERNEL    = DETECTION["blur_kernel"]
MORPH_KERNEL   = DETECTION["morph_kernel"]
MAX_CANDIDATES = DETECTION["max_candidates"]

USE_BG_SUBTRACTOR      = DETECTION["use_background_subtractor"]
BG_HISTORY             = DETECTION["bg_history"]
BG_VAR_THRESHOLD       = DETECTION["bg_var_threshold"]
BG_LEARNING_RATE       = DETECTION["bg_learning_rate"]
VIBRATION_REJECT_RATIO = DETECTION["vibration_reject_ratio"]

# 明るいLEDを狙う検知モード (detection_params.json の _comment_mode を参照)
DETECT_MODE        = DETECTION["detect_mode"]
BRIGHT_THRESHOLD   = DETECTION["bright_threshold"]
BRIGHT_MIN_AREA_PX = DETECTION["bright_min_area_px"]
BRIGHT_MAX_AREA_PX = DETECTION["bright_max_area_px"]

# 静的輝点マスク: 窓・白い反射・照明のように「ずっと明るいまま動かない」画素を
# 学習して bright マスクから引く。学習中は機体を画角に入れないこと。
STATIC_BRIGHT_MASK       = DETECTION["static_bright_mask"]
STATIC_MASK_LEARN_FRAMES = DETECTION["static_mask_learn_frames"]
STATIC_MASK_RATIO        = DETECTION["static_mask_ratio"]
STATIC_MASK_DILATE_PX    = DETECTION["static_mask_dilate_px"]

# ==========================================
# LED点滅パターンによる候補フィルタ (core/blink.py)
# ==========================================
# カメラ検知（motion/bright）は「明るい/動いた」としか言えず、1枚の画像
# だけでは窓の反射や照明と機体を区別できない。機体のLEDを一定の周波数で
# 点滅させておき、候補が時間方向にその周波数で明滅しているかを見て、
# 点滅しない静的な明点を積極的に落とす。tracker.py が2カメラそれぞれに
# 1個ずつ BlinkTracker を持ち、PairSelector（幾何整合）より前段でかける。
#
# 判定は候補位置まわりの平均輝度に対するロックイン検波 (6Hz 成分の割合)。
# 実タイムスタンプで相関を取るので、fps が 12 未満でも位相が散っていれば検出できる。
#
# ★ 効くのは露出時間。点灯/消灯は各 83ms なので、露出がそれに近いと
#   1枚の中で平均されて振幅が消える。2026-09-15 の実測では Camera1 の
#   read() が毎回 99ms (=自動露出が 1/10s 前後まで伸びて 10fps) だった。
#   追跡時は TRACKING_EXPOSURE で 1/60s 以下に絞ること。
#   フレーム間隔が半周期を超えていると起動後に [WARN] が出る。
#
# ★ 点滅周波数は flight_controller/src/drone_s5.cpp の検出用LED
#   (millis() % 167 < 83) と一致させること。
#
# 画面上の各光点の横に「score/depth」を表示する。黄=点滅確認、灰=未確認。
BLINK_DETECT_ENABLED = True
LED_BLINK_HZ         = 6.0     # 機体LEDの点滅周波数 [Hz]
BLINK_MATCH_DIST_PX  = 30      # フレーム間で同一光点とみなす最大移動量 [px]
BLINK_ROI_PX         = 8       # 輝度を測る ROI の半径 [px] (17x17)
BLINK_HISTORY_SEC    = 1.2     # ロックインの窓 [s]。長いほど確実だが確定が遅い
BLINK_MIN_SCORE      = 0.5     # 分散のうち 6Hz 成分の割合の下限 (矩形波で約0.81)
BLINK_MIN_DEPTH      = 1.5     # ROI平均輝度の標準偏差の下限。静止光のノイズ相関を弾く

# カメラ別の上書き (キーは BlinkTracker の引数名)。
# ★ Camera1: 10m 先の LED は 1280x720 で 3px 角、ピーク輝度 200 程度しかない
#   (tests/ の 2026-09-16 16:48 画面録画で実測)。半径 8 の ROI (289px) で
#   平均すると変調が 5 階調・標準偏差 2.5 まで薄まり、min_depth=1.5 に
#   対して余裕が無い。ROI を半径 3 (49px) に絞って変調を残す (実測 depth 35)。
#   ホバリングの揺れは match_dist_px 側で追従するので ROI は小さくてよい。
#   (2026-09-16 は min_score 0.5 -> 0.4 で合わせたが、時刻の揺らぎでスコアが
#    0.3〜0.9 を行き来していた。翌日の長露出で破綻したので下のトグル判定へ移行)
#   tools/replay_detect.py で再現できる。
# ★ 2026-09-17 Camera1: α6400 が 1/15s の自動露出 (実効 10fps、間隔 33〜230ms) で
#   ロックインのスコアが LED でも 0.28 しか出ず、検知 0%。点滅の判定を時刻に
#   頼らない2値トグル判定 (core/blink.py 冒頭) に切り替える。toggle_min_hz を
#   与えると min_score / min_depth は使われない。LED は毎秒 5〜12 回切り替わり、
#   明暗差 58〜76。床の反射・照明は差 30 以下 (当初 contrast 35。下の調整で 28 に下げたが、
#   止まっている物は「2値的」「切り替わり回数」でも落ちる)。
#   tests/ の 2026-09-17 09:36:51 と 2026-09-16 16:48 の録画で確認。
# ★ 2026-09-17 10:08 Camera1: 0.8m/s 以下で動き続ける機体 (30 秒) で点滅確定 0.4%。
#   画面上 最大 ~500px/s で動き、候補が 30px 以上跳ぶとトラックが切れて、点滅の履歴
#   (0.9 秒) が溜まらなかった。動く機体向けに次を足した (core/blink.py 冒頭):
#     max_speed_px_s 400 : トラックが速度を持ち、予測位置から割り当て・輝度を測る
#     roi_peak_px 8      : 予測のずれを吸収するため、半径 8px 内の roi_px 角平均の最大値で測る
#     min_samples 6 / min_span_ratio 0.5 : 0.6 秒・6 枚から確定 (切れた後の復帰を早く)
#     toggle_min_contrast 28 : 遠い所 (明暗差 30 前後) を拾う
#     coast_sec 0.3      : 白飛びした窓の前を横切る間などは確定のまま予測位置を出す
#   録画 3 本 (tests/test_camera1_replay.py) で: 動く 70% / 静止 81.5% / 10m 静止 91%。
#   切り替わり 3.5Hz・coast 0.6s にすると動く録画は 78% まで上がるが、Camera2 の
#   手を振る人の録画で人を確定する回数が 3 → 15 に増えたので控えめな値にした。
BLINK_CAMERA_OVERRIDES = {
    "Camera1": {"roi_px": 2, "roi_peak_px": 8, "max_speed_px_s": 400,
                "toggle_min_hz": 4.5, "toggle_min_contrast": 28.0,
                "min_samples": 6, "min_span_ratio": 0.5, "coast_sec": 0.3},
}


def blink_params_for(label: str) -> dict:
    """BlinkTracker に渡すキーワード引数。共通値にカメラ別の上書きを重ねる。"""
    params = {
        "match_dist_px": BLINK_MATCH_DIST_PX,
        "roi_px": BLINK_ROI_PX,
        "history_sec": BLINK_HISTORY_SEC,
        "min_score": BLINK_MIN_SCORE,
        "min_depth": BLINK_MIN_DEPTH,
        "enabled": BLINK_DETECT_ENABLED,
    }
    params.update(BLINK_CAMERA_OVERRIDES.get(label, {}))
    return params
