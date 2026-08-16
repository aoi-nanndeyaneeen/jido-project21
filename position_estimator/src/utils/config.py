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
RPI_HOST     = "192.168.11.13"      # ラズパイIP
RPI_PORT     = 5555  # カメラ2 (ラズパイ接続)

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
FIELD_W = 26.0   # 幅   [m]
FIELD_D = 42.0   # 奥行 [m]

_HW = FIELD_W / 2.0   # 13.0
_HD = FIELD_D / 2.0   # 21.0

# ==========================================
# キャリブレーション基準5点 (3D座標)
# ==========================================
# ★ 当日、会場の画角を見てから切り替えられるようプリセット化してある。
#    CALIB_PRESET を書き換えるだけでよい。
#
# クリック順序は全プリセット共通:
#     1) 手前左  2) 手前右  3) 奥右  4) 奥左  5) 4番の点の真上（高さ CALIB_POLE_H）
#
# 【5点目について】
#   4点が床の同一平面上にあるため、カメラの姿勢とスケールを決めているのは
#   実質この5点目だけ。ここが数pxずれると全体が傾く。最重要の点。
#   ・目印になる垂直構造物（ネット支柱・ゴールポスト・壁のライン）に合わせるか、
#     既知の長さの棒を立てること。空中の何もない点をクリックしてはいけない。
#   ・棒は「遠い隅」より「近い隅」に立てた方が精度が出る。
#     2mの棒は45m先だと画面上わずか46px、15m先なら138px。3倍の分解能差。
#     近い隅が画角に入るなら CALIB_POLE_AT_NEAR = True にする。

CALIB_POLE_H       = 2.0     # 5点目の高さ [m]（持ち込む棒の長さに合わせる）
CALIB_POLE_AT_NEAR = False   # True: 手前左(1番)の真上 / False: 奥左(4番)の真上


def _make_preset(half_w: float, half_d: float, pole_h: float, pole_near: bool):
    """4隅 + 高さ点 の5点を生成する。"""
    p1 = [-half_w, -half_d, 0.0]   # 手前左
    p2 = [ half_w, -half_d, 0.0]   # 手前右
    p3 = [ half_w,  half_d, 0.0]   # 奥右
    p4 = [-half_w,  half_d, 0.0]   # 奥左
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
    "custom": np.array([
        [-13.0, -21.0, 0.0],
        [ 13.0, -21.0, 0.0],
        [ 13.0,  21.0, 0.0],
        [-13.0,  21.0, 0.0],
        [-13.0,  21.0, 2.0],
    ], dtype=np.float32),
}

# ★ 当日はここだけ書き換える
CALIB_PRESET = "corners"

FIELD_POINTS = CALIB_PRESETS[CALIB_PRESET]

# クリック時に画面へ表示するラベル（順序ミスを防ぐ）
CALIB_POINT_LABELS = [
    "1: 手前左 (near-left)",
    "2: 手前右 (near-right)",
    "3: 奥右   (far-right)",
    "4: 奥左   (far-left)",
    f"5: {'1番' if CALIB_POLE_AT_NEAR else '4番'}の真上 {CALIB_POLE_H:.1f}m",
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
FALLBACK_HFOV_DEG = {
    "Camera1": 90.0,   # Logicool C920 (90°版)
    "Camera2": 72.6,   # Sony α6400 + 16mm (APS-C)
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
# ★ 現在の 5.0 は内部パラメータが概算値だったため緩めてある。
#    チェッカーボード実測後は 1.0〜1.5 まで絞れるはず。
#    キャリブ完了時に表示される三角測量誤差を見て決めること。
MAX_RESIDUAL_M = 5.0

# ==========================================
# 3D空間ゲート (Phase B)
# ==========================================
# 三角測量した候補ペアが物理的にありえる位置にあるかを判定する。
# 審査員席はフィールド外(y > FIELD_D/2)にあるため、正しくペアリングされれば
# ここで自動的に落ちる。誤ったペアリングは residual が跳ね上がって落ちる。
GATE_MARGIN_M = 2.0                      # フィールド境界の外側にとる余裕 [m]
GATE_X = (-_HW - GATE_MARGIN_M, _HW + GATE_MARGIN_M)
GATE_Y = (-_HD - GATE_MARGIN_M, _HD + GATE_MARGIN_M)
GATE_Z = (0.5, 10.0)                     # 高度 [m]。上昇旋回で4mを超える想定

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
# ダミー飛行（カメラ未検出フォールバック）
# ==========================================
DUMMY_FALLBACK_FRAMES = 30    # 何フレーム連続未検出でダミーに切り替えるか
DUMMY_ORBIT_RADIUS    = 4.0   # 旋回半径 [m]
DUMMY_ORBIT_ALT       = 3.0   # 飛行高度 [m]
DUMMY_ORBIT_PERIOD    = 10.0  # 1周の時間 [s]

# ==========================================
# 3D表示の固定範囲（フィールド寸法に追従）
# ==========================================
VIEW_X = (-_HW - 2.0, _HW + 2.0)
VIEW_Y = (-_HD - 2.0, _HD + 2.0)
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
}


def load_detection_params(path: Path = _DETECTION_JSON) -> dict:
    """detection_params.json を読む。無い/壊れている場合は既定値で続行する。"""
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

# ==========================================
# 画像空間ジャンプフィルタ（Phase B で3Dゲートに置き換え予定）
# ==========================================
PIXEL_SPEED_LIMIT_PX_S = 1500   # 画像内で許容する最大移動速度 [px/s]
JUMP_RECOVERY_FRAMES   = 5      # 何フレーム連続で外れたら「本当の移動」と認めるか
