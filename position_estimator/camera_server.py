"""
camera_server.py  [RPi上で単独実行]

カメラサーバー。CALIB（フィールド基準点クリック）と STREAM（動体検知の配信）の
2モードを持つ。position_estimator 本体からは import されない独立プロセス。

検知パラメータは detection_params.json から読む（PC側 src/utils/config.py と
同じファイル）。以前は両者に別々の数値がハードコードされていてずれていた。
このファイルを RPi にも配置すること。
"""

import base64
import json
import socket
import time
from pathlib import Path

import cv2

HOST, PORT     = "0.0.0.0", 5555
CAMERA_ID      = 0
SEND_PREVIEW   = True
PREVIEW_SCALE  = 0.5
TARGET_WIDTH   = 1280
TARGET_HEIGHT  = 720
TARGET_FPS     = 60   # config.py の CAMERA_FPS と合わせること

# オートフォーカスは必ず切る。フォーカスが動くと焦点距離が変わり、
# 事前にチェッカーボードで測った内部パラメータ K が無効になる。
DISABLE_AUTOFOCUS = True
FOCUS_VALUE       = 0        # 0 = 無限遠
LOCK_EXPOSURE_ON_STREAM = True

DSHOW_EXPOSURE_MANUAL = 0.25

# ── 検知パラメータ（PC側と共有）────────────────────────────────
_DEFAULT_PARAMS = {
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


def load_params():
    path = Path(__file__).resolve().parent / "detection_params.json"
    params = dict(_DEFAULT_PARAMS)
    try:
        with open(path, encoding="utf-8") as f:
            loaded = json.load(f)
        params.update({k: v for k, v in loaded.items() if not k.startswith("_")})
        print(f"[RPi] 検知パラメータを読み込み: {path.name}")
    except FileNotFoundError:
        print("[RPi] detection_params.json が無いため既定値を使用します")
    except (json.JSONDecodeError, OSError) as e:
        print(f"[RPi] detection_params.json 読込失敗（既定値を使用）: {e}")
    return params


P = load_params()

# ── 共有状態（モジュールレベル） ─────────────────────────────
calib_pts    = []
shared_state = {"quit": False}


# ── 動体検知 ──────────────────────────────────────────────────
class Detector:
    """
    動体候補を「複数」返す検知器。

    候補を1個に絞らないのは、42m先だと機体も人も20px程度で
    大きさも形も区別がつかないため。どれが機体かはPC側が
    2カメラの幾何整合（三角測量の残差とフィールド内判定）で決める。
    """

    def __init__(self):
        self.blur = (P["blur_kernel"], P["blur_kernel"])
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (P["morph_kernel"], P["morph_kernel"]))
        self.prev_gray = None
        self.bg = None
        self.vibration_rejected = False
        if P["use_background_subtractor"]:
            self.bg = self._make_bg()

    def _make_bg(self):
        return cv2.createBackgroundSubtractorMOG2(
            history=P["bg_history"],
            varThreshold=P["bg_var_threshold"],
            detectShadows=False,
        )

    def reset(self):
        self.prev_gray = None
        if self.bg is not None:
            self.bg = self._make_bg()

    def _mask(self, gray_blurred):
        if self.bg is not None:
            m = self.bg.apply(gray_blurred, learningRate=P["bg_learning_rate"])
            _, m = cv2.threshold(m, 127, 255, cv2.THRESH_BINARY)
            return m

        if self.prev_gray is None:
            self.prev_gray = gray_blurred
            return None
        diff = cv2.absdiff(self.prev_gray, gray_blurred)
        _, m = cv2.threshold(diff, P["diff_threshold"], 255, cv2.THRESH_BINARY)
        self.prev_gray = gray_blurred
        return m

    def detect(self, frame):
        """[(u, v, area, x, y, w, h), ...] を面積の大きい順に返す。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, self.blur, 0)

        mask = self._mask(gray)
        self.vibration_rejected = False
        if mask is None:
            return []

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        # 三脚の揺れ・照明の急変で画面全体が前景になったフレームは丸ごと捨てる
        if cv2.countNonZero(mask) / float(mask.size) > P["vibration_reject_ratio"]:
            self.vibration_rejected = True
            return []

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < P["min_area_px"] or area > P["max_area_px"]:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            x, y, w, h = cv2.boundingRect(c)
            out.append([M["m10"] / M["m00"], M["m01"] / M["m00"],
                        float(area), int(x), int(y), int(w), int(h)])

        out.sort(key=lambda c: -c[2])
        return out[:P["max_candidates"]]


# ── カメラ設定 ────────────────────────────────────────────────
def apply_focus_settings(cap):
    if not DISABLE_AUTOFOCUS:
        return
    try:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, FOCUS_VALUE)
        print("[RPi] オートフォーカス無効・無限遠固定")
    except Exception as e:
        print(f"[RPi] [WARN] フォーカス設定に失敗: {e}")


def lock_exposure(cap):
    """
    現在の自動露出の結果を読み取り、その値で固定する。

    STREAM開始時（＝競技開始時）に呼ぶ。それまでは自動のままなので
    会場の明るさに勝手に合い、当日の手作業は不要。
    """
    if not LOCK_EXPOSURE_ON_STREAM:
        return
    try:
        exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
        wb = cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, DSHOW_EXPOSURE_MANUAL)
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        if wb and wb > 0:
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
        print(f"[RPi] 露出を固定 (exposure={exposure:.1f}, wb={wb:.0f})")
    except Exception as e:
        print(f"[RPi] [WARN] 露出固定に失敗（自動のまま続行）: {e}")


# ── マウスコールバック（接続前から有効） ─────────────────────
def on_click(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    W = TARGET_WIDTH
    # [EXIT] ボタン（右上）
    if x > W - 120 and y < 58:
        shared_state["quit"] = True
        return
    # [RESET] ボタン（EXITの左隣）
    if W - 260 < x < W - 130 and y < 58:
        calib_pts.clear()
        print("[RPi] キャリブ点リセット")
        return
    # キャリブ点追加
    if len(calib_pts) < 5:
        calib_pts.append([x, y])
        print(f"[RPi] 点{len(calib_pts)}/5: ({x}, {y})")


# ── 共通オーバーレイ描画 ─────────────────────────────────────
def draw_overlay(frame):
    """ボタンとキャリブ点を常時描画して返す"""
    disp = frame.copy()
    W = TARGET_WIDTH

    # [EXIT] ボタン
    cv2.rectangle(disp, (W - 120, 0), (W, 58), (0, 0, 180), -1)
    cv2.putText(disp, "EXIT", (W - 105, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

    # [RESET] ボタン
    cv2.rectangle(disp, (W - 260, 0), (W - 130, 58), (70, 70, 70), -1)
    cv2.putText(disp, "RESET", (W - 255, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

    # キャリブ点
    for i, p in enumerate(calib_pts):
        cv2.circle(disp, (p[0], p[1]), 10, (0, 0, 255), -1)
        cv2.putText(disp, str(i + 1), (p[0] + 14, p[1] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    return disp


# ── ローカルディスプレイ初期化 ────────────────────────────────
def init_local_display(cap):
    try:
        cv2.namedWindow("RPi Camera", cv2.WINDOW_NORMAL)
        ret, frame = cap.read()
        if ret:
            disp = draw_overlay(frame)
            cv2.putText(disp, "BOOTING...", (20, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow("RPi Camera", disp)
        cv2.setWindowProperty("RPi Camera",
                              cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.waitKey(100)
        print("[RPi] ローカルディスプレイ: 有効")
        return True
    except Exception as e:
        print(f"[RPi] ディスプレイ無効: {e}")
        return False


# ── 1行受信 ───────────────────────────────────────────────────
def recv_line(conn) -> str:
    buf = b""
    while b"\n" not in buf:
        chunk = conn.recv(64)
        if not chunk:
            raise ConnectionError("client disconnected")
        buf += chunk
    return buf.split(b"\n")[0].decode().strip()


# ── キャリブレーションモード ──────────────────────────────────
def handle_calib(conn, cap, use_display):
    """
    接続前に点を選択済みなら即送信。
    未選択なら5点が揃うまで待機（その間もクリック可能）。
    """
    print(f"[RPi] CALIB接続 - 現在{len(calib_pts)}点選択済み")

    while len(calib_pts) < 5:
        ret, frame = cap.read()
        if not ret:
            break
        if use_display:
            disp = draw_overlay(frame)
            remaining = 5 - len(calib_pts)
            cv2.putText(disp, f"CALIB: Click {remaining} more points",
                        (20, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(disp, "BL -> BR -> TR -> TL -> TL+2m",
                        (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (180, 180, 180), 1)
            cv2.imshow("RPi Camera", disp)
            cv2.waitKey(1)

    if len(calib_pts) < 5:
        print("[RPi] キャリブ中断（フレーム取得失敗）")
        return

    conn.sendall((json.dumps({"calib_pts": calib_pts}) + "\n").encode())
    print(f"[RPi] キャリブ点送信完了: {calib_pts}")

    # 完了表示（2秒）
    deadline = time.time() + 2.0
    while time.time() < deadline:
        ret, frame = cap.read()
        if not ret:
            break
        if use_display:
            disp = draw_overlay(frame)
            cv2.putText(disp, "CALIB COMPLETE!  Sent to laptop",
                        (20, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow("RPi Camera", disp)
            cv2.waitKey(1)

    calib_pts.clear()


# ── ストリーミングモード ──────────────────────────────────────
def handle_stream(conn, cap, use_display):
    detector = Detector()
    lock_exposure(cap)

    while not shared_state["quit"]:
        ret, frame = cap.read()
        ts = time.time()
        if not ret:
            break

        cands = detector.detect(frame)
        best = cands[0] if cands else None

        if use_display:
            disp = draw_overlay(frame)
            for i, c in enumerate(cands):
                cu, cvv = int(c[0]), int(c[1])
                is_best = (i == 0)
                color = (0, 255, 0) if is_best else (110, 110, 110)
                cv2.rectangle(disp, (c[3], c[4]), (c[3] + c[5], c[4] + c[6]),
                              color, 2 if is_best else 1)
                if is_best:
                    cv2.circle(disp, (cu, cvv), 3, (0, 255, 0), -1)
                    cv2.putText(disp, f"({cu},{cvv})", (cu + 16, cvv - 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if not cands:
                label = ("FRAME REJECTED" if detector.vibration_rejected
                         else "NO TARGET")
                color = ((0, 140, 255) if detector.vibration_rejected
                         else (80, 80, 80))
                cv2.putText(disp, label, (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2)
            cv2.putText(disp, f"STREAMING  cands={len(cands)}",
                        (20, TARGET_HEIGHT - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.imshow("RPi Camera", disp)
            cv2.waitKey(1)

        payload = {
            # 候補リスト [u, v, area, x, y, w, h]（PC側が幾何整合で選ぶ）
            "pts": cands,
            # 後方互換: 従来の単一点フィールド
            "pt": [best[0], best[1]] if best else None,
            # 2カメラの時刻ずれ補正用（Phase C のカルマン予測で使用）
            "t": ts,
            "rejected": detector.vibration_rejected,
        }
        if SEND_PREVIEW:
            small = cv2.resize(frame, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE)
            for i, c in enumerate(cands):
                cv2.circle(small,
                           (int(c[0] * PREVIEW_SCALE), int(c[1] * PREVIEW_SCALE)),
                           8, (0, 255, 0) if i == 0 else (110, 110, 110),
                           2 if i == 0 else 1)
            _, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
            payload["frame"] = base64.b64encode(buf).decode()

        try:
            conn.sendall((json.dumps(payload) + "\n").encode())
        except (BrokenPipeError, ConnectionResetError, OSError):
            print("[RPi] ストリーム切断")
            break


# ── メイン ───────────────────────────────────────────────────
def main():
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  TARGET_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    apply_focus_settings(cap)

    ret, frame = cap.read()
    if not ret:
        print("[RPi] カメラ起動失敗")
        return
    h, w = frame.shape[:2]
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[RPi Camera] {w}x{h}@{actual_fps:.1f}fps (要求{TARGET_FPS}fps)  "
          f"ポート {PORT} で待機中...")

    use_display = init_local_display(cap)
    if use_display:
        cv2.setMouseCallback("RPi Camera", on_click)

    with socket.socket() as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)
        srv.settimeout(0.033)   # 非ブロッキングaccept（~30fps）

        while not shared_state["quit"]:
            # ── 接続待ち中もカメラ表示 ──────────────────────────
            if use_display:
                ret, frame = cap.read()
                if ret:
                    disp = draw_overlay(frame)
                    n = len(calib_pts)
                    if n < 5:
                        cv2.putText(disp,
                                    f"Waiting...  Calib pts: {n}/5",
                                    (20, 52),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
                    else:
                        cv2.putText(disp,
                                    "5 pts ready. Waiting for laptop CALIB...",
                                    (20, 52),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 180), 2)
                    cv2.imshow("RPi Camera", disp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

            # ── 接続受付 ────────────────────────────────────────
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[RPi] accept エラー: {e}")
                continue

            print(f"[RPi] 接続: {addr}")
            try:
                conn.sendall(
                    (json.dumps({"width": w, "height": h}) + "\n").encode()
                )
                try:
                    mode = recv_line(conn)
                except ConnectionError:
                    print("[RPi] モード受信前に切断（無視）")
                    continue

                if mode == "CALIB":
                    handle_calib(conn, cap, use_display)
                elif mode == "STREAM":
                    handle_stream(conn, cap, use_display)
                else:
                    print(f"[RPi] 不明なモード: {mode}")

            except Exception as e:
                print(f"[RPi] 接続処理エラー: {e}")
            finally:
                conn.close()

    cv2.destroyAllWindows()
    cap.release()


if __name__ == "__main__":
    main()
