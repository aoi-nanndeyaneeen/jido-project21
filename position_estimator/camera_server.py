import cv2, socket, json, base64, numpy as np, time

HOST, PORT     = "0.0.0.0", 5555
CAMERA_ID      = 0
SEND_PREVIEW   = True
PREVIEW_SCALE  = 0.5
TARGET_WIDTH   = 1280
TARGET_HEIGHT  = 720

# --- 差分パラメータ（config.py の値と合わせること） ---
DIFF_THRESHOLD = 15
MIN_AREA       = 40
MORPH_KERNEL   = 5

# ── 共有状態（モジュールレベル） ─────────────────────────────
calib_pts    = []
shared_state = {"quit": False}


# ── 動体検知 ──────────────────────────────────────────────────
def detect_plane(prev_gray, curr_gray):
    diff = cv2.absdiff(prev_gray, curr_gray)
    _, thresh = cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, k)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_AREA:
        return None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return (M["m10"] / M["m00"], M["m01"] / M["m00"])


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
    ret, frame = cap.read()
    if not ret:
        return
    prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    while not shared_state["quit"]:
        ret, frame = cap.read()
        if not ret:
            break
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pt = detect_plane(prev_gray, curr_gray)
        prev_gray = curr_gray

        if use_display:
            disp = draw_overlay(frame)
            if pt:
                cx, cy = int(pt[0]), int(pt[1])
                cv2.circle(disp, (cx, cy), 12, (0, 255, 0), 2)
                cv2.circle(disp, (cx, cy),  3, (0, 255, 0), -1)
                cv2.putText(disp, f"({cx},{cy})", (cx + 16, cy - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(disp, "NO TARGET", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (80, 80, 80), 2)
            cv2.putText(disp, "STREAMING", (20, TARGET_HEIGHT - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.imshow("RPi Camera", disp)
            cv2.waitKey(1)

        payload = {"pt": pt}
        if SEND_PREVIEW:
            small = cv2.resize(frame, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE)
            if pt:
                cv2.circle(small,
                           (int(pt[0] * PREVIEW_SCALE), int(pt[1] * PREVIEW_SCALE)),
                           8, (0, 255, 0), 2)
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

    ret, frame = cap.read()
    if not ret:
        print("[RPi] カメラ起動失敗")
        return
    h, w = frame.shape[:2]
    print(f"[RPi Camera] {w}x{h}  ポート {PORT} で待機中...")

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