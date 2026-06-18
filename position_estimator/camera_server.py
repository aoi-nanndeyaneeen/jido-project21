import cv2, socket, json, base64, numpy as np

HOST, PORT = "0.0.0.0", 5555
CAMERA_ID = 0
SEND_PREVIEW = True
PREVIEW_SCALE = 0.5

# --- 差分パラメータ ---
DIFF_THRESHOLD = 25
MIN_AREA = 100

# --- 解像度設定 ---
TARGET_WIDTH  = 1280
TARGET_HEIGHT = 720


def detect_plane(prev_gray, curr_gray):
    diff = cv2.absdiff(prev_gray, curr_gray)
    _, thresh = cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
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


def init_local_display():
    """ローカルディスプレイへのフルスクリーン表示を初期化。
    ディスプレイが使えない場合は False を返す。"""
    try:
        cv2.namedWindow("RPi Camera", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("RPi Camera",
                              cv2.WND_PROP_FULLSCREEN,
                              cv2.WINDOW_FULLSCREEN)
        # テスト描画で表示可能か確認
        test = np.zeros((100, 100, 3), dtype=np.uint8)
        cv2.imshow("RPi Camera", test)
        cv2.waitKey(1)
        print("[RPi] ローカルディスプレイ: 有効")
        return True
    except Exception as e:
        print(f"[RPi] ローカルディスプレイ: 無効 ({e})")
        return False


def serve_client(conn, cap, w, h, use_display):
    """1クライアントへの送信ループ。切断されたらreturnする。"""
    ret, frame = cap.read()
    if not ret:
        return
    prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    conn.sendall((json.dumps({"width": w, "height": h}) + "\n").encode())

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pt = detect_plane(prev_gray, curr_gray)
        prev_gray = curr_gray

        # ── ローカル表示（フルスクリーン） ────────────────────────
        if use_display:
            disp = frame.copy()
            if pt:
                cx, cy = int(pt[0]), int(pt[1])
                cv2.circle(disp, (cx, cy), 12, (0, 255, 0), 2)
                cv2.circle(disp, (cx, cy),  3, (0, 255, 0), -1)
                cv2.putText(disp, f"({cx}, {cy})",
                            (cx + 16, cy - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(disp, "NO TARGET",
                            (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (80, 80, 80), 2)
            cv2.putText(disp, "RPi Camera Server  [Q] quit",
                        (20, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.imshow("RPi Camera", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[RPi] ローカルQキーで終了")
                return

        # ── ネットワーク送信 ──────────────────────────────────────
        payload = {"pt": pt}

        if SEND_PREVIEW:
            small = cv2.resize(frame, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE)
            if pt:
                sx = int(pt[0] * PREVIEW_SCALE)
                sy = int(pt[1] * PREVIEW_SCALE)
                cv2.circle(small, (sx, sy), 8, (0, 255, 0), 2)
            _, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
            payload["frame"] = base64.b64encode(buf).decode()

        try:
            conn.sendall((json.dumps(payload) + "\n").encode())
        except (BrokenPipeError, ConnectionResetError, OSError):
            print("[RPi] クライアント切断")
            break


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

    use_display = init_local_display()

    with socket.socket() as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)

        while True:
            print("[RPi] 接続待ち...")
            conn, addr = srv.accept()
            print(f"[RPi] 接続: {addr}")
            with conn:
                serve_client(conn, cap, w, h, use_display)

    if use_display:
        cv2.destroyAllWindows()
    cap.release()


if __name__ == "__main__":
    main()