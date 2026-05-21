import cv2, socket, json, base64, numpy as np

HOST, PORT = "0.0.0.0", 5555
CAMERA_ID = 0
SEND_PREVIEW = False
PREVIEW_SCALE = 0.5

# --- 差分パラメータ ---
DIFF_THRESHOLD = 25
MIN_AREA = 100
# ----------------------

# --- 解像度設定 ---
TARGET_WIDTH  = 1280
TARGET_HEIGHT = 720
# ------------------

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

def main():
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  TARGET_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)

    ret, frame = cap.read()
    if not ret:
        print("[RPi] Camera open failed")
        return

    prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = frame.shape[:2]
    print(f"[RPi Camera] actual resolution: {w}x{h}, waiting for connection...")

    with socket.socket() as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)
        conn, addr = srv.accept()
        print(f"[RPi] Connected: {addr}")
        conn.sendall((json.dumps({"width": w, "height": h}) + "\n").encode())

        with conn:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                pt = detect_plane(prev_gray, curr_gray)
                prev_gray = curr_gray

                payload = {"pt": pt}

                if SEND_PREVIEW:
                    small = cv2.resize(frame, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE)
                    if pt:
                        cx = int(pt[0] * PREVIEW_SCALE)
                        cy = int(pt[1] * PREVIEW_SCALE)
                        cv2.circle(small, (cx, cy), 8, (0, 255, 0), 2)
                    _, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    payload["frame"] = base64.b64encode(buf).decode()

                try:
                    conn.sendall((json.dumps(payload) + "\n").encode())
                except BrokenPipeError:
                    break

    cap.release()

if __name__ == "__main__":
    main()