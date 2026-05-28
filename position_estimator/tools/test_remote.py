import socket, json, base64, cv2, numpy as np

sock = socket.socket()
sock.connect(("192.168.11.13", 5555))
sock.settimeout(2.0)
buf = ""

# 解像度情報を受け取る
while "\n" not in buf:
    buf += sock.recv(65536).decode()
line, buf = buf.split("\n", 1)
print("Camera info:", json.loads(line))

while True:
    try:
        while "\n" not in buf:
            buf += sock.recv(65536).decode()   # ここが重要
        line, buf = buf.split("\n", 1)
        d = json.loads(line)

        if "frame" in d:
            img_bytes = base64.b64decode(d["frame"])
            arr = np.frombuffer(img_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow("RPi Camera Preview", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except socket.timeout:
        continue
    except KeyboardInterrupt:
        break

cv2.destroyAllWindows()
sock.close()