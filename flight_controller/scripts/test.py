import serial
import numpy as np
import cv2

PORT = "COM8"      # ←自分のCOM番号
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Waiting for READY...")

# READYが来るまで待つ
while True:
    line = ser.readline().decode(errors="ignore")
    if "READY" in line:
        break

print("Connected!")

SCALE = 20

while True:

    # フレーム開始マーカー(0xAA 0x55)を探す
    while True:
        b = ser.read(1)
        if not b:
            continue

        if b[0] == 0xAA:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x55:
                break

    # 35×35 = 1225Byte読み込む
    frame = ser.read(35 * 35)

    if len(frame) != 35 * 35:
        continue

    img = np.frombuffer(frame, dtype=np.uint8)
    img = img.reshape((35, 35))

    # コントラスト自動調整
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)

    # 拡大表示
    img = cv2.resize(
        img,
        (35 * SCALE, 35 * SCALE),
        interpolation=cv2.INTER_NEAREST
    )

    cv2.imshow("PMW3901", img)

    key = cv2.waitKey(1)
    if key == 27:      # ESCで終了
        break

ser.close()
cv2.destroyAllWindows()