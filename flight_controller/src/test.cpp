import serial
import numpy as np
import cv2

# COM番号を変更してください
ser = serial.Serial("COM5", 115200)

while True:

    line = ser.readline().decode().strip()

    if line != "FRAME":
        continue

    img = []

    while True:

        line = ser.readline().decode().strip()

        if line == "END":
            break

        img.append(int(line))

    if len(img) != 35*35:
        continue

    img = np.array(img, dtype=np.uint8)
    img = img.reshape((35,35))

    # コントラスト自動補正
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)

    # 20倍拡大
    big = cv2.resize(img,
                     (700,700),
                     interpolation=cv2.INTER_NEAREST)

    cv2.imshow("PMW3901", big)

    if cv2.waitKey(1) == 27:
        break

ser.close()
cv2.destroyAllWindows()