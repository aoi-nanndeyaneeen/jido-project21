import serial
import numpy as np
import cv2
import time

PORT = "COM8"
BAUD = 200000000

last=time.time()
count=0

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Connected!")

SCALE = 20

while True:

    # AA55を探す
    while True:
        b = ser.read(1)

        if not b:
            continue

        if b[0] == 0xAA:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x55:
                break

    frame = ser.read(35*35)

    count += 1

    if time.time() - last >= 1:
        print("FPS =", count)
        count = 0
        last = time.time()

    if len(frame) != 35*35:
        continue

    img = np.frombuffer(frame, dtype=np.uint8).reshape((35,35))

    img = cv2.normalize(img,None,0,255,cv2.NORM_MINMAX)

    img = cv2.resize(img,(700,700),interpolation=cv2.INTER_NEAREST)

    cv2.imshow("PMW3901",img)

    if cv2.waitKey(1)==27:
        break