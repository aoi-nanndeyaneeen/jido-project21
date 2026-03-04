import cv2

# IDを0〜3まで順番に試して、開けたものを全てウィンドウ表示する
for i in range(4):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"ID {i} が開けました。ウィンドウを確認してください。")
        ret, frame = cap.read()
        if ret:
            cv2.imshow(f"Camera ID: {i}", frame)
        cap.release()
    else:
        print(f"ID {i} は開けません。")

cv2.waitKey(0)
cv2.destroyAllWindows()