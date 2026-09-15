"""画面解像度に追従する OpenCV ウィンドウ配置。"""


def primary_display_size():
    """Windows の主ディスプレイ解像度。取得不能時だけ安全な既定値を返す。"""
    try:
        import ctypes
        user32 = ctypes.windll.user32
        return int(user32.GetSystemMetrics(0)), int(user32.GetSystemMetrics(1))
    except Exception:
        return 1920, 1080


def flight_window_layout(screen_w, screen_h):
    """高さを上段:下段:空き領域=4.5:4.5:1に分ける。"""
    screen_w = max(1, int(screen_w))
    screen_h = max(1, int(screen_h))
    # タスクバーなどに重ならないよう、下端10%は何も置かない。
    # 余りの1pxも予約領域へ寄せるので、表示ウィンドウは決して下端を越えない。
    top_h = (screen_h * 9) // 20
    bottom_h = (screen_h * 9) // 20
    left_w = screen_w // 2
    velocity_w = (screen_w * 2) // 3
    return {
        "Camera 1": (0, 0, left_w, top_h),
        "Camera 2": (left_w, 0, screen_w - left_w, top_h),
        "Velocity": (0, top_h, velocity_w, bottom_h),
        "Graph": (velocity_w, top_h, screen_w - velocity_w, bottom_h),
        # Link Status は従来どおり左上に置く。必要ならOS側で最前面へ移動できる。
        "Link Status": (0, 0, min(760, left_w), min(300, top_h)),
    }


def apply_window_layout(layout):
    """計算済みの配置を OpenCV のウィンドウに反映する。"""
    import cv2
    for name, (x, y, width, height) in layout.items():
        cv2.resizeWindow(name, width, height)
        cv2.moveWindow(name, x, y)
