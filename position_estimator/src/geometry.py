# 数学関数群、レイキャスト、角度計算などをまとめるモジュール

import cv2
import numpy as np
from config import VALID_X, VALID_Y, VALID_Z


def get_ray(u, v, K, R, tvec):
    """ピクセル座標(u,v)からワールド空間のレイ（原点・方向）を返す"""
    pt_2d = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pt_2d, K, None)
    dir_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
    cam_pos = -R.T.dot(tvec).flatten()
    dir_world = R.T.dot(dir_cam)
    dir_world /= np.linalg.norm(dir_world)
    return cam_pos, dir_world.flatten()


def accel_to_angles(accel):
    """加速度ベクトルから絶対 roll/pitch を計算（ヨーは不可）"""
    ax, ay, az = accel
    norm = np.sqrt(ax**2 + ay**2 + az**2)
    if norm < 1e-6:
        return 0.0, 0.0
    ax, ay, az = ax/norm, ay/norm, az/norm
    pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))
    roll  = np.degrees(np.arctan2(ay, az))
    return roll, pitch


def calc_tilt(accel, ref_roll, ref_pitch):
    """加速度からキャリブ基準を引いた相対 roll/pitch を返す"""
    roll, pitch = accel_to_angles(accel)
    return roll - ref_roll, pitch - ref_pitch


def is_valid_position(P):
    """座標がフィールド有効範囲内かチェック"""
    return (VALID_X[0] <= P[0] <= VALID_X[1] and
            VALID_Y[0] <= P[1] <= VALID_Y[1] and
            VALID_Z[0] <= P[2] <= VALID_Z[1])