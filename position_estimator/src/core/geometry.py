# 数学関数群、レイキャスト、角度計算、交点推定などをまとめるモジュール

import cv2
import numpy as np
from utils.config import VALID_X, VALID_Y, VALID_Z


def get_ray(u, v, K, R, tvec):
    """ピクセル座標(u,v)からワールド空間のレイ（原点・方向）を返す"""
    pt_2d = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pt_2d, K, None)
    dir_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
    cam_pos = -R.T.dot(tvec).flatten()
    dir_world = R.T.dot(dir_cam)
    dir_world /= np.linalg.norm(dir_world)
    return cam_pos, dir_world.flatten()


def intersect_rays(O1, D1, O2, D2):
    """
    2本のレイの最近接点（中間点）を3次元位置として返す。

    レイ1: P = O1 + t1 * D1
    レイ2: P = O2 + t2 * D2

    最小二乗法で t1, t2 を求め、中間点をワールド座標として返す。
    平行に近い場合 (行列式が小さい場合) は None を返す。

    Args:
        O1 (np.ndarray): カメラ1の位置 (3,)
        D1 (np.ndarray): カメラ1のレイ方向 (3,) ※正規化済み
        O2 (np.ndarray): カメラ2の位置 (3,)
        D2 (np.ndarray): カメラ2のレイ方向 (3,) ※正規化済み

    Returns:
        np.ndarray or None: 推定された3D座標 (3,), または計算不能な場合 None
        float: 2本のレイ間の最近接距離 (誤差の目安)
    """
    # 連立方程式: t1*D1 - t2*D2 = O2 - O1 を最小二乗で解く
    # A * [t1, t2]^T = b
    A = np.column_stack([D1, -D2])   # (3, 2)
    b = O2 - O1                       # (3,)

    # 最小二乗解: (A^T A) [t1,t2]^T = A^T b
    ATA = A.T @ A   # (2, 2)
    det = ATA[0, 0] * ATA[1, 1] - ATA[0, 1] * ATA[1, 0]
    if abs(det) < 1e-10:
        # レイがほぼ平行 → 交点計算不能
        return None, float('inf')

    ATb = A.T @ b   # (2,)
    params = np.linalg.solve(ATA, ATb)  # [t1, t2]
    t1, t2 = params

    P1 = O1 + t1 * D1
    P2 = O2 + t2 * D2
    P_mid = (P1 + P2) / 2.0          # 最近接中間点
    residual = np.linalg.norm(P1 - P2)  # 残差距離 (誤差の目安)

    return P_mid, residual


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