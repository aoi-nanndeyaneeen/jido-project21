# 数学関数群、レイキャスト、角度計算、交点推定などをまとめるモジュール

from typing import NamedTuple, Optional

import cv2
import numpy as np


class CameraCalib(NamedTuple):
    """
    1台分のキャリブレーション一式。

    K, dist … 内部パラメータ（レンズの性質。事前にチェッカーボードで実測）
    R, tvec … 外部パラメータ（設置位置と向き。当日5点クリックで算出）

    dist が None の場合は歪み補正なしになる。実測値を用意すること。
    """
    K: np.ndarray
    dist: Optional[np.ndarray]
    R: np.ndarray
    tvec: np.ndarray

    @property
    def origin(self) -> np.ndarray:
        """カメラのワールド座標 (3,)"""
        return (-self.R.T.dot(self.tvec)).flatten()

    def ray(self, u, v):
        """ピクセル(u,v)に対応するワールド空間のレイ (原点, 方向) を返す。"""
        return get_ray(u, v, self.K, self.R, self.tvec, self.dist)

    def project(self, points_3d):
        """ワールド座標の3D点群を画像座標へ投影する。(N,2)"""
        objp = np.asarray(points_3d, dtype=np.float64).reshape(-1, 3)
        rvec, _ = cv2.Rodrigues(self.R)
        proj, _ = cv2.projectPoints(objp, rvec, self.tvec, self.K, self.dist)
        return proj.reshape(-1, 2)


def get_ray(u, v, K, R, tvec, dist=None):
    """
    ピクセル座標(u,v)からワールド空間のレイ（原点・方向）を返す。

    Args:
        dist: 歪み係数。チェッカーボード実測値があれば必ず渡すこと。
              None だと歪み補正なしになり、特に画像周辺部で誤差が出る
              （C920 の樽型歪みは無視できない）。
    """
    pt_2d = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pt_2d, K, dist)
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


def solve_extrinsics(object_points, image_points, K, dist=None):
    """
    フィールド基準点のクリック結果から外部パラメータ (R, tvec) を求める。

    EPnP で初期解を出したあと solvePnPRefineLM で再投影誤差を直接最小化する。
    5点しかないので、この refine の有無で精度がはっきり変わる。

    Returns:
        (R, tvec, rvec) または解けなければ (None, None, None)
    """
    objp = np.asarray(object_points, dtype=np.float32).reshape(-1, 3)
    imgp = np.asarray(image_points,  dtype=np.float32).reshape(-1, 2)

    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist, flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        return None, None, None

    # 再投影誤差を直接最小化して精度を詰める
    try:
        rvec, tvec = cv2.solvePnPRefineLM(objp, imgp, K, dist, rvec, tvec)
    except cv2.error:
        pass   # 古いOpenCVでは未実装。EPnPの解のまま使う

    R, _ = cv2.Rodrigues(rvec)
    return R, tvec, rvec


def reprojection_error(object_points, image_points, K, rvec, tvec, dist=None):
    """
    solvePnP の答えの検算。

    求めた (R, tvec) を使って3D点を画像に投影し直し、
    実際のクリック位置からどれだけずれたかを px で測る。

    ずれる原因は「クリック位置のずれ」「K が実際と違う」
    「入力した3D座標が違う（順序ミス・実測ミス）」の3つ。
    特に順序ミスは値が跳ね上がるので一発で分かる。

    Returns:
        mean_px (float), per_point_px (np.ndarray)
    """
    objp = np.asarray(object_points, dtype=np.float32).reshape(-1, 3)
    imgp = np.asarray(image_points,  dtype=np.float32).reshape(-1, 2)

    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)

    per_point = np.linalg.norm(proj - imgp, axis=1)
    return float(per_point.mean()), per_point


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


def approx_camera_matrix(width, height, hfov_deg=78.0):
    """
    水平画角から内部行列を概算する。

    ★ これはチェッカーボード実測値が無い場合の暫定値でしかない。
       本番前に tools/calibrate_intrinsics.py で必ず実測すること。

    従来は focal = width 決め打ちだったが、これは水平画角53°相当であり
    実機（C920 90°版 / α6400+16mm 72.6°）とかけ離れていた。
    K が違うと solvePnP の R,tvec が歪み、三角測量の残差が構造的に大きくなる。
    """
    focal = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    return np.array([[focal, 0,     width  / 2],
                     [0,     focal, height / 2],
                     [0,     0,     1         ]], dtype=np.float32)
