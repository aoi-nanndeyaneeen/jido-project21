# CSVログ記録クラス

import csv
import datetime
from pathlib import Path


class CsvLogger:
    """ディレクトリ作成・ファイルオープン・ヘッダー書き込みをまとめた共通CSVロガー基底クラス"""

    def __init__(self, path: Path, header: list, mode: str = "w"):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        write_header = mode == "w" or not path.exists()
        self._fh = open(path, mode=mode, newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        if write_header:
            self._writer.writerow(header)

    def close(self):
        self._fh.close()


class FlightLogger(CsvLogger):
    HEADER = ["Time", "Detected",
              "Pos_X(m)", "Pos_Y(m)", "Pos_Z(m)",
              "Residual(m)", "Cam1_Detected", "Cam2_Detected", "Jump_Rejected"]

    def __init__(self, log_path: Path):
        super().__init__(log_path, self.HEADER, mode="a")

    def write(self, P_vec, current_z, residual, cam1_detected, cam2_detected, jump_rejected):
        """毎フレーム呼び出す。P_vec=None なら未検知として記録"""
        t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if P_vec is not None:
            self._writer.writerow([t, 1,
                round(P_vec[0], 3), round(P_vec[1], 3), round(current_z, 3),
                round(residual, 3), int(cam1_detected), int(cam2_detected), int(jump_rejected)])
        else:
            self._writer.writerow([t, 0, "", "", round(current_z, 3),
                round(residual, 3), int(cam1_detected), int(cam2_detected), int(jump_rejected)])
        self._fh.flush()
