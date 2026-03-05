# CSVログ記録クラス

import csv
import datetime
from pathlib import Path


class FlightLogger:
    HEADER = ["Time", "Detected",
              "Target_X(m)", "Target_Y(m)", "Target_Z(m)",
              "Roll(deg)", "Pitch(deg)",
              "Alt_raw(m)", "Alt_offset(m)"]

    def __init__(self, log_path: Path):
        log_path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not log_path.exists()
        self._fh = open(log_path, mode='a', newline='', encoding='utf-8')
        self._writer = csv.writer(self._fh)
        if write_header:
            self._writer.writerow(self.HEADER)

    def write(self, P_vec, current_z, roll, pitch, raw_alt, alt_offset):
        """毎フレーム呼び出す。P_vec=None なら未検知として記録"""
        t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if P_vec is not None:
            self._writer.writerow([t, 1,
                round(P_vec[0], 3), round(P_vec[1], 3), round(current_z, 3),
                round(roll, 2), round(pitch, 2),
                round(raw_alt, 3), round(alt_offset, 3)])
        else:
            self._writer.writerow([t, 0, "", "", round(current_z, 3),
                round(roll, 2), round(pitch, 2),
                round(raw_alt, 3), round(alt_offset, 3)])
        self._fh.flush()

    def close(self):
        self._fh.close()