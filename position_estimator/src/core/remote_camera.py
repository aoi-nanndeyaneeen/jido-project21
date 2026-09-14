import socket, json, base64, cv2, time
import numpy as np
from core.camera import Candidate

RECONNECT_INTERVAL_S = 1.0   # 切断後、何秒おきに再接続を試みるか
CONNECT_TIMEOUT_S    = 1.0   # 接続試行1回あたりの上限（呼び出し元スレッドを長時間止めないため）


class RemoteCamera:
    def __init__(self, host: str, port: int = 5555, label: str = "RemoteCamera"):
        self.host  = host
        self.port  = port
        self.label = label
        self.sock  = None
        self.buf   = ""
        self.width  = None
        self.height = None
        self._mode  = "STREAM"          # 再接続時に同じモードで繋ぎ直すため保持
        self._next_reconnect_t = 0.0
        self.last_frame_time    = 0.0
        self.vibration_rejected = False

    def connect(self, mode: str = "STREAM"):
        """
        サーバーに接続してモードを通知する。
        mode: "STREAM"（通常）または "CALIB"（キャリブレーション）
        """
        self._mode = mode
        self.buf = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(CONNECT_TIMEOUT_S)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(2.0)

        # 解像度情報を受信
        info = self._readline()
        d = json.loads(info)
        self.width, self.height = d["width"], d["height"]

        # ★ モードをサーバーに送信（必須）
        self.sock.sendall(f"{mode}\n".encode())
        self.sock.settimeout(1.0)   # 通常通信用に戻す
        print(f"[{self.label}] 接続完了: {self.width}x{self.height} (mode={mode})")

    def _try_reconnect(self):
        """
        切断中に呼ばれる。RECONNECT_INTERVAL_S おきにのみ実際の接続を試み、
        それ以外は即座に戻ることで read_and_track() ループを詰まらせない。
        """
        now = time.time()
        if now < self._next_reconnect_t:
            return
        self._next_reconnect_t = now + RECONNECT_INTERVAL_S
        try:
            self.connect(self._mode)
            print(f"[{self.label}] 再接続に成功しました。")
        except Exception:
            if self.sock is not None:
                try:
                    self.sock.close()
                except OSError:
                    pass
            self.sock = None

    def _readline(self) -> str:
        while "\n" not in self.buf:
            chunk = self.sock.recv(65536).decode()
            if not chunk:
                raise ConnectionError("Remote camera disconnected")
            self.buf += chunk
        line, self.buf = self.buf.split("\n", 1)
        return line

    @staticmethod
    def _parse_candidates(d) -> list:
        """
        RPiから届いた候補リストを Candidate に変換する。

        新形式 "pts": [[u, v, area, x, y, w, h], ...]
        旧形式 "pt" : [u, v]                      （古いRPiとの後方互換）
        """
        pts = d.get("pts")
        if pts:
            out = []
            for p in pts:
                if len(p) >= 7:
                    out.append(Candidate(p[0], p[1], p[2],
                                         int(p[3]), int(p[4]), int(p[5]), int(p[6])))
                elif len(p) >= 2:
                    area = p[2] if len(p) > 2 else 0.0
                    out.append(Candidate(p[0], p[1], area,
                                         int(p[0]), int(p[1]), 0, 0))
            return out

        pt = d.get("pt")
        if pt:
            return [Candidate(pt[0], pt[1], 0.0, int(pt[0]), int(pt[1]), 0, 0)]
        return []

    def read_and_detect(self):
        """
        RPiから1フレーム分の検知結果を受け取る。

        Returns:
            (frame, candidates, timestamp) — 未接続・受信失敗時は (None, [], 0.0)
            ※ frame は帯域節約のため縮小済みプレビュー。座標はフル解像度基準。
        """
        if self.sock is None:
            # 切断中：一定間隔でだけ再接続を試み、それ以外は即座に「未検出」として戻る
            self._try_reconnect()
            if self.sock is None:
                return None, [], 0.0

        try:
            line = self._readline()
            d = json.loads(line)
            candidates = self._parse_candidates(d)
            # RPi側で打刻した時刻。2カメラの時刻ずれ補正に使う
            ts = d.get("t") or time.time()
            self.last_frame_time = ts
            self.vibration_rejected = bool(d.get("rejected", False))

            frame = None
            if "frame" in d:
                img_bytes = base64.b64decode(d["frame"])
                arr = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

            return frame, candidates, ts

        except (socket.timeout, json.JSONDecodeError):
            return None, [], 0.0

        except (ConnectionError, OSError) as e:
            # 相手（ラズパイ）の再起動・切断など。ソケットを畳んで次回から再接続を試みる。
            print(f"[{self.label}] 切断を検知しました ({e})。再接続を試みます...")
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
            self.buf = ""
            return None, [], 0.0

    def draw_candidates(self, frame, candidates, best_index=None):
        """プレビュー（縮小済み）に候補を描く。座標はフル解像度基準なので変換する。"""
        if frame is None:
            return
        sx = frame.shape[1] / self.width  if self.width  else 1.0
        sy = frame.shape[0] / self.height if self.height else 1.0

        for i, c in enumerate(candidates):
            is_best = (best_index is not None and i == best_index)
            color = (0, 255, 0) if is_best else (110, 110, 110)
            dx, dy = int(c.u * sx), int(c.v * sy)
            cv2.circle(frame, (dx, dy), 8 if is_best else 5, color,
                       2 if is_best else 1)
            if is_best:
                cv2.putText(frame, f"({int(c.u)},{int(c.v)})", (dx + 12, dy - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.putText(frame, self.label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)
        if self.vibration_rejected:
            cv2.putText(frame, "FRAME REJECTED", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)

    def get_intrinsics(self):
        """
        内部パラメータ (K, dist) を返す。
        事前実測値があればそれを使い、無ければ公称画角からの概算にフォールバック。
        """
        from utils.calib_store import resolve_intrinsics
        return resolve_intrinsics(self.label, self.width, self.height)

    def get_approx_camera_matrix(self):
        """後方互換。新しいコードは get_intrinsics() を使うこと。"""
        return self.get_intrinsics()[0]

    def reset_background(self):
        pass

    def release(self):
        if self.sock:
            self.sock.close()