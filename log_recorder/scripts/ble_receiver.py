# scripts/ble_receiver.py
# log_recorder (XIAO ESP32C3) が BLE Notify で流してくる LogLinkProto フレームを
# PC で受信し、SD版と同じ LOGnnnn.BIN (32Bヘッダ + Rec直列) として保存する。
# できた .BIN は flight_controller/scripts/bin2csv.py でそのまま CSV 化できる。
#
# 【使い方】
#   pip install bleak
#   python ble_receiver.py                  # ./logs/LOGnnnn.BIN に保存
#   python ble_receiver.py --out D:\logs
#   python ble_receiver.py --name S5-LogBLE  # デバイス名を変えたとき
#
# 【前提】
#   log_recorder/src/main.cpp の BLE_SERVICE_UUID / BLE_CHAR_UUID /
#   BLE_DEVICE_NAME と一致させてあること (どれかを変えたらこちらも直す)。
#
# 【間引きについて】
#   main.cpp の BLE_REC_DECIM により T_REC は間引かれて届く (既定 1/4)。
#   間引かれた分だけ seq が飛んで見えるが、これは損失ではなく意図的な間引き。
#   UART 側 (FC<->XIAO) の本当の損失 (crc_err/drop_ring) は T_STAT を
#   BLE に転送していないためこのスクリプトからは見えない。ロガー側 USB
#   シリアルの 's' コマンドで確認すること。
#
# 【再接続】
#   ドローンが電波圏外に出るなどで切れても自動で再接続を試みる。再接続後に
#   同じ t0_ms の T_START が来れば「同じ飛行の続き」とみなして同じファイルに
#   書き続ける (log_recorder 側が 1Hz で T_START を再送する仕組みと対応)。
#   別の t0_ms が来たら別ファイルに切り替える。
import argparse
import asyncio
import struct
import sys
from pathlib import Path

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("[ERROR] bleak が要ります: pip install bleak")

# ---- quad/LogLinkProto.h と一致させること ---------------------------------
SOF1, SOF2 = 0xA5, 0x5A
T_START, T_REC, T_STOP, T_ACT_ACK = 0x01, 0x02, 0x03, 0x04
T_TELEM = 0x05  # 地上局リンクの下りテレメトリ (S5Telem.h のフレームを連結)。.BIN には書かない
T_ACT = 0x82   # ロガー -> FC (Notify には流れない。参考として持っておくだけ)
T_CMD = 0x83   # 同上。操縦指令 (CmdFrame) を UART へ中継するときの型
CMD_PAYLOAD = 22  # sizeof(S5C::CmdFrame)
HDR_LEN = 5          # SOF1 SOF2 type len seq
BIN_HDR_LEN = 32
BIN_HDR_T0_OFS = 12
ACT_ACK_STRUCT = "<BBBBB"   # action, action_seq, result, imu_ok, i2c_found

# ---- log_recorder/src/main.cpp と一致させること -----------------------------
BLE_DEVICE_NAME = "S5-LogBLE"
BLE_CHAR_UUID = "d5913037-2d8a-41ee-85b9-4e361aa5c8a7"      # Notify (ログ)
BLE_CHAR_CMD_UUID = "d5913038-2d8a-41ee-85b9-4e361aa5c8a7"  # Write (デバッグ指令。ActReq 2byte)
BLE_CHAR_CTRL_UUID = "d5913039-2d8a-41ee-85b9-4e361aa5c8a7"  # Write (操縦指令。CmdFrame 22byte)


def crc8(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class FrameParser:
    """firmware の Rx::feed と同じバイト単位の状態機械。BLE の notify は
    ほぼフレーム境界通りに届くが、保証はないのでストリームとして扱う。"""

    def __init__(self, on_frame):
        self._on_frame = on_frame
        self._st = 0
        self._type = 0
        self._len = 0
        self._seq = 0
        self._payload = bytearray()

    def feed(self, chunk: bytes) -> None:
        for c in chunk:
            if self._st == 0:
                if c == SOF1:
                    self._st = 1
            elif self._st == 1:
                self._st = 2 if c == SOF2 else (1 if c == SOF1 else 0)
            elif self._st == 2:
                self._type = c
                self._st = 3
            elif self._st == 3:
                self._len = c
                self._st = 4
            elif self._st == 4:
                self._seq = c
                self._payload = bytearray()
                self._st = 6 if self._len == 0 else 5
            elif self._st == 5:
                self._payload.append(c)
                if len(self._payload) >= self._len:
                    self._st = 6
            elif self._st == 6:
                calc = crc8(bytes([self._type, self._len, self._seq]) + bytes(self._payload))
                if calc == c:
                    self._on_frame(self._type, self._seq, bytes(self._payload))
                self._st = 0


class BinLogger:
    """受信したフレームを SD版と同じ LOGnnnn.BIN 形式で書き出す。"""

    # ★ on_event: ログの開閉を知らせる先。既定は print だが、全画面で
    #   描き直す側 (position_estimator/src/console.py の BLE タップ) から
    #   使うときに print されると画面が崩れるので差し替えられるようにした。
    def __init__(self, outdir: Path, on_event=None):
        self._say = on_event if on_event is not None else print
        self._outdir = outdir
        self._f = None
        self._path = None
        self._t0 = None
        self._bytes = 0
        self._rec_count = 0
        self._last_seq = None
        self._seq_gap = 0

    def _next_path(self) -> Path:
        for i in range(10000):
            p = self._outdir / f"LOG{i:04d}.BIN"
            if not p.exists():
                return p
        return self._outdir / "LOG9999.BIN"

    def _close(self) -> None:
        if self._f is None:
            return
        self._f.close()
        self._say(f"[CLOSE] {self._path.name}  {self._bytes}B  "
                  f"{self._rec_count}rec  seq_gap={self._seq_gap}")
        self._f = None

    def on_frame(self, type_: int, seq: int, payload: bytes) -> None:
        if self._last_seq is not None:
            gap = (seq - self._last_seq - 1) & 0xFF
            if gap:
                self._seq_gap += gap
        self._last_seq = seq

        if type_ == T_START:
            if len(payload) != BIN_HDR_LEN:
                return
            t0 = struct.unpack_from("<I", payload, BIN_HDR_T0_OFS)[0]
            if self._f is not None and t0 == self._t0:
                return   # 1Hz 再送。同じセッションの続きなので何もしない
            self._close()
            self._path = self._next_path()
            self._f = self._path.open("wb")
            self._f.write(payload)
            self._t0 = t0
            self._bytes = BIN_HDR_LEN
            self._rec_count = 0
            self._seq_gap = 0
            self._say(f"[OPEN]  {self._path.name}  t0={t0}")

        elif type_ == T_REC:
            if self._f is None:
                return    # START 前に届いた分は捨てる (ロガー側の orphan と同じ扱い)
            self._f.write(payload)
            self._bytes += len(payload)
            self._rec_count += 1

        elif type_ == T_STOP:
            self._close()

    def close(self) -> None:
        self._close()

    def status(self) -> dict:
        """今書いているファイルの状況。コンソールの表示用。"""
        return {"path": self._path, "open": self._f is not None,
                "bytes": self._bytes, "rec": self._rec_count,
                "seq_gap": self._seq_gap, "t0": self._t0}


async def run(args) -> None:
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    logger = BinLogger(outdir)
    parser = FrameParser(logger.on_frame)

    try:
        while True:
            print(f"[SCAN] '{args.name}' を探索中...")
            dev = await BleakScanner.find_device_by_name(args.name, timeout=15.0)
            if dev is None:
                print("[SCAN] 見つかりません。3秒後に再試行します")
                await asyncio.sleep(3)
                continue

            try:
                async with BleakClient(dev) as client:
                    print(f"[CONNECT] {dev.address}")

                    def handle_notify(_, data: bytearray) -> None:
                        parser.feed(bytes(data))

                    await client.start_notify(BLE_CHAR_UUID, handle_notify)
                    while client.is_connected:
                        await asyncio.sleep(0.5)
            except Exception as e:
                print(f"[ERROR] {e}")

            print("[DISCONNECT] 再接続します")
            await asyncio.sleep(2)
    finally:
        logger.close()


def main() -> None:
    ap = argparse.ArgumentParser(
        description="log_recorder (XIAO ESP32C3) から BLE でログを受信して .BIN に保存する")
    ap.add_argument("--out", default="logs", help="保存先フォルダ (既定: ./logs)")
    ap.add_argument("--name", default=BLE_DEVICE_NAME, help="BLE デバイス名")
    args = ap.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[STOP] 終了します")


if __name__ == "__main__":
    main()
