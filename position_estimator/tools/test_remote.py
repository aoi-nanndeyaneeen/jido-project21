"""
Camera2（RPi）のCALIB→STREAMフローをmain.pyと無関係に単独テスト。
ラップトップで実行し、ターミナル出力をすべてコピーして教えてください。
"""
import socket, json, time

RPI_HOST = "192.168.11.13"
RPI_PORT = 5555


def recv_line(sock, buf=b""):
    while b"\n" not in buf:
        buf += sock.recv(4096)
    line, rest = buf.split(b"\n", 1)
    return line.decode(), rest


# ─── Step 1: CALIB ───────────────────────────────────────────
print("=" * 40)
print("Step 1: CALIB接続")
try:
    s = socket.socket()
    s.settimeout(120.0)
    s.connect((RPI_HOST, RPI_PORT))
    print("  [OK] 接続")

    line, buf = recv_line(s)
    print(f"  [OK] 解像度受信: {line}")

    s.sendall(b"CALIB\n")
    print("  [OK] CALIB送信 ← ラズパイで5点クリックしてください")

    line, buf = recv_line(s, buf)
    data = json.loads(line)
    print(f"  [OK] 点受信: {data['calib_pts']}")
    s.close()
    print("  [OK] CALIB接続クローズ")

except Exception as e:
    print(f"  [FAIL] {e}")
    exit(1)

time.sleep(0.5)

# ─── Step 2: STREAM ──────────────────────────────────────────
print()
print("=" * 40)
print("Step 2: STREAM接続")
try:
    s2 = socket.socket()
    s2.settimeout(5.0)
    s2.connect((RPI_HOST, RPI_PORT))
    print("  [OK] 接続")

    line, buf2 = recv_line(s2)
    print(f"  [OK] 解像度受信: {line}")

    s2.sendall(b"STREAM\n")
    print("  [OK] STREAM送信")

    print("  受信データ（5フレーム）:")
    for i in range(5):
        line, buf2 = recv_line(s2, buf2)
        d = json.loads(line)
        print(f"    [{i}] pt={d.get('pt')}  frame={'YES' if 'frame' in d else 'NO'}")

    s2.close()
    print("  [OK] 正常動作確認")

except Exception as e:
    print(f"  [FAIL] {e}")