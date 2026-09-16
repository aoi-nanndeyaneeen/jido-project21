# position_estimator/used  -  使わなくなったもの置き場

削除はしないが、`main.py` / `console.py` のどちらからも参照されないものをここへ移す。

| ファイル | 何だったか | 現行の代わり |
|---|---|---|
| `view_3d.py` | matplotlib の 3D ビュー (初期) | `ui/view_velocity.py` / `ui/view_graph.py` |
| `autopilot.py` | PC で位置PID → スティック相当 (roll/pitch/throttle) を計算して機体へ送る旧方式。IM920sL の遅延を姿勢ループに入れると発振する・GroundData が 32 byte を超えていて届いていなかった、の 2 点で廃止 (WAYPOINT_BRINGUP.md §0) | `core/mission.py` (setpoint = 目標速度/高度だけを送る) |
| `controller.py` | `AltitudeController`。[T] キーで入れる目標高度を Graph に描くだけで、機体には無関係だった | Graph の Target 線はミッションの目標高度 / 機体の保持高度を出す (`app/main_loop.py` `_target_alt_for_graph`) |
| `communication.py` | `SerialReceiver`。旧地上局 main_pc.cpp のテキスト出力 (`Roll_Ang:...`) を読む経路 (`SERIAL_ENABLED`) | `core/s5_link.py` |
| `view_rc.py` `test_view_rc.py` | autopilot.py の RCCommand を描く UI とその単体テスト | — |
| `receiver.py` `sensor_receiver.py` | UDP (5005) で高度センサを受ける初期の実験 | — |
| `dummy_source.py` | CameraTracker / SerialReceiver と同じインターフェースの合成データ源 | `core/dummy_flight.py` (追跡の失探フォールバック) / `tools/replay_detect.py` |

`utils/config.py` の `SERIAL_*` / `ALT_SENSOR_*` / `YAW_SEND_*` は旧経路の設定だが、
config は他所からも読まれるので値は残してある (使われていない)。
