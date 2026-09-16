# ground_receiver/used  -  使わなくなったもの置き場

削除はしないが、ビルドにも運用にも入らないものをここへ移す (2026-09-16)。

| ファイル | 何だったか | 現行の代わり |
|---|---|---|
| `main_pc.cpp` | 旧地上局。`Config.h` の PlaneData/GroundData (46 byte) を IM920 で中継していた。GroundData は IM920sL の 32 byte 上限を超えていて機体に届いていなかった | `src/main.cpp` (S5Telem/S5Cmd プロトコル) |
| `include/Config.h` `Telemetry.h` `Serial_com.h` `Serial_monitor.h` | 上の main_pc.cpp 専用ヘッダ | `../protocol/S5Telem.h` `S5Cmd.h` `Im920Frame.h` |
| `logger.py` `plot_3d.py` | main_pc.cpp の `Roll_Ang:...` テキスト出力を CSV 化 / 3D 描画 | `position_estimator/src/console.py` (S5Link が CSV を書く) / `tools/analyze_poshold.py` |

旧 platformio.ini の env (`teensy40` / `uno` = main_pc.cpp を焼くための環境) も同時に外した。
戻したいときは git 履歴の `ground_receiver/platformio.ini` (2026-09-16 以前) を参照。
