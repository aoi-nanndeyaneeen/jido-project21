// ============================================================
//  s5_log.cpp  -  s5 (POSHOLD) 解析テレメトリの受信 & CSV化 (地上局)
// ============================================================
//  機体側 drone_s5.cpp が 15Hz で投げてくる S5T::AltFrame / PosFrame /
//  AttFrame / ParamFrame を受け取り、USB シリアルへ CSV として吐き出す。PC 側は
//    python tools/s5_logger.py
//  で受けて ground_receiver/logs/ に保存し、
//    python tools/analyze_poshold.py
//  で解析する。
//
//  ★ IM920sL の実効ペイロードは 32 バイト (実測)。そのため 1 パケットに
//    全部は載らず、A(高度+姿勢角) / B(水平位置) / C(姿勢ループ内部) を
//    順番に送っている。何をどの順で送るかはモードで変わる (S5Telem.h)。
//    ここでは受け取るたびに「他のフレームは前回値」で 1 行を組み立てる
//    (forward-fill)。frame 列が「今回どれが新しいか」= 0:A 1:B 2:C。
//    列の意味は S5Telem.h を参照。
//
//  ビルド / 書き込み (XIAO RP2040 の地上局):
//      pio run -e xiao_s5_log -t upload
//      pio device monitor -e xiao_s5_log      (手で見るだけならこれでも可)
//
//  USB シリアルから使えるキー (押せば必ず1行返事を出す):
//      1 : CSV 出力 ON   /   0 : CSV 出力 OFF   (冪等。s5_logger.py はこれ)
//      l : CSV 出力のトグル (人間が手で押す用)
//          ★ これは「画面に CSV を吐く」だけ。ファイルを作るのは PC 側の
//            s5_logger.py。シリアルモニタで押してもファイルはできない。
//      s : 今の状態を1回表示
//      d : IM920 の生の行をそのまま表示 (リンクの切り分け用)
//      z : 統計 (受信数 / 欠落数) をゼロクリア
//      h : ヘルプ
//
//  出力プロトコル (flight_controller/scripts/logger.py と同じ流儀):
//      HEADER,<列名...>     ... LOG_START の直前に1回
//      LOG_START
//      DATA,<値...>         ... 1パケット = 1行
//      LOG_STOP
//      PARAM,<ゲイン...>    ... 数秒に1回。どのゲインで飛んでいたかの記録
//      # ...                ... 人間向けのメッセージ (PC側は読み飛ばす)
//
//  ★ 配線は im920_passthrough.cpp と同じ (XIAO TX=D6/GP0 -> IM920 RXD,
//    XIAO RX=D7/GP1 <- IM920 TXD)。IM920 の電源は 3V3 から取ること。
//    5V ピンから取ると USB 列挙に失敗して書き込めなくなる。
//  ★ マルチホップ設定 (ENHP/DSHP) は両機で一致していないと一切通信できない。
//    現状は両機とも DSHP。確認は im920_test / im920_passthrough で RPRM。
// ============================================================
#include <Arduino.h>
#include "S5Telem.h"

// ------------------------------------------------------------
//  ボードごとの UART 設定
// ------------------------------------------------------------
#if defined(ARDUINO_ARCH_RP2040)
constexpr int PIN_XIAO_TX = 0;   // D6 -> IM920 RXD
constexpr int PIN_XIAO_RX = 1;   // D7 <- IM920 TXD
#endif

static HardwareSerial* IM = &Serial1;
constexpr unsigned long IM_BAUD  = 19200;
constexpr unsigned long USB_BAUD = 115200;

// ------------------------------------------------------------
//  CSV の列。A/B/C を forward-fill して 1 行にまとめる。
//  ★ 列を足したら必ず emitData() 側も同じ順で足すこと。
// ------------------------------------------------------------
static const char CSV_HEADER[] =
    "rx_ms,t_ms,seq,lost,rssi,frame,"
    "mode,alt_state,armed,flow_ok,range_ok,range_valid,"
    "alt_en,alt_act,pos_hold,airborne,dry_run,sat,tx_drop,"
    "thr,bad,"
    "roll,pitch,yaw,"
    "range_h,range_raw,alt_hold,climb,alt_vzt,alt_corr,alt_thr,"
    "fh_vxc,fh_vyc,fh_vxt,fh_vyt,"
    "fh_posn,fh_pose,fh_holdn,fh_holde,"
    "fh_leanr,fh_leanp,"
    "m1,m2,m3,m4,mixsat,"
    "roll_gyr,pitch_gyr,yaw_gyr,roll_ratetar,pitch_ratetar,"
    "roll_cmd,pitch_cmd,yaw_cmd";

// ------------------------------------------------------------
//  状態
// ------------------------------------------------------------
static bool csv_on = false;

static S5T::AltFrame   last_alt{};
static S5T::PosFrame   last_pos{};
static S5T::AttFrame   last_att{};
static S5T::ParamFrame last_param{};
static bool  have_alt = false, have_pos = false, have_att = false, have_param = false;
static int   last_rssi  = -1;
static uint32_t last_rx_ms = 0;

// 機体時刻の展開。t_cs は 10ms 単位の uint16 なので 655.36 秒で一周する。
static uint32_t t_ms_unwrapped = 0;
static uint16_t t_cs_prev = 0;
static bool     t_init = false;

// 受信統計
// ★ n_rx_bytes / n_rx_lines は「IM920 から素のバイトが来ているか」の切り分け用。
//   0 のまま = 配線か電源かボーレート。増えているのに n_alt+n_pos が 0 =
//   相手が違う / パケット定義がずれている、と原因を分けられる。
static uint32_t n_rx_bytes = 0, n_rx_lines = 0;
static bool     raw_dump = false;   // 'd' で IM920 の生の行をそのまま出す
static uint32_t n_alt = 0, n_pos = 0, n_att = 0, n_param = 0;
static uint32_t n_lost  = 0;   // seq の飛びから数えた累積欠落
static uint32_t n_bad_cs = 0;  // チェックサム不一致
static uint32_t n_bad_len = 0; // 長さ / type が合わない (構造体バージョン違い?)
static bool     seq_init = false;
static uint8_t  prev_seq = 0;

static String rx_line;

// ------------------------------------------------------------
//  ユーティリティ
// ------------------------------------------------------------
static int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static const char* modeName(uint8_t m) {
    switch (m) {
        case 0: return "RATE";
        case 1: return "ANGLE";
        case 2: return "AUTO";
        case 3: return "POSHOLD";
    }
    return "?";
}

static const char* altStateName(uint8_t s) {
    switch (s) {
        case 0: return "OFF";
        case 1: return "STANDBY";
        case 2: return "NO_HOVER_THR";
        case 3: return "NO_RANGE";
        case 4: return "HOLDING";
        case 5: return "RANGE_LOST";
    }
    return "?";
}

static inline int flg(uint16_t f, uint16_t m) { return (f & m) ? 1 : 0; }

// 直近に届いたフレームのヘッダ (A/B/C どれも flags と modes を積んでいる)。
//  最後に受け取ったものを覚えておくだけ。seq の大小比較だと 255->0 の
//  折り返しで古いほうを選んでしまう。
static S5T::Header live_h{};
static uint8_t     live_modes = 0;
static const S5T::Header& liveHeader() { return live_h; }
static uint8_t liveModes()             { return live_modes; }

// t_cs (10ms単位 uint16) を単調増加の ms へ展開する
static uint32_t unwrapTime(uint16_t t_cs) {
    if (!t_init) { t_init = true; t_cs_prev = t_cs; t_ms_unwrapped = (uint32_t)t_cs * 10u; }
    else {
        uint16_t d = (uint16_t)(t_cs - t_cs_prev);   // 巻き戻しは自然に扱える
        t_cs_prev = t_cs;
        t_ms_unwrapped += (uint32_t)d * 10u;
    }
    return t_ms_unwrapped;
}

// ------------------------------------------------------------
//  CSV 出力
// ------------------------------------------------------------
static void startCsv() {
    Serial.println();
    Serial.print("HEADER,"); Serial.println(CSV_HEADER);
    Serial.println("LOG_START");
    csv_on = true;
}

static void stopCsv() {
    csv_on = false;
    Serial.println("LOG_STOP");
    Serial.printf("# A=%lu B=%lu C=%lu param=%lu lost=%lu badcs=%lu badlen=%lu\n",
                  (unsigned long)n_alt, (unsigned long)n_pos, (unsigned long)n_att,
                  (unsigned long)n_param, (unsigned long)n_lost,
                  (unsigned long)n_bad_cs, (unsigned long)n_bad_len);
}

static void emitParam(const S5T::ParamFrame& p) {
    Serial.printf("PARAM,ver=%u,alt_en=%d,dry=%d,sonar=%d,stick_vz=%d,"
                  "flow_vel_kp=%.3f,flow_vel_ki=%.3f,flow_vel_kd=%.3f,flow_pos_kp=%.3f,"
                  "alt_pos_kp=%.3f,alt_rate_kp=%.3f,alt_rate_ki=%.3f,alt_rate_kd=%.3f,"
                  "alt_hover_thr=%.3f,alt_target_m=%.3f,flow_max_lean=%.3f,alt_thr_auth=%.3f\n",
                  (unsigned)p.ver,
                  flg(p.cfg_flags, S5T::PF_ALT_HOLD_EN),
                  flg(p.cfg_flags, S5T::PF_DRY_RUN),
                  flg(p.cfg_flags, S5T::PF_SONAR),
                  flg(p.cfg_flags, S5T::PF_STICK_VZ),
                  p.flow_vel_kp / S5T::SC_GAIN, p.flow_vel_ki / S5T::SC_GAIN,
                  p.flow_vel_kd / S5T::SC_GAIN, p.flow_pos_kp / S5T::SC_GAIN,
                  p.alt_pos_kp  / S5T::SC_GAIN, p.alt_rate_kp / S5T::SC_GAIN,
                  p.alt_rate_ki / S5T::SC_GAIN, p.alt_rate_kd / S5T::SC_GAIN,
                  p.alt_hover_thr / S5T::SC_GAIN, p.alt_target_m / S5T::SC_GAIN,
                  p.flow_max_lean / S5T::SC_GAIN, p.alt_thr_auth / S5T::SC_GAIN);
}

// 量子化を物理値へ戻して 1行書く。CSV_HEADER と同じ順であること。
//  fresh: 今回どのフレームが届いたか (0:A 1:B 2:C)。他は前回値。
static void emitData(uint32_t rx_ms, uint32_t t_ms, uint8_t seq, int fresh) {
    const S5T::AltFrame& a = last_alt;
    const S5T::PosFrame& b = last_pos;
    const S5T::AttFrame& c = last_att;
    const uint16_t f = liveHeader().flags;
    const uint8_t  m = liveModes();

    Serial.printf(
        "DATA,%lu,%lu,%u,%lu,%d,%d,"
        "%u,%u,%d,%d,%d,%d,"
        "%d,%d,%d,%d,%d,%d,%d,"
        "%.3f,%u,"
        "%.2f,%.2f,%.1f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,"
        "%.3f,%.3f,%.3f,%.3f,%u,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%.4f,%.4f,%.4f\n",
        (unsigned long)rx_ms, (unsigned long)t_ms, (unsigned)seq,
        (unsigned long)n_lost, last_rssi, fresh,
        (unsigned)S5T::unpackMode(m), (unsigned)S5T::unpackAltState(m),
        flg(f, S5T::F_ARMED), flg(f, S5T::F_FLOW_OK),
        flg(f, S5T::F_RANGE_OK), flg(f, S5T::F_RANGE_VALID),
        flg(f, S5T::F_ALT_EN), flg(f, S5T::F_ALT_ACT),
        flg(f, S5T::F_POS_HOLD), flg(f, S5T::F_AIRBORNE),
        flg(f, S5T::F_DRY_RUN), flg(f, S5T::F_SAT), flg(f, S5T::F_TX_DROP),
        a.thr / 250.0f, (unsigned)b.bad,
        a.roll_cd / S5T::SC_CDEG, a.pitch_cd / S5T::SC_CDEG,
        a.yaw_dd / S5T::SC_DDEG,
        a.range_h_mm / S5T::SC_MM, a.range_raw_mm / S5T::SC_MM,
        a.alt_hold_mm / S5T::SC_MM, a.climb_mmps / S5T::SC_MM,
        a.alt_vz_tar_mmps / S5T::SC_MM,
        a.alt_thr_corr / S5T::SC_1E4, a.alt_thr_out / S5T::SC_1E4,
        b.vx_mmps / S5T::SC_MM, b.vy_mmps / S5T::SC_MM,
        b.vx_tar_mmps / S5T::SC_MM, b.vy_tar_mmps / S5T::SC_MM,
        b.pos_n_cm / S5T::SC_CM, b.pos_e_cm / S5T::SC_CM,
        b.hold_n_cm / S5T::SC_CM, b.hold_e_cm / S5T::SC_CM,
        b.lean_roll_cd / S5T::SC_CDEG, b.lean_pitch_cd / S5T::SC_CDEG,
        c.m1 / 250.0f, c.m2 / 250.0f, c.m3 / 250.0f, c.m4 / 250.0f,
        (unsigned)c.sat,
        c.roll_rate_dd / S5T::SC_DDEG, c.pitch_rate_dd / S5T::SC_DDEG,
        c.yaw_rate_dd / S5T::SC_DDEG,
        c.roll_rate_tar_dd / S5T::SC_DDEG, c.pitch_rate_tar_dd / S5T::SC_DDEG,
        c.roll_cmd / S5T::SC_1E4, c.pitch_cmd / S5T::SC_1E4,
        c.yaw_cmd / S5T::SC_1E4);
}

// ------------------------------------------------------------
//  人間向けの表示 (CSV を止めているときだけ)
// ------------------------------------------------------------
//  ★ ANSI のクリア (\033[2J) は使わない。PlatformIO / VSCode のシリアル
//    モニタでは処理されずに画面が崩れるうえ、崩れた画面と「キーが効いて
//    いない」の区別が付かなくなるため。ただ下へ流していく。
static void printStatus() {
    Serial.println();
    Serial.println("---- s5 TELEMETRY RECEIVER  [l]=CSV出力 [d]=生データ [z]=統計クリア [h]=help ----");
    const uint32_t age = millis() - last_rx_ms;
    const bool live = (have_alt || have_pos || have_att);
    Serial.printf("link : %s  (最終受信 %lu ms前)  RSSI=%d\n",
                  (live && age < 1000) ? "OK" : "LOST",
                  (unsigned long)(live ? age : 0), last_rssi);
    Serial.printf("IM920 生受信: %lu bytes / %lu 行\n",
                  (unsigned long)n_rx_bytes, (unsigned long)n_rx_lines);
    Serial.printf("stats: A=%lu B=%lu C=%lu P=%lu  lost=%lu  badcs=%lu badlen=%lu",
                  (unsigned long)n_alt, (unsigned long)n_pos, (unsigned long)n_att,
                  (unsigned long)n_param, (unsigned long)n_lost,
                  (unsigned long)n_bad_cs, (unsigned long)n_bad_len);
    const uint32_t tot = n_alt + n_pos + n_att + n_param + n_lost;
    if (tot) Serial.printf("   欠落率 %.1f%%", 100.0f * (float)n_lost / (float)tot);
    Serial.println();

    if (!live) {
        Serial.println("まだ1パケットも受信していません。");
        if (n_rx_bytes == 0) {
            Serial.println("  IM920 から1バイトも来ていません。無線ではなく配線側の問題です:");
            Serial.println("   ・IM920 の TXD が D7(GP1) に来ているか (im920_passthrough の PINS)");
            Serial.println("   ・IM920 の電源は 3V3 か (5Vピンだと書き込みも不安定になる)");
            Serial.println("   ・ボーレート 19200 か");
        } else {
            Serial.println("  バイトは来ているのでリンク自体は生きています。");
            Serial.println("   ・GN(グループ番号)/CH/ホップ(ENHP/DSHP) が両機で同じか (RPRM)");
            Serial.println("   ・[d] で生の行を見て、中身が想定どおりか確認する");
            Serial.println("   ・badlen が増える = S5Telem.h が機体側とずれている");
        }
        return;
    }

    const uint16_t f = liveHeader().flags;
    const uint8_t  m = liveModes();
    Serial.printf("\nMODE=%-8s  ALT=%-12s  %s%s%s\n",
                  modeName(S5T::unpackMode(m)),
                  altStateName(S5T::unpackAltState(m)),
                  flg(f, S5T::F_ARMED) ? "ARMED " : "DISARMED ",
                  flg(f, S5T::F_DRY_RUN) ? "[DRY-RUN] " : "",
                  flg(f, S5T::F_SAT) ? "[MIX-SAT] " : "");

    if (have_alt) {
        const S5T::AltFrame& a = last_alt;
        Serial.printf("att  : roll=%+7.2f pitch=%+7.2f yaw=%+7.1f [deg]   thr_stick=%.3f\n",
                      a.roll_cd / S5T::SC_CDEG, a.pitch_cd / S5T::SC_CDEG,
                      a.yaw_dd / S5T::SC_DDEG, a.thr / 250.0f);
        Serial.println("[高度ホールド]");
        Serial.printf("  h=%.3f m (生 %.3f)  target=%.3f m  err=%+.3f m   range %s\n",
                      a.range_h_mm / S5T::SC_MM, a.range_raw_mm / S5T::SC_MM,
                      a.alt_hold_mm / S5T::SC_MM,
                      (a.alt_hold_mm - a.range_h_mm) / S5T::SC_MM,
                      flg(f, S5T::F_RANGE_VALID) ? "valid" : "LOST");
        Serial.printf("  vz  : 実測 %+.3f / 目標 %+.3f [m/s]\n",
                      a.climb_mmps / S5T::SC_MM, a.alt_vz_tar_mmps / S5T::SC_MM);
        Serial.printf("  thr : out=%.3f  corr=%+.4f   %s\n",
                      a.alt_thr_out / S5T::SC_1E4, a.alt_thr_corr / S5T::SC_1E4,
                      flg(f, S5T::F_ALT_ACT) ? "ACTIVE" : "(手動)");
    }

    if (have_pos) {
        const S5T::PosFrame& b = last_pos;
        Serial.println("[水平位置ホールド]");
        Serial.printf("  v   : 実測 vx=%+.3f vy=%+.3f / 目標 vx=%+.3f vy=%+.3f [m/s]\n",
                      b.vx_mmps / S5T::SC_MM, b.vy_mmps / S5T::SC_MM,
                      b.vx_tar_mmps / S5T::SC_MM, b.vy_tar_mmps / S5T::SC_MM);
        const float dn = (b.pos_n_cm - b.hold_n_cm) / S5T::SC_CM;
        const float de = (b.pos_e_cm - b.hold_e_cm) / S5T::SC_CM;
        Serial.printf("  pos : N=%+.2f E=%+.2f / hold N=%+.2f E=%+.2f [m]  ずれ %.2f m\n",
                      b.pos_n_cm / S5T::SC_CM, b.pos_e_cm / S5T::SC_CM,
                      b.hold_n_cm / S5T::SC_CM, b.hold_e_cm / S5T::SC_CM,
                      sqrtf(dn * dn + de * de));
        Serial.printf("  lean: roll=%+.2f pitch=%+.2f [deg]  bad=%u  %s\n",
                      b.lean_roll_cd / S5T::SC_CDEG, b.lean_pitch_cd / S5T::SC_CDEG,
                      (unsigned)b.bad,
                      flg(f, S5T::F_POS_HOLD) ? "HOLDING" : "(スティック操作中)");
    }

    if (have_att) {
        const S5T::AttFrame& c = last_att;
        Serial.println("[姿勢ループ]");
        Serial.printf("  motor: M1=%.3f M2=%.3f M3=%.3f M4=%.3f   飽和=%c%c%c%c\n",
                      c.m1 / 250.0f, c.m2 / 250.0f, c.m3 / 250.0f, c.m4 / 250.0f,
                      (c.sat & 1) ? '1' : '-', (c.sat & 2) ? '2' : '-',
                      (c.sat & 4) ? '3' : '-', (c.sat & 8) ? '4' : '-');
        Serial.printf("  rate : 実測 r=%+7.1f p=%+7.1f y=%+7.1f / 目標 r=%+7.1f p=%+7.1f [deg/s]\n",
                      c.roll_rate_dd / S5T::SC_DDEG, c.pitch_rate_dd / S5T::SC_DDEG,
                      c.yaw_rate_dd / S5T::SC_DDEG,
                      c.roll_rate_tar_dd / S5T::SC_DDEG,
                      c.pitch_rate_tar_dd / S5T::SC_DDEG);
        Serial.printf("  cmd  : roll=%+.4f pitch=%+.4f yaw=%+.4f\n",
                      c.roll_cmd / S5T::SC_1E4, c.pitch_cmd / S5T::SC_1E4,
                      c.yaw_cmd / S5T::SC_1E4);
    }

    if (have_param) {
        Serial.println("[ゲイン (機体から受信)]");
        Serial.printf("  flow vel P=%.3f I=%.3f D=%.3f   flow pos P=%.3f\n",
                      last_param.flow_vel_kp / S5T::SC_GAIN,
                      last_param.flow_vel_ki / S5T::SC_GAIN,
                      last_param.flow_vel_kd / S5T::SC_GAIN,
                      last_param.flow_pos_kp / S5T::SC_GAIN);
        Serial.printf("  alt  pos P=%.3f   alt rate P=%.3f I=%.3f D=%.3f  hover=%.3f\n",
                      last_param.alt_pos_kp  / S5T::SC_GAIN,
                      last_param.alt_rate_kp / S5T::SC_GAIN,
                      last_param.alt_rate_ki / S5T::SC_GAIN,
                      last_param.alt_rate_kd / S5T::SC_GAIN,
                      last_param.alt_hover_thr / S5T::SC_GAIN);
    }
}

static void printHelp() {
    Serial.println("# --- s5 telemetry receiver ---");
    Serial.println("#   1 : CSV 出力 ON    0 : CSV 出力 OFF  (冪等。logger.py はこれを使う)");
    Serial.println("#   l : CSV 出力のトグル (人間用)");
    Serial.println("#       ★これは「画面に CSV を吐く」だけ。ファイルを作るのは PC 側の");
    Serial.println("#         python tools/s5_logger.py です (起動時に自動で l を送ります)");
    Serial.println("#   s : 状態を1回表示");
    Serial.println("#   d : IM920 の生の行をそのまま表示 (リンクの切り分け用)");
    Serial.println("#   z : 統計クリア");
    Serial.println("#   h : このヘルプ");
    Serial.printf ("#   ver=%u  A=%u B=%u P=%u byte (+checksum4 = %u, IM920sL上限 %u)\n",
                   (unsigned)S5T::VERSION,
                   (unsigned)sizeof(S5T::AltFrame), (unsigned)sizeof(S5T::PosFrame),
                   (unsigned)sizeof(S5T::ParamFrame),
                   (unsigned)(sizeof(S5T::AltFrame) + S5T::CHECKSUM_BYTES),
                   (unsigned)S5T::IM920SL_MAX_PAYLOAD);
}

// ------------------------------------------------------------
//  1行ぶんの受信処理
//    IM920 の受信行は  "<ノード>,<モジュールID>,<RSSI>:<データ16進>"
//    データ部はモジュール設定によってバイト間にカンマが入るので落とす。
// ------------------------------------------------------------
static void handleLine(String& line) {
    n_rx_lines++;
    if (raw_dump) { Serial.print("# RAW "); Serial.println(line); }

    const int colon = line.indexOf(':');
    if (colon < 0) {
        // "OK" / "NG" / 起動メッセージなど。PC 側が読み飛ばせるよう # を付ける。
        line.trim();
        if (line.length()) { Serial.print("# "); Serial.println(line); }
        return;
    }

    // --- ヘッダ部から RSSI を拾う (3番目のフィールド) ---
    int rssi = -1;
    {
        String head = line.substring(0, colon);
        const int c1 = head.indexOf(',');
        const int c2 = (c1 >= 0) ? head.indexOf(',', c1 + 1) : -1;
        if (c2 >= 0) {
            String r = head.substring(c2 + 1);
            r.trim();
            rssi = (int)strtoul(r.c_str(), nullptr, 16);
        }
    }

    // --- データ部を16進デコード ---
    String data = line.substring(colon + 1);
    data.replace(",", "");
    data.trim();

    static uint8_t buf[64];
    const int n_hex = data.length();
    if ((n_hex & 1) || n_hex < 16 || (n_hex / 2) > (int)sizeof(buf)) {
        n_bad_len++;
        return;
    }
    const int n = n_hex / 2;
    for (int i = 0; i < n; ++i) {
        const int hi = hexVal(data[2 * i]), lo = hexVal(data[2 * i + 1]);
        if (hi < 0 || lo < 0) { n_bad_len++; return; }
        buf[i] = (uint8_t)((hi << 4) | lo);
    }

    // --- チェックサム (末尾4バイト。データ総和 + 値 == 0) ---
    const int payload = n - (int)S5T::CHECKSUM_BYTES;
    uint32_t sum = 0;
    for (int i = 0; i < payload; ++i) sum += buf[i];
    uint32_t cs = 0;
    memcpy(&cs, buf + payload, S5T::CHECKSUM_BYTES);
    if ((uint32_t)(sum + cs) != 0u) { n_bad_cs++; return; }

    last_rx_ms = millis();
    last_rssi  = rssi;

    if (payload != (int)S5T::PACKET_BYTES) {
        n_bad_len++;
        if (n_bad_len <= 3)
            Serial.printf("# 長さ不一致 type=0x%02X len=%d (期待 %u)\n",
                          buf[0], payload, (unsigned)S5T::PACKET_BYTES);
        return;
    }

    // --- seq の飛びで欠落を数える (255 -> 0 の折り返しも自然に扱える) ---
    const uint8_t seq = buf[1];
    if (seq_init) {
        const uint8_t gap = (uint8_t)(seq - prev_seq);
        if (gap > 1) n_lost += (uint32_t)(gap - 1);
    }
    prev_seq = seq;
    seq_init = true;

    switch (buf[0]) {
        case S5T::TYPE_ALT: {
            memcpy(&last_alt, buf, sizeof(last_alt));
            have_alt = true;
            n_alt++;
            live_h = last_alt.h;  live_modes = last_alt.modes;
            const uint32_t t = unwrapTime(last_alt.h.t_cs);
            if (csv_on) emitData(last_rx_ms, t, seq, 0);
            break;
        }
        case S5T::TYPE_POS: {
            memcpy(&last_pos, buf, sizeof(last_pos));
            have_pos = true;
            n_pos++;
            live_h = last_pos.h;  live_modes = last_pos.modes;
            const uint32_t t = unwrapTime(last_pos.h.t_cs);
            if (csv_on) emitData(last_rx_ms, t, seq, 1);
            break;
        }
        case S5T::TYPE_ATT: {
            memcpy(&last_att, buf, sizeof(last_att));
            have_att = true;
            n_att++;
            live_h = last_att.h;  live_modes = last_att.modes;
            const uint32_t t = unwrapTime(last_att.h.t_cs);
            if (csv_on) emitData(last_rx_ms, t, seq, 2);
            break;
        }
        case S5T::TYPE_PARAM: {
            memcpy(&last_param, buf, sizeof(last_param));
            have_param = true;
            n_param++;
            if (last_param.ver != S5T::VERSION) {
                Serial.printf("# !! パケットバージョン不一致: 機体=%u 地上=%u  "
                              "S5Telem.h を両側そろえて焼き直すこと\n",
                              (unsigned)last_param.ver, (unsigned)S5T::VERSION);
            }
            if (csv_on) emitParam(last_param);
            break;
        }
        default:
            n_bad_len++;
            if (n_bad_len <= 3)
                Serial.printf("# 未知の type=0x%02X (期待 A=0x%02X B=0x%02X C=0x%02X P=0x%02X)\n",
                              buf[0], S5T::TYPE_ALT, S5T::TYPE_POS,
                              S5T::TYPE_ATT, S5T::TYPE_PARAM);
            break;
    }
}

// ------------------------------------------------------------
//  キー入力
// ------------------------------------------------------------
//  ★ どのキーにも必ず1行返す。無反応だと「キーが届いていない」のか
//    「届いたが機体からパケットが来ていない」のか区別できないため。
static void csvOn() {
    if (csv_on) {
        // 既に出力中。HEADER を出し直して PC 側と足並みをそろえる。
        Serial.println("# CSV 出力は既に ON。HEADER を再送します");
    } else {
        Serial.println("# CSV 出力 ON。"
                       "※ファイルを作るのは tools/s5_logger.py です");
    }
    startCsv();
    if (have_param) emitParam(last_param);
    if (!have_alt && !have_pos && !have_att)
        Serial.println("# まだテレメトリが来ていないので DATA 行は出ません "
                       "(2秒ごとに待機中の表示を出します)");
}

static void csvOff() {
    if (!csv_on) { Serial.println("# CSV 出力は既に OFF"); return; }
    stopCsv();
}

static void handleKey(char c) {
    if (c == '\r' || c == '\n' || c == ' ') return;   // 改行は無視 (返事もしない)
    switch (c) {
        // ★ '1'/'0' は冪等な ON/OFF。s5_logger.py はこちらを使う。
        //   'l' のトグルだと、受信機が既に出力中のときに logger を起動した
        //   場合に「開始のつもりが停止」になり、0行のログができてしまう。
        case '1': csvOn();  break;
        case '0': csvOff(); break;

        case 'l': case 'L':
            csv_on ? csvOff() : csvOn();
            break;
        case 's': case 'S':
            printStatus();
            break;
        case 'd': case 'D':
            raw_dump = !raw_dump;
            Serial.printf("# 生データ表示 = %s\n", raw_dump ? "ON" : "OFF");
            break;
        case 'z': case 'Z':
            n_rx_bytes = n_rx_lines = 0;
            n_alt = n_pos = n_att = n_param = n_lost = n_bad_cs = n_bad_len = 0;
            seq_init = false;
            Serial.println("# 統計をクリアしました");
            break;
        case 'h': case 'H': case '?':
            printHelp();
            break;
        default:
            Serial.printf("# 未知のキー '%c' (0x%02X)。使えるのは 1 / 0 / l / s / d / z / h\n",
                          (c >= 0x20 && c < 0x7f) ? c : '?', (uint8_t)c);
            break;
    }
}

// ------------------------------------------------------------
void setup() {
    Serial.begin(USB_BAUD);
#if defined(ARDUINO_ARCH_RP2040)
    Serial1.setTX(PIN_XIAO_TX);
    Serial1.setRX(PIN_XIAO_RX);
#endif
    IM->begin(IM_BAUD);
    rx_line.reserve(256);

    while (!Serial && millis() < 3000) {}
    Serial.println();
    Serial.println("# === s5 telemetry receiver / logger ===");
    printHelp();
}

void loop() {
    // --- IM920 から1行ずつ取り込む ---
    while (IM->available()) {
        const char c = (char)IM->read();
        n_rx_bytes++;
        if (c == '\n') {
            handleLine(rx_line);
            rx_line = "";
        } else if (c != '\r') {
            if (rx_line.length() < 250) rx_line += c;
            else { rx_line = ""; n_bad_len++; }   // 長すぎる = 化けている
        }
    }

    // --- キー入力 ---
    while (Serial.available()) handleKey((char)Serial.read());

    const uint32_t now = millis();

    // --- CSV を出していないときは、人間向けの状態を 1秒ごとに流す ---
    if (!csv_on) {
        static uint32_t last_draw = 0;
        if (now - last_draw >= 1000) { last_draw = now; printStatus(); }
        return;
    }

    // --- CSV 中にテレメトリが途切れたら、黙り込まずに知らせる ---
    //   '#' 始まりなので s5_logger.py は CSV に書かず、画面にだけ出す。
    {
        static uint32_t last_beat = 0;
        if (now - last_rx_ms > 2000 && now - last_beat >= 2000) {
            last_beat = now;
            Serial.printf("# テレメトリ待機中: IM920 %lu bytes / %lu 行, "
                          "A=%lu B=%lu C=%lu badcs=%lu badlen=%lu\n",
                          (unsigned long)n_rx_bytes, (unsigned long)n_rx_lines,
                          (unsigned long)n_alt, (unsigned long)n_pos,
                          (unsigned long)n_att,
                          (unsigned long)n_bad_cs, (unsigned long)n_bad_len);
        }
    }
}
