// ============================================================
//  esc_calib.cpp  -  ESCキャリブレーション専用ファーム
// ============================================================
//  ★★★ 必ずプロペラを外して実行してください ★★★
//
//  【なぜ専用ファームなのか】
//    ESCがキャリブレーションモードに入るのは、
//    「ESCに電源が入った瞬間に フルスロットル信号 が来ている」ときだけ。
//
//    通常の drone.cpp は起動に数秒かかる上、シリアルで対話する作りだった。
//    Teensyの電源をバッテリーから分離していると、バッテリーを挿した瞬間に
//    シリアルモニタが再接続されてしまい、対話しながらの手順が成立しない。
//
//    そこでこのファームは シリアルを一切使わずに、
//    起動したら即座に 2000us を出し続ける。
//    バッテリーを挿すタイミングは人間が自由に選べる。
//
//  【手順】
//    1. プロペラを外す
//    2. バッテリーを抜く (TeensyはUSB給電のまま)
//    3. このファームを書き込む
//         pio run -e esc_calib -t upload
//    4. 書き込み直後(またはTeensyのリセットボタン)から
//       HIGH_MS の間、全ピンに 2000us が出続ける
//    5. その間に バッテリーを挿す
//       → ESCが「ピピッ」と鳴って上限を記録
//    6. HIGH_MS 経過で自動的に 1000us に落ちる
//       → ESCが鳴って下限を記録 → 完了
//    7. LEDが速い点滅に変わったら終わり。バッテリーを抜く
//    8. ★必ず drone に戻す★
//         pio run -e drone -t upload
//
//  【LEDの見方】(Teensy内蔵LED = pin 13)
//    点灯しっぱなし : 2000us 出力中 (この間にバッテリーを挿す)
//    ゆっくり点滅   : 1000us 出力中 (下限の記録待ち)
//    速い点滅       : 完了。以後ずっと 1000us を保持
// ============================================================

#include <Arduino.h>

// ============================================================
//  設定
// ============================================================
namespace Cal {

// モーターの信号ピン。drone.cpp と揃えること。
constexpr int PIN[] = { 0, 1, 2, 3 };
constexpr int COUNT = sizeof(PIN) / sizeof(PIN[0]);

constexpr int PWM_HZ  = 400;   // drone.cpp と同じ
constexpr int PWM_BITS = 12;   // 12bit = 0..4095

// 400Hz / 12bit のとき、1周期は 2500us。
//   1000us → 4096 * 1000/2500 = 1638
//   2000us → 4096 * 2000/2500 = 3277
// (Actuators.cpp の motor::write と同じ値)
constexpr int PWM_MIN = 1638;  // 1000us = スロットル0
constexpr int PWM_MAX = 3277;  // 2000us = フルスロットル

// フルスロットルを出し続ける時間 [ms]
// この間にバッテリーを挿す。余裕を持って長めにしてある。
constexpr uint32_t HIGH_MS = 10000;

// スロットル0を保持して、ESCの確認音を待つ時間 [ms]
constexpr uint32_t LOW_MS = 6000;

constexpr int LED_PIN = 13;

} // namespace Cal

// ============================================================
//  出力
// ============================================================
static void writeAll(int pwm_val) {
    for (int i = 0; i < Cal::COUNT; ++i) {
        analogWrite(Cal::PIN[i], pwm_val);
    }
}

// 指定時間だけ待ちながらLEDを点滅させる。period_ms = 0 なら点灯しっぱなし。
static void holdWithBlink(uint32_t duration_ms, uint32_t period_ms) {
    const uint32_t start = millis();
    while (millis() - start < duration_ms) {
        if (period_ms == 0) {
            digitalWrite(Cal::LED_PIN, HIGH);
        } else {
            digitalWrite(Cal::LED_PIN,
                         ((millis() / period_ms) % 2) ? HIGH : LOW);
        }
    }
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
    pinMode(Cal::LED_PIN, OUTPUT);
    digitalWrite(Cal::LED_PIN, LOW);

    // --- PWMの準備 ---
    // ★ここでは待たない。シリアルの接続を待つと、その間ESCに信号が出ず
    //   キャリブレーションのタイミングを逃す。
    analogWriteResolution(Cal::PWM_BITS);
    for (int i = 0; i < Cal::COUNT; ++i) {
        pinMode(Cal::PIN[i], OUTPUT);
        analogWriteFrequency(Cal::PIN[i], Cal::PWM_HZ);
    }

    // --- 1) 即座にフルスロットル ---
    //     この間にバッテリーを挿す
    writeAll(Cal::PWM_MAX);
    holdWithBlink(Cal::HIGH_MS, 0);          // 点灯しっぱなし

    // --- 2) スロットル0へ ---
    writeAll(Cal::PWM_MIN);
    holdWithBlink(Cal::LOW_MS, 400);         // ゆっくり点滅

    // --- 3) 完了。以後ずっと 1000us を保持する ---
    writeAll(Cal::PWM_MIN);

    // シリアルが繋がっていれば結果を出す (繋がっていなくても止まらない)
    Serial.begin(115200);
    Serial.println("\n=== ESC calibration sequence finished ===");
    Serial.println("バッテリーを抜いて、drone を書き込み直してください:");
    Serial.println("  pio run -e drone -t upload");
}

void loop() {
    // 完了の合図: 速い点滅。出力は 1000us のまま。
    digitalWrite(Cal::LED_PIN, ((millis() / 100) % 2) ? HIGH : LOW);
    writeAll(Cal::PWM_MIN);
    delay(20);
}
