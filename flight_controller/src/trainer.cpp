// ============================================================
// trainer.cpp  -  トレーナー機 (Seeed XIAO RP2040版)
// ============================================================
//
// [移植メモ]
//  Teensy4.0 -> Seeed XIAO RP2040 への移植で変更した点:
//   1) Serial4 / Serial5 (Teensyの追加ハードウェアUART) が存在しないため、
//      教官側(Main_sbus)は Serial1 (本物のUART0, D6/D7固定) を使用。
//      生徒側(sbus)は SerialPIO (PIOベースの疑似UART, RXのみ) を D5 に割当。
//   2) SBUSは反転シリアルだが、RP2040のUARTはハード的に反転非対応。
//      ただしarduino-picoコアの setInvertRX(true) でGPIOレベル反転が
//      できるため、外部反転回路(トランジスタ等)は不要。
//      -> Sbus / SerialPIO の begin() が呼ばれる"前"に必ず呼ぶこと。
//   3) XIAO RP2040 は使えるGPIOが11本 (D0-D10) しかないため、
//      ピン配置を下記のとおり詰め直した。全ピンPWM対応なのでサーボは
//      どこに割り当てても問題ない。
//
//      D6(GPIO0) : Main_sbus (Serial1 TX, 未使用だが予約される)
//      D7(GPIO1) : Main_sbus (Serial1 RX)
//      D5(GPIO7) : sbus      (SerialPIO RX, TXはNOPIN)
//      D0(GPIO26): Ail1
//      D1(GPIO27): Ail2
//      D2(GPIO28): Ele
//      D3(GPIO29): Rud
//      D4(GPIO6) : Flp1
//      D8(GPIO2) : Flp2
//      D9(GPIO4) : Thr
//      D10(GPIO3): 予備
//
//   4) Teensy-DShotライブラリはこのファイルでは未使用 (Thrは通常PWM)
//      なので今回の移植では問題なし。
//   5) Receiver.h の Sbus クラスは bfs::SbusRx(ser, true) を使うのみで
//      RP2040向けの変更は不要と確認済み。ただし反転はライブラリ側では
//      行わないため、setInvertRX()を必ずbegin()より前に呼ぶこと
//      (下のsetup()内の呼び出し順を変えないこと)。
//   6) Actuators.h は drone.cpp が使うため無変更。このファイルでは
//      Actuators.h/.cppの内容(DShot部分を除く)を直接展開している。
// ============================================================

#include <Arduino.h>
#include <math.h>

#include <cctype>
#include <cmath>

#include "Config.h"

// ------------------------------------------------------------
// [XIAO RP2040 / trainer専用]
// Actuators.h / Actuators.cpp は drone.cpp が使い続けるため変更せず、
// このファイルには同等の内容をDShot依存なしで直接展開しています。
// (Actuators.hはServo.hに加えてTeensy専用のDShot.hを無条件includeして
//  いるため、そのままincludeするとRP2040ビルドが通らない)
// ------------------------------------------------------------
#include <Servo.h>

template <typename T>
class Actuator {
  protected:
    bool _built = false;
    float _off = 0, _end_1 = -1.0, _end_2 = 1.0;
    int _pin = -1, _minPWM = 600, _maxPWM = 2000;
    Servo _servo;

  public:
    T& set_pin(int pin)                  { if (!_built) _pin = pin;                 return static_cast<T&>(*this); }
    T& set_minPWM(int v)                 { if (!_built) _minPWM = v;                return static_cast<T&>(*this); }
    T& set_maxPWM(int v)                 { if (!_built) _maxPWM = v;                return static_cast<T&>(*this); }
    T& set_offset(float v)               { if (!_built) _off = v;                   return static_cast<T&>(*this); }
    T& set_endpoints(float e1, float e2) { if (!_built) { _end_1 = e1; _end_2 = e2; } return static_cast<T&>(*this); }

    void begin() {
        if (_pin == -1) return;
        _servo.attach(_pin, _minPWM, _maxPWM);
        _servo.writeMicroseconds(1000);
        _built = true;
    }
    bool is_ready() { return _built; }
};

class Norm_Servo : public Actuator<Norm_Servo> {
public:
    void write(float input);
};

class Elevon_Servo : public Actuator<Elevon_Servo> {
private:
    float p_ratio = 1.0, r_ratio = 1.0;
public:
    Elevon_Servo& set_ratio(float p, float r) { if (!_built) { p_ratio = p; r_ratio = r; } return *this; }
    void write(float p_input, float r_input);
};

class Flap_Servo : public Actuator<Flap_Servo> {
public:
    void write(Sw input);
};

class motor : public Actuator<motor> {
public:
    void write(float input);
};

// --- Actuators.cpp の内容をそのまま移植 (関数名衝突を避けるためstatic化) ---
static float trainer_s_map(float input, float offset, float end1, float end2) {
    if (input > 0) return offset + input * (end2 - offset);
    else           return offset + input * (offset - end1);
}

static int trainer_s_float_to_microseconds(float input, int minPWM, int maxPWM) {
    float cenPWM = (minPWM + maxPWM) / 2;
    return (int)(input * (maxPWM - cenPWM) + cenPWM);
}

void Norm_Servo::write(float input) {
    if (is_ready())
        _servo.writeMicroseconds(
            trainer_s_float_to_microseconds(
                trainer_s_map(input, _off, _end_1, _end_2), _minPWM, _maxPWM));
}

void Elevon_Servo::write(float p_input, float r_input) {
    if (!is_ready()) return;
    float mixed = p_input * p_ratio + r_input * r_ratio;
    mixed = constrain(mixed, -1.0f, 1.0f);
    _servo.writeMicroseconds(
        trainer_s_float_to_microseconds(
            trainer_s_map(mixed, _off, _end_1, _end_2), _minPWM, _maxPWM));
}

void Flap_Servo::write(Sw input) {
    if (!is_ready()) return;
    float val = (input == up) ? 1.0f : (input == cen) ? 0.0f : -1.0f;
    _servo.writeMicroseconds(
        trainer_s_float_to_microseconds(
            trainer_s_map(val, _off, _end_1, _end_2), _minPWM, _maxPWM));
}

void motor::write(float input) {
    if (is_ready())
        _servo.writeMicroseconds(input * (_maxPWM - _minPWM) + _minPWM);
}

// 大量のヘッダファイル (Actuators.hは上で展開済みのためincludeしない)
#include "Control.h"
#include "Receiver.h"
#include "Sensors.h"
#include "Serial_com.h"
#include "Telemetry.h"
#include "flight_mode.h"

namespace T = Config::Timing;

// ============================================================
//  インスタンス生成
// ============================================================

// ============================================================
//  § 1  PIDゲイン
// ============================================================
Axis_value Roll(-0.005f, 0.0f, 0.0f, 15.0f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f,
                0.0f),
    Pitch(0.003f, 0.0f, 0.0f, 8.0f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f, 0.0f),
    Yaw(-0.03f, 0.0f, 0.0f, -1.5f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f, 0.0f);

float BANK_ANGLE = 25.0f;  // バンク角 [deg]
unsigned long TURN_MS = 4000UL;
float RUDDER_COORD = 0.66;

// ------------------------------------------------------------
// [XIAO RP2040] SBUS用シリアルポート
//   教官側: Serial1  (本物のハードウェアUART0, D6=TX / D7=RX 固定)
//   生徒側: SerialPIO (PIOベース疑似UART, RXのみ使用 / TX=NOPIN)
// ------------------------------------------------------------
SerialPIO SubReceiverSerial(NOPIN, D5);  // 生徒側SBUS (RXのみ, D5=GPIO7)

Sbus Main_sbus(&Serial1);           // 教官側SBUS
Sbus sbus(&SubReceiverSerial);      // 生徒側SBUS

Flight_mode Mode;

Norm_Servo Ail1, Ail2, Ele, Rud;
Flap_Servo Flp1, Flp2;
motor Thr;

// ============================================================
//  プロトタイプ宣言
// ============================================================
void updateSensorsAndComms();
void writeServos();
void reset_all();

// ============================================================
//  § 組み合わせテスト用フラグ
// ============================================================
bool USE_MPU = false;
bool USE_BARO = false;
bool USE_SBUS = true;
bool USE_IM920 = false;
bool USE_SERVO = true;

void setup() {
  // サーボの宣言 (XIAO RP2040 ピン配置)
  Ail1.set_pin(D0).set_endpoints(-1.0, 1.0).set_offset(-0.15).begin();

  Ail2.set_pin(D1)
      .set_endpoints(-1.0, 1.0)  // リバース
      .set_offset(-0.3)
      .begin();

  Ele.set_pin(D2).set_offset(0.5).set_endpoints(-1.0, 1.0).begin();

  Rud.set_pin(D3)
      .set_endpoints(-1.0, 1.0)  // リバース
      .begin();

  Flp1.set_pin(D4).set_endpoints(-1.0, 1.0).begin();

  Flp2.set_pin(D8).set_endpoints(-1.0, 1.0).begin();

  Thr.set_pin(D9).set_minPWM(1000).set_maxPWM(2000).begin();

  Serial.begin(115200);
  uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms < 2000));
  Serial.println("\n\n--- XIAO RP2040 SYSTEM BOOT (High-Power Triage) ---");

  if (USE_SBUS) {
    Serial.println("Init SBUS...");

    // ★重要★ begin()より前に反転設定をすること
    //  (arduino-pico: setInvertRX/TXは "_running==false" の間だけ有効)
    Serial1.setInvertRX(true);          // 教官側: SBUSは反転信号
    SubReceiverSerial.setInvertRX(true);  // 生徒側: SBUSは反転信号

    sbus.begin();
    Main_sbus.begin();
  }

  if (USE_SERVO) {
    Serial.println("Init Actuators...");
    Ail1.begin();
    Ail2.begin();
    Ele.begin();
    Rud.begin();
    Flp1.begin();
    Flp2.begin();
    Thr.begin();
  }

  Serial.println("--- Setup Complete. Loop start ---");
}

// ============================================================
//  loop
// ============================================================
void loop() {
  static uint32_t last_alive_ms = 0;
  if (millis() - last_alive_ms > 1000) {
    last_alive_ms = millis();
    Serial.printf("### XIAO RP2040 Alive - MPU:%s BARO:%s SBUS:%s ###\n",
                  USE_MPU ? "ON" : "OFF", USE_BARO ? "ON" : "OFF",
                  USE_SBUS ? "ON" : "OFF");
  }

  if (T::freq<T::MAIN_Hz>(T::Main_dt)) {  // 周期制御 (1000Hz)
    if (USE_MPU || USE_SBUS) {
      updateSensorsAndComms();
    }

    if (USE_SERVO) {
      writeServos();
    }

    if (USE_SERVO) {
      if (sbus.isSafe() || Main_sbus.isSafe()) {
        if (Main_sbus.Ch_state(Aux2) == up ||
            Main_sbus.Ch_state(THR_CUT) == up) {
          Thr.write(Main_sbus.des[Ch::THR]);
        } else if (Main_sbus.Ch_state(Aux2) == cen) {
          Thr.write(sbus.des[Ch::THR] * 0.7f + Main_sbus.des[Ch::THR] * 0.3f);
        } else {
          Thr.write(sbus.des[Ch::THR]);
        }
      } else {
        Thr.write(0);
      }
    }

    if (sbus.Ch_state(THR_CUT) == up) {
      Flp1.write(sbus.Ch_state(Aux1));
      Flp2.write(sbus.Ch_state(Aux1));
    } else {
      Flp1.write(Main_sbus.Ch_state(Aux1));
      Flp2.write(Main_sbus.Ch_state(Aux1));
    }

    static int dbg_cnt = 0;
    if (++dbg_cnt >= 100) {
      dbg_cnt = 0;

      Serial.print("\033[2J\033[H");
      Serial.printf("### MPU=%s BARO=%s SBUS=%s IM920=%s SERVO=%s ###\n",
                    USE_MPU ? "ON" : "OFF", USE_BARO ? "ON" : "OFF",
                    USE_SBUS ? "ON" : "OFF", USE_IM920 ? "ON" : "OFF",
                    USE_SERVO ? "ON" : "OFF");
      print_flightmode(Mode.get_mode(), BANK_ANGLE, TURN_MS);
      if (USE_MPU)
        print_MPU(Roll.ang, Pitch.ang, Yaw.ang, Roll.gyr, Pitch.gyr, Yaw.gyr);
      if (USE_SBUS)
        print_sbus(sbus.des[Ch::ROLL], sbus.des[Ch::PITCH], sbus.des[Ch::THR],
                   sbus.des[Ch::YAW], sbus.des[Ch::Aux1], sbus.des[Ch::Aux2],
                   sbus.des[Ch::Aux3]);
      print_sbus(Main_sbus.des[Ch::ROLL], Main_sbus.des[Ch::PITCH],
                 Main_sbus.des[Ch::THR], Main_sbus.des[Ch::YAW],
                 Main_sbus.des[Ch::Aux1], Main_sbus.des[Ch::Aux2],
                 Main_sbus.des[Ch::Aux3]);

      print_timing(T::Main_dt);
    }
  }
}

// ============================================================
//  センサ・受信機・無線通信の更新
// ============================================================
void updateSensorsAndComms() {
  sbus.update();
  Main_sbus.update();
  switch (Main_sbus.Ch_state(Aux2)) {
    case down:
      Roll.update_value(sbus.des[Ch::ROLL], 0, 0, 0);
      Pitch.update_value(sbus.des[Ch::PITCH], 0, 0, 0);
      Yaw.update_value(sbus.des[Ch::YAW], 0, 0, 0);
      break;
    case cen:
      Roll.update_value(
          constrain(sbus.des[Ch::ROLL] + Main_sbus.des[Ch::ROLL] * 1.3, -1, 1),
          0, 0, 0);
      Pitch.update_value(
          constrain(sbus.des[Ch::PITCH] + Main_sbus.des[Ch::PITCH] * 1.3, -1, 1),
          0, 0, 0);
      Yaw.update_value(
          constrain(sbus.des[Ch::YAW] + Main_sbus.des[Ch::YAW] * 1.3, -1, 1),
          0, 0, 0);
      break;

    case up:
      Roll.update_value(Main_sbus.des[Ch::ROLL], 0, 0, 0);
      Pitch.update_value(Main_sbus.des[Ch::PITCH], 0, 0, 0);
      Yaw.update_value(Main_sbus.des[Ch::YAW], 0, 0, 0);
      break;
    default:
      break;
  }

  switch (Update_SerialCommand()) {
    case 'R':
      reset_all();
      break;
    case 'P':
      handlePIDTuning(Roll, Pitch, Yaw);
      break;
    default:
      break;
  }
}

void reset_all() {
  Config::Timing::resetTiming();
  Roll.pid_reset();
  Pitch.pid_reset();
  Yaw.pid_reset();
  Serial.println("INFO: Remote Reset Complete.");
}

void writeServos() {
  Roll.cmd = Roll.sbus;
  Pitch.cmd = Pitch.sbus;
  Yaw.cmd = Yaw.sbus;
  if (Main_sbus.Ch_state(THR_CUT) == up) {
    Ail1.write(Main_sbus.des[Ch::ROLL]);
    Ail2.write(Main_sbus.des[Ch::ROLL]);
    Ele.write(Main_sbus.des[Ch::PITCH]);
    Rud.write(Main_sbus.des[Ch::YAW]);
  } else {
    Ail1.write(Roll.cmd);
    Ail2.write(Roll.cmd);
    Ele.write(Pitch.cmd);
    Rud.write(Yaw.cmd);
  }
}