// ============================================================
// src/main_test.cpp  IMU 6軸確認版
//
// 【Teleplotでリアルタイム表示】
//   VSCode拡張 "Teleplot" をインストールして使用
//   表示される変数: ax, ay, az, gx, gy, gz, a_norm
//
// 【シリアルコマンド】
//   R : オフセットキャリブレーション再実行
//   P : スナップショット + レジスタ値表示
//   S : スケール設定の読み返し確認
//
// 【軸確認手順】(Pキーを押しながら各姿勢を確認)
//   水平置き (チップ面上向き) : az≈+1g, ax≈0, ay≈0
//   手前に90°傾ける           : ax≈+1g, az≈0
//   右に90°傾ける             : ay≈-1g, az≈0
//   裏返し                    : az≈-1g, ax≈0, ay≈0
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// ============================================================
//  設定
// ============================================================
constexpr float        LOOP_HZ = 200.0f;
constexpr unsigned long PERIOD = (unsigned long)(1e6f / LOOP_HZ);
constexpr uint8_t      MPU_ADDR = 0x68;

// ±2g設定時のスケール。後述のレジスタ確認で実際の設定値を見てから変える
// FS=0(±2g)  → 16384
// FS=1(±4g)  → 8192
// FS=2(±8g)  → 4096
// FS=3(±16g) → 2048
float ACCEL_SCALE = 16384.0f;
float GYRO_SCALE  = 131.0f;   // ±250dps固定

// ============================================================
//  グローバル
// ============================================================
MPU6050 mpu(MPU_ADDR);
int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
float   ax, ay, az, gx, gy, gz;
unsigned long t_prev = 0;
int counter = 0;
bool mpu_ok = false;

// ============================================================
//  レジスタ直読み (ライブラリを介さず確認)
// ============================================================
uint8_t read_register(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    return Wire.read();
}

void write_register(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

// ============================================================
//  スケール確認と強制設定
// ============================================================
void check_and_set_scale() {
    // レジスタ直書き (ライブラリが効かない場合の保険)
    // 0x1C ACCEL_CONFIG: bits[4:3] = 00 → ±2g
    // 0x1B GYRO_CONFIG:  bits[4:3] = 00 → ±250dps
    write_register(0x1C, 0x00);
    write_register(0x1B, 0x00);
    delay(10);

    uint8_t accel_cfg = read_register(0x1C);
    uint8_t gyro_cfg  = read_register(0x1B);

    // bits[4:3]を取り出してFSレンジを判定
    uint8_t afs = (accel_cfg >> 3) & 0x03;
    uint8_t gfs = (gyro_cfg  >> 3) & 0x03;

    // 実際のFSレンジに合わせてスケールを設定
    const float accel_scales[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    const float gyro_scales[]  = {131.0f,   65.5f,   32.8f,   16.4f};
    ACCEL_SCALE = accel_scales[afs];
    GYRO_SCALE  = gyro_scales[gfs];

    Serial.println("INFO: === スケール確認 ===");
    Serial.printf("INFO: ACCEL_CONFIG=0x%02X → FS=%d → ±%dg → スケール=%.0f\n",
                  accel_cfg, afs, (2 << afs), ACCEL_SCALE);
    Serial.printf("INFO: GYRO_CONFIG =0x%02X → FS=%d → スケール=%.1f\n",
                  gyro_cfg, gfs, GYRO_SCALE);

    if (afs == 0)
        Serial.println("INFO: [OK] 加速度 ±2g 設定確認");
    else
        Serial.printf("INFO: [WARN] 加速度が±%dgになっています。スケールを自動修正しました。\n", 2<<afs);
}

// ============================================================
//  オフセットキャリブレーション
// ============================================================
void calibrate() {
    Serial.println("INFO: キャリブレーション開始 (静止 300サンプル)...");
    long s_ax=0,s_ay=0,s_az=0,s_gx=0,s_gy=0,s_gz=0;
    for(int i=0;i<300;i++){
        mpu.getMotion6(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);
        s_ax+=ax_r; s_ay+=ay_r; s_az+=az_r;
        s_gx+=gx_r; s_gy+=gy_r; s_gz+=gz_r;
        delay(5);
    }
    float m_ax=s_ax/300.0f, m_ay=s_ay/300.0f, m_az=s_az/300.0f;
    float m_gx=s_gx/300.0f, m_gy=s_gy/300.0f, m_gz=s_gz/300.0f;

    // キャリブ前の合力
    float norm_before = sqrtf((m_ax/ACCEL_SCALE)*(m_ax/ACCEL_SCALE)
                             +(m_ay/ACCEL_SCALE)*(m_ay/ACCEL_SCALE)
                             +(m_az/ACCEL_SCALE)*(m_az/ACCEL_SCALE));
    Serial.printf("INFO: キャリブ前  ax=%.4f ay=%.4f az=%.4f |a|=%.4f g\n",
                  m_ax/ACCEL_SCALE, m_ay/ACCEL_SCALE, m_az/ACCEL_SCALE, norm_before);
    Serial.printf("INFO: 生値平均    ax=%.0f ay=%.0f az=%.0f\n", m_ax, m_ay, m_az);

    // az生値が16384に近い → センサは水平
    // ax生値が16384に近い → センサは手前90°傾き
    float ideal_az = ACCEL_SCALE; // 水平置きなら重力はzに乗る想定
    Serial.printf("INFO: az生値=%.0f (水平なら±%.0f に近いはず)\n", m_az, ideal_az);

    if (fabsf(m_az) < fabsf(m_ax) && fabsf(m_az) < fabsf(m_ay)) {
        Serial.println("INFO: [WARN] az が最小 → センサが横倒しの可能性。水平に置いてRで再実行。");
    }

    // オフセット設定 (重力が乗っている軸は理想値との差をオフセット)
    // 水平置き前提: az → 理想+16384、ax,ay → 理想0
    mpu.setXAccelOffset((int16_t)(-m_ax / 8));
    mpu.setYAccelOffset((int16_t)(-m_ay / 8));
    mpu.setZAccelOffset((int16_t)(-(m_az - ACCEL_SCALE) / 8));
    mpu.setXGyroOffset((int16_t)(-m_gx / 4));
    mpu.setYGyroOffset((int16_t)(-m_gy / 4));
    mpu.setZGyroOffset((int16_t)(-m_gz / 4));
    delay(200);

    // キャリブ後確認
    s_ax=0;s_ay=0;s_az=0;s_gx=0;s_gy=0;s_gz=0;
    for(int i=0;i<100;i++){
        mpu.getMotion6(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);
        s_ax+=ax_r;s_ay+=ay_r;s_az+=az_r;
        s_gx+=gx_r;s_gy+=gy_r;s_gz+=gz_r;
        delay(5);
    }
    m_ax=s_ax/100.0f; m_ay=s_ay/100.0f; m_az=s_az/100.0f;
    m_gx=s_gx/100.0f; m_gy=s_gy/100.0f; m_gz=s_gz/100.0f;
    float norm_after = sqrtf((m_ax/ACCEL_SCALE)*(m_ax/ACCEL_SCALE)
                            +(m_ay/ACCEL_SCALE)*(m_ay/ACCEL_SCALE)
                            +(m_az/ACCEL_SCALE)*(m_az/ACCEL_SCALE));
    Serial.printf("INFO: キャリブ後  ax=%.4f ay=%.4f az=%.4f |a|=%.4f g\n",
                  m_ax/ACCEL_SCALE, m_ay/ACCEL_SCALE, m_az/ACCEL_SCALE, norm_after);
    float gyro_norm = sqrtf((m_gx/GYRO_SCALE)*(m_gx/GYRO_SCALE)
                           +(m_gy/GYRO_SCALE)*(m_gy/GYRO_SCALE)
                           +(m_gz/GYRO_SCALE)*(m_gz/GYRO_SCALE));
    Serial.printf("INFO: ジャイロ    gx=%.4f gy=%.4f gz=%.4f |g|=%.4f dps\n",
                  m_gx/GYRO_SCALE, m_gy/GYRO_SCALE, m_gz/GYRO_SCALE, gyro_norm);

    // 合力判定
    if (norm_after > 0.97f && norm_after < 1.03f)
        Serial.println("INFO: [OK] 合力≒1g → センサ正常");
    else if (norm_before > 1.8f && norm_after > 1.8f)
        Serial.println("INFO: [ERROR] 合力≈2g → スケール設定を確認 (Sキー)");
    else if (norm_after < 0.5f)
        Serial.println("INFO: [ERROR] 合力<0.5g → センサ故障の可能性大");
    else
        Serial.printf("INFO: [WARN] 合力=%.4g → 許容範囲外。センサの傾きかオフセット残差。\n", norm_after);

    Serial.println("INFO: キャリブレーション完了");
}

// ============================================================
//  スナップショット表示
// ============================================================
void printSnapshot() {
    mpu.getMotion6(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);
    float _ax=ax_r/ACCEL_SCALE, _ay=ay_r/ACCEL_SCALE, _az=az_r/ACCEL_SCALE;
    float _gx=gx_r/GYRO_SCALE,  _gy=gy_r/GYRO_SCALE,  _gz=gz_r/GYRO_SCALE;
    float norm = sqrtf(_ax*_ax+_ay*_ay+_az*_az);
    Serial.println("INFO: ===== スナップショット =====");
    Serial.printf ("INFO: 加速度[g] ax=%+.4f  ay=%+.4f  az=%+.4f  |a|=%.4f\n",_ax,_ay,_az,norm);
    Serial.printf ("INFO: ジャイロ  gx=%+.3f  gy=%+.3f  gz=%+.3f  dps\n",_gx,_gy,_gz);
    Serial.println("INFO: ---- 判定 ----");
    Serial.printf ("INFO: |a|=%.4f → %s\n", norm,
                   norm>0.97f&&norm<1.03f ? "正常 (≒1g)" :
                   norm>1.8f             ? "スケール誤り疑い (≒2g)" :
                   norm<0.5f             ? "故障疑い (<0.5g)" : "要確認");
    Serial.println("INFO: ---- 軸確認ガイド ----");
    Serial.println("INFO: 水平置き  → az≈+1.0g, ax≈0, ay≈0");
    Serial.println("INFO: 手前90°  → ax≈+1.0g, az≈0");
    Serial.println("INFO: 右90°    → ay≈-1.0g, az≈0");
    Serial.println("INFO: 裏返し   → az≈-1.0g, ax≈0, ay≈0");
    Serial.println("INFO: ==========================");
}

// ============================================================
//  setup
// ============================================================
void setup() {
    Serial.begin(115200);
    while(!Serial && millis()<3000){}
    Wire.begin(); Wire.setClock(400000);
    delay(300);

    Serial.println("INFO: === IMU 6軸確認モード ===");
    Serial.println("INFO: コマンド: R=キャリブ  P=スナップショット  S=スケール確認");

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("INFO: [ERROR] MPU6050 応答なし。配線を確認してください。");
        while(1){}
    }
    Serial.println("INFO: MPU6050 接続OK");

    check_and_set_scale();
    calibrate();

    // Teleplot用ヘッダ出力
    Serial.println("INFO: Teleplot表示開始 (ax ay az gx gy gz a_norm)");
    t_prev = micros();
    mpu_ok = true;
}

// ============================================================
//  loop
// ============================================================
void loop() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        if (c=='R') { check_and_set_scale(); calibrate(); }
        if (c=='P') printSnapshot();
        if (c=='S') check_and_set_scale();
    }

    unsigned long t_now = micros();
    if (t_now - t_prev < PERIOD) return;
    t_prev = t_now;
    counter++;

    if (!mpu_ok) return;

    mpu.getMotion6(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);
    ax=ax_r/ACCEL_SCALE; ay=ay_r/ACCEL_SCALE; az=az_r/ACCEL_SCALE;
    gx=gx_r/GYRO_SCALE;  gy=gy_r/GYRO_SCALE;  gz=gz_r/GYRO_SCALE;

    // Teleplot形式で10Hz出力
    if (counter % 20 == 0) {
        float a_norm = sqrtf(ax*ax+ay*ay+az*az);
        Serial.printf(">ax:%.4f\n", ax);
        Serial.printf(">ay:%.4f\n", ay);
        Serial.printf(">az:%.4f\n", az);
        Serial.printf(">gx:%.3f\n", gx);
        Serial.printf(">gy:%.3f\n", gy);
        Serial.printf(">gz:%.3f\n", gz);
        // a_normが常に1.0gなら正常、2.0gならスケール誤り
        Serial.printf(">a_norm:%.4f\n", a_norm);
    }
}
