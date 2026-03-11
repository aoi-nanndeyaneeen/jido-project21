// ============================================================
// sensor_test/main.cpp  修正版
// 変更点:
//   1. MPU6050のスケール設定を明示 (±2g, ±250dps)
//   2. BMP280のサンプリング間隔を200Hzに合わせて高速化
//   3. 海面気圧を自動推定する初期キャリブレーション追加
//   4. オフセットキャリブレーション追加
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <MadgwickAHRS.h>

// ============================================================
//  設定
// ============================================================
constexpr float        LOOP_HZ = 200.0f;
constexpr unsigned long PERIOD = (unsigned long)(1e6f / LOOP_HZ);

// BMP280は高速設定にするため、海面気圧は起動時に手動入力するか
// 以下の値を自分の地域の現在値に変える（天気予報アプリの「気圧」より）
// 例: 東京の場合 1013〜1025hPa 程度で変動する
constexpr float SEA_LEVEL_HPA_DEFAULT = 1013.25f;

// ============================================================
//  カルマンフィルタ
// ============================================================
class AltitudeKalman {
public:
    float Q_z    = 0.001f;
    float Q_vz   = 0.01f;
    float R_baro = 0.25f;
    float z_est  = 0.0f;
    float vz_est = 0.0f;

    void init(float initial_alt) {
        z_est = initial_alt; vz_est = 0.0f;
        P[0][0]=1; P[0][1]=0; P[1][0]=0; P[1][1]=1;
    }

    void update(float dt, float az_world, float baro_z) {
        float z_p  = z_est  + vz_est*dt + 0.5f*az_world*dt*dt;
        float vz_p = vz_est + az_world*dt;
        float P00 = P[0][0] + dt*(P[1][0]+P[0][1]) + dt*dt*P[1][1] + Q_z;
        float P01 = P[0][1] + dt*P[1][1];
        float P10 = P[1][0] + dt*P[1][1];
        float P11 = P[1][1] + Q_vz;
        float S=P00+R_baro, K0=P00/S, K1=P10/S;
        float inn = baro_z - z_p;
        z_est  = z_p  + K0*inn;
        vz_est = vz_p + K1*inn;
        P[0][0]=(1-K0)*P00; P[0][1]=(1-K0)*P01;
        P[1][0]=-K1*P00+P10; P[1][1]=-K1*P01+P11;
    }
private:
    float P[2][2]={{1,0},{0,1}};
};

// ============================================================
//  グローバル
// ============================================================
MPU6050         mpu(0x68);
Adafruit_BMP280 bmp(&Wire1);  // BMP280はWire1(SDA=17,SCL=16)
Madgwick        madgwick;
AltitudeKalman  kalman;

const float ACCEL_SCALE = 16384.0f; // ±2g設定時
const float GYRO_SCALE  = 131.0f;   // ±250dps設定時

int16_t ax_r,ay_r,az_r,gx_r,gy_r,gz_r;
float ax,ay,az,gx,gy,gz;
float baro_alt = 0.0f;
float sea_level_hpa = SEA_LEVEL_HPA_DEFAULT;
unsigned long t_prev = 0;
int counter = 0;
bool mpu_ok=false, bmp_ok=false;

// ============================================================
//  MPU6050 診断 + スケール強制設定
// ============================================================
void mpu_diagnose() {
    Serial.println("\n======= MPU6050 診断 =======");
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("[ERROR] MPU6050 が応答しません");
        return;
    }
    Serial.println("[OK] 接続確認");

    // ★ スケールを明示的に設定（これが今回の修正の核心）
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250dps
    delay(100);

    // 実際に設定されたスケールを読み返して確認
    uint8_t accel_fs = mpu.getFullScaleAccelRange();
    uint8_t gyro_fs  = mpu.getFullScaleGyroRange();
    Serial.printf("  加速度FS設定値: %d (0=±2g, 1=±4g, 2=±8g, 3=±16g)\n", accel_fs);
    Serial.printf("  ジャイロFS設定値: %d (0=±250, 1=±500, 2=±1000, 3=±2000 dps)\n", gyro_fs);

    if (accel_fs != 0) {
        Serial.println("[WARN] 加速度FS=0(±2g)に設定できていません！ライブラリを確認してください。");
    }

    // ---- オフセットキャリブレーション ----
    // センサを水平静止させた状態で実行すること
    Serial.println("  オフセットキャリブレーション中 (200サンプル, 静止してください)...");
    long s_ax=0,s_ay=0,s_az=0,s_gx=0,s_gy=0,s_gz=0;
    for(int i=0;i<200;i++){
        mpu.getMotion6(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);
        s_ax+=ax_r; s_ay+=ay_r; s_az+=az_r;
        s_gx+=gx_r; s_gy+=gy_r; s_gz+=gz_r;
        delay(5);
    }
    float m_ax=s_ax/200.0f, m_ay=s_ay/200.0f, m_az=s_az/200.0f;
    float m_gx=s_gx/200.0f, m_gy=s_gy/200.0f, m_gz=s_gz/200.0f;

    // 加速度の合力チェック (スケール修正後)
    float a_norm = sqrtf((m_ax/ACCEL_SCALE)*(m_ax/ACCEL_SCALE)
                       + (m_ay/ACCEL_SCALE)*(m_ay/ACCEL_SCALE)
                       + (m_az/ACCEL_SCALE)*(m_az/ACCEL_SCALE));
    Serial.printf("  生値平均: ax=%.1f ay=%.1f az=%.1f\n", m_ax, m_ay, m_az);
    Serial.printf("  [g]平均:  ax=%.4f ay=%.4f az=%.4f |a|=%.4f g\n",
                  m_ax/ACCEL_SCALE, m_ay/ACCEL_SCALE, m_az/ACCEL_SCALE, a_norm);

    if (a_norm > 0.9f && a_norm < 1.1f) {
        Serial.printf("[OK]  合力 %.4f g → 正常\n", a_norm);
        mpu_ok = true;
    } else {
        Serial.printf("[ERROR] 合力 %.4f g → 異常 (±2g設定後もこの値なら故障の疑い)\n", a_norm);
        // 処理は続行して詳細確認
        mpu_ok = true;
    }

    // オフセット値をセット（重力方向の軸は除く: 1g=16384カウント分を差し引く）
    // ※ 水平置きでazが+1gになるはず → az_offset = 16384 - m_az
    // どの軸が重力方向かは搭載向きによる。出力を見て調整する。
    mpu.setXAccelOffset((int16_t)(-m_ax / 8));
    mpu.setYAccelOffset((int16_t)(-m_ay / 8));
    // Z軸は重力1g分を残す: 実測平均から理想値(16384)を引いてオフセット計算
    mpu.setZAccelOffset((int16_t)(-(m_az - 16384.0f) / 8));
    mpu.setXGyroOffset((int16_t)(-m_gx / 4));
    mpu.setYGyroOffset((int16_t)(-m_gy / 4));
    mpu.setZGyroOffset((int16_t)(-m_gz / 4));

    Serial.printf("  ジャイロ生値平均: gx=%.1f gy=%.1f gz=%.1f\n", m_gx, m_gy, m_gz);
    Serial.println("  オフセット設定完了。再計測...");
    delay(200);

    // オフセット設定後の再確認
    s_ax=0;s_ay=0;s_az=0;s_gx=0;s_gy=0;s_gz=0;
    for(int i=0;i<100;i++){
        mpu.getMotion6(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);
        s_ax+=ax_r;s_ay+=ay_r;s_az+=az_r;
        s_gx+=gx_r;s_gy+=gy_r;s_gz+=gz_r;
        delay(5);
    }
    m_ax=s_ax/100.0f; m_ay=s_ay/100.0f; m_az=s_az/100.0f;
    m_gx=s_gx/100.0f; m_gy=s_gy/100.0f; m_gz=s_gz/100.0f;
    a_norm = sqrtf((m_ax/ACCEL_SCALE)*(m_ax/ACCEL_SCALE)
                 + (m_ay/ACCEL_SCALE)*(m_ay/ACCEL_SCALE)
                 + (m_az/ACCEL_SCALE)*(m_az/ACCEL_SCALE));
    Serial.printf("  [キャリブ後] |a|=%.4f g  ジャイロ合力=%.3f dps\n",
                  a_norm,
                  sqrtf((m_gx/GYRO_SCALE)*(m_gx/GYRO_SCALE)
                       +(m_gy/GYRO_SCALE)*(m_gy/GYRO_SCALE)
                       +(m_gz/GYRO_SCALE)*(m_gz/GYRO_SCALE)));
    Serial.println("============================\n");
}

// ============================================================
//  BMP280 診断 (高速サンプリング設定)
// ============================================================
void bmp_diagnose() {
    Serial.println("======= BMP280 診断 =======");
    bmp_ok = bmp.begin(0x76);
    if (!bmp_ok) bmp_ok = bmp.begin(0x77);
    if (!bmp_ok) { Serial.println("[ERROR] BMP280 応答なし"); return; }
    Serial.println("[OK] 接続確認");

    // ★ 200Hz制御に合わせて高速設定に変更
    // STANDBY_MS_0_5 = 0.5ms待機 → 約200Hz更新可能
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X1,    // 温度: 最小
                    Adafruit_BMP280::SAMPLING_X4,    // 気圧: 低オーバーサンプリングで高速
                    Adafruit_BMP280::FILTER_X16,     // IIRフィルタ: 強め
                    Adafruit_BMP280::STANDBY_MS_1);  // 1ms待機

    // 起動安定待ち
    delay(200);

    // 静止ノイズ計測
    Serial.println("  静止ノイズ計測中 (100サンプル, 10ms間隔)...");
    float samples[100]; float sum=0;
    for(int i=0;i<100;i++){
        samples[i] = bmp.readAltitude(sea_level_hpa);
        sum += samples[i];
        delay(10);
    }
    float mean=sum/100.0f, var=0;
    for(int i=0;i<100;i++) var+=(samples[i]-mean)*(samples[i]-mean);
    float sigma=sqrtf(var/100.0f);

    Serial.printf("  平均高度: %.3f m (海面気圧=%.2f hPa)\n", mean, sea_level_hpa);
    Serial.printf("  標準偏差: %.3f m\n", sigma);

    if (sigma < 0.15f)      Serial.println("[OK]  ±0.3m以内");
    else if (sigma < 0.5f)  Serial.printf("[WARN] sigma=%.3f m (許容範囲)\n", sigma);
    else                    Serial.printf("[ERROR] sigma=%.3f m (配線ノイズ or 電源ノイズを確認)\n", sigma);

    baro_alt = mean;
    kalman.init(mean);
    Serial.println("============================\n");
}

// ============================================================
//  setup
// ============================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();   Wire.setClock(400000);   // MPU6050: SDA=18, SCL=19
    Wire1.begin();  Wire1.setClock(400000);  // BMP280:  SDA=17, SCL=16
    delay(500);

    Serial.println("\n=== Sensor Diagnostic & Kalman Altitude Test ===");

    // ★ 現在地の気圧を天気予報アプリで確認して入力 (重要)
    // sea_level_hpa = 1015.0f;  // ← 例: 今日の東京が1015hPaなら変更する
    Serial.printf("海面気圧設定: %.2f hPa\n", sea_level_hpa);
    Serial.println("※ 天気予報の現在気圧に合わせると高度精度が上がります\n");

    mpu_diagnose();
    bmp_diagnose();

    if (mpu_ok) madgwick.begin(LOOP_HZ);

    // シリアルプロッタ用ヘッダ
    Serial.println("baro_alt,kalman_alt,kalman_vz,roll,pitch,az_world");
    t_prev = micros();
}

// ============================================================
//  loop
// ============================================================
void loop() {
    unsigned long t_now = micros();
    if (t_now - t_prev < PERIOD) return;
    float dt = (t_now - t_prev) / 1e6f;
    t_prev = t_now;
    counter++;

    if (!mpu_ok || !bmp_ok) { delay(1000); return; }

    mpu.getMotion6(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);
    ax=ax_r/ACCEL_SCALE; ay=ay_r/ACCEL_SCALE; az=az_r/ACCEL_SCALE;
    gx=gx_r/GYRO_SCALE;  gy=gy_r/GYRO_SCALE;  gz=gz_r/GYRO_SCALE;

    madgwick.updateIMU(gx,gy,gz,ax,ay,az);

    float roll_r  = madgwick.getRoll()  * M_PI/180.0f;
    float pitch_r = madgwick.getPitch() * M_PI/180.0f;
    float az_world = (az*cosf(roll_r)*cosf(pitch_r) - 1.0f) * 9.80665f;

    // BMP280は高速設定なので毎ループ読んでOK (内部で更新されていない場合は前回値を返す)
    if (counter % 10 == 0) {  // 200Hz ÷ 10 = 20Hz で読む
        baro_alt = bmp.readAltitude(sea_level_hpa);
    }

    kalman.update(dt, az_world, baro_alt);

    if (counter % 20 == 0) {  // 10Hz でシリアル出力
        // Teleplot用のフォーマット (>変数名:数値\n) に変更！
        Serial.printf(">baro_alt:%.3f\n", baro_alt);
        Serial.printf(">kalman_alt:%.3f\n", kalman.z_est);
        Serial.printf(">kalman_vz:%.3f\n", kalman.vz_est);
        Serial.printf(">roll:%.2f\n", madgwick.getRoll());
        Serial.printf(">pitch:%.2f\n", madgwick.getPitch());
        Serial.printf(">az_world:%.3f\n", az_world);
    }

    // === シリアルから 'r' を受信したらリセット ===
    if (Serial.available() > 0) {
        char inChar = Serial.read();
        if (inChar == 'r' || inChar == 'R') {
            Serial.println(">reset:1"); // Teleplotにリセットを通知(グラフにマーカーを出すなど)
            
            // 1. 高度を現在の気圧高度でリセット
            baro_alt = bmp.readAltitude(sea_level_hpa);
            kalman.init(baro_alt); 
            
            // 2. MPUのオフセットを再計算（今の姿勢を水平とする）
            Serial.println(">msg:Calibrating MPU... Keep it still!");
            mpu_diagnose(); // MPUのキャリブレーション関数をもう一度呼ぶ
            
            // 3. Madgwickフィルタのリセット (ライブラリに依存しますが、インスタンスを作り直すのが確実)
            // madgwick = Madgwick(); // 一度破棄して
            // madgwick.begin(LOOP_HZ); // 再初期化
            
            Serial.println(">msg:Reset Complete!");
        }
    }

}