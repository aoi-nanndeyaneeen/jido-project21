//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void reset() { q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f; anglesComputed = 0; }
    // 加速度補正の強さ [rad/s 相当の定速スルー]。詳細は flight_controller/include/sensor/IMU.h。
    void  setBeta(float b) { beta = b; }
    float getBeta() const  { return beta; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    // 固定周期版 (begin() の sampleFrequency で積分)。周期が保証できるときだけ使う。
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        updateIMU(gx, gy, gz, ax, ay, az, invSampleFreq);
    }
    // 実測 dt 版 (2026-09-18 追加)。ループ周期が揺れる板 (RP2040 で 1.3〜1.7ms) では
    // こちらを使う。固定周期版のまま実測 1.5ms で回すと角度が 1/1.5 に縮んで出る。
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt_s);
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
};
#endif

