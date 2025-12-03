/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "common/vector.h"

#include "pg/pg.h"

// Exported symbols
extern bool canUseGPSHeading;

typedef union {
    float v[4];
    struct {
        float w, x, y, z;
    };
} quaternion_t;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree  eg attitude.values.yaw 180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;
#define EULER_INITIALIZE  { { 0, 0, 0 } }

extern attitudeEulerAngles_t attitude;
extern matrix33_t rMat;
extern quaternion_t imuAttitudeQuaternion; //attitude quaternion to use in blackbox

#ifdef ANTC_LOG
// 姿态估计快照数据结构：用于记录送入imuMahonyAHRSupdate的gyro和acc数据，以及姿态估计后的角度
typedef struct {
    float gyroX;      // 陀螺仪X轴（度/秒）
    float gyroY;      // 陀螺仪Y轴（度/秒）
    float gyroZ;      // 陀螺仪Z轴（度/秒）
    float accX;       // 加速度计X轴
    float accY;       // 加速度计Y轴
    float accZ;       // 加速度计Z轴
    float rollAngle;  // 姿态估计后的roll角度（度）
    float pitchAngle; // 姿态估计后的pitch角度（度）
    float yawAngle;   // 姿态估计后的yaw角度（度）
} imuAttitudeSnapshot_t;

extern imuAttitudeSnapshot_t imuAttitudeSnapshot;

// 原始传感器数据快照：用于记录对齐前的原始gyro和acc数据
typedef struct {
    float gyroRawX;   // 陀螺仪X轴原始ADC值（对齐前）
    float gyroRawY;   // 陀螺仪Y轴原始ADC值（对齐前）
    float gyroRawZ;   // 陀螺仪Z轴原始ADC值（对齐前）
    float accRawX;    // 加速度计X轴原始ADC值（对齐前）
    float accRawY;    // 加速度计Y轴原始ADC值（对齐前）
    float accRawZ;    // 加速度计Z轴原始ADC值（对齐前）
} imuRawSensorSnapshot_t;

extern imuRawSensorSnapshot_t imuRawSensorSnapshot;

// 角度环快照数据结构：用于记录pidLevel中角度环计算的数据（只记录pitch轴）
typedef struct {
    float pitchAngleTarget;           // pitch轴目标角度（度）
    float pitchCurrentAngle;           // pitch轴当前角度（度）
    float pitchErrorAngleGain;         // pitch轴 errorAngle * angleGain
    float pitchAngleFeedforward;       // pitch轴前馈值
} pidAngleLevelSnapshot_t;

extern pidAngleLevelSnapshot_t pidAngleLevelSnapshot;
#endif // ANTC_LOG

typedef struct imuConfig_s {
    uint16_t imu_dcm_kp;          // DCM filter proportional gain ( x 10000)
    uint16_t imu_dcm_ki;          // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t imu_process_denom;
    int16_t mag_declination;      // Magnetic declination in degrees * 10
} imuConfig_t;

PG_DECLARE(imuConfig_t, imuConfig);

typedef struct imuRuntimeConfig_s {
    float imuDcmKi;
    float imuDcmKp;
} imuRuntimeConfig_t;

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value);

float getSinPitchAngle(void);
float getCosTiltAngle(void);
void getQuaternion(quaternion_t * q);
void imuUpdateAttitude(timeUs_t currentTimeUs);

void imuInit(void);

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw);  // in deg
void imuSetAttitudeQuat(float w, float x, float y, float z);
#if defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt);
#endif
#endif

bool imuQuaternionHeadfreeOffsetSet(void);
void imuQuaternionHeadfreeTransformVectorEarthToBody(vector3_t *v);
bool isUpright(void);
