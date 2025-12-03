#include "platform.h"

#ifdef ANTC_LOG

#include <string.h>
#include "antc_log_protocol.h"
#include "antc_log_uart.h"
#include "flight/pid.h"
#include "sensors/gyro.h"
#include "common/axis.h"
#include "common/time.h"
#include "flight/imu.h"
#include "config/config.h"
#include "sensors/acceleration.h"

// 控制各数据组是否发送的宏定义（可通过条件编译控制）
#ifndef ANTC_LOG_ENABLE_GROUP1
#define ANTC_LOG_ENABLE_GROUP1 0
#endif

#ifndef ANTC_LOG_ENABLE_GROUP2
#define ANTC_LOG_ENABLE_GROUP2 1
#endif

#ifndef ANTC_LOG_ENABLE_GROUP3
#define ANTC_LOG_ENABLE_GROUP3 1
#endif

#ifndef ANTC_LOG_ENABLE_GROUP4
#define ANTC_LOG_ENABLE_GROUP4 1
#endif

// 各数据组的默认发送频率（微秒）
#ifndef ANTC_LOG_GROUP1_INTERVAL_US
#define ANTC_LOG_GROUP1_INTERVAL_US 20000  // 20ms
#endif

#ifndef ANTC_LOG_GROUP2_INTERVAL_US
#define ANTC_LOG_GROUP2_INTERVAL_US 20000  // 20ms
#endif

#ifndef ANTC_LOG_GROUP3_INTERVAL_US
#define ANTC_LOG_GROUP3_INTERVAL_US 20000  // 20ms
#endif

#ifndef ANTC_LOG_GROUP4_INTERVAL_US
#define ANTC_LOG_GROUP4_INTERVAL_US 20000  // 20ms
#endif

// 发送4个float数据
void antcLogSendUserDatafloat4(uint8_t group, float a, float b, float c, float d) {
    uint8_t _cnt = 0;
    antc_log_atkp_t p;

    p.msgID = ANTC_LOG_UP_USER_DATA1 + group - 1;

    float values[4] = {a, b, c, d};
    for (uint8_t i = 0; i < 4; i++) {
        float temp = values[i];
        p.data[_cnt++] = BYTE3(temp);
        p.data[_cnt++] = BYTE2(temp);
        p.data[_cnt++] = BYTE1(temp);
        p.data[_cnt++] = BYTE0(temp);
    }

    p.dataLen = _cnt;
    antcLogUartSend(&p);
}

// 发送6个float数据
void antcLogSendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f) {
    uint8_t _cnt = 0;
    antc_log_atkp_t p;

    p.msgID = ANTC_LOG_UP_USER_DATA1 + group - 1;

    float values[6] = {a, b, c, d, e, f};
    for (uint8_t i = 0; i < 6; i++) {
        float temp = values[i];
        p.data[_cnt++] = BYTE3(temp);
        p.data[_cnt++] = BYTE2(temp);
        p.data[_cnt++] = BYTE1(temp);
        p.data[_cnt++] = BYTE0(temp);
        // memcpy(&p.data[_cnt], &values[i], sizeof(float));
        // _cnt += sizeof(float);
    }

    p.dataLen = _cnt;
    antcLogUartSend(&p);
}

// 发送9个float数据
void antcLogSendUserDatafloat9(uint8_t group, float a, float b, float c, float d, float e, float f, float g, float h, float i) {
    uint8_t _cnt = 0;
    antc_log_atkp_t p;

    p.msgID = ANTC_LOG_UP_USER_DATA1 + group - 1;

    float values[9] = {a, b, c, d, e, f, g, h, i};
    for (uint8_t j = 0; j < 9; j++) {
        float temp = values[j];
        p.data[_cnt++] = BYTE3(temp);
        p.data[_cnt++] = BYTE2(temp);
        p.data[_cnt++] = BYTE1(temp);
        p.data[_cnt++] = BYTE0(temp);
    }

    p.dataLen = _cnt;
    antcLogUartSend(&p);
}

// 发送Group1数据：内环PID的期望角速度和实际角速度
// roll期望, pitch期望, yaw期望, roll实际, pitch实际, yaw实际
static void antcLogSendGroup1PidRateData(timeUs_t currentTimeUs, timeDelta_t sendInterval) {
#if ANTC_LOG_ENABLE_GROUP1
    static timeUs_t lastSendTime = 0;
    
    if (lastSendTime == 0 || cmpTimeUs(currentTimeUs, lastSendTime) >= sendInterval) {
        lastSendTime = currentTimeUs;
        
        // 获取内环PID的期望角速度和实际角速度
        float rollSetpoint = pidRuntime.previousPidSetpoint[FD_ROLL];
        float pitchSetpoint = pidRuntime.previousPidSetpoint[FD_PITCH];
        float yawSetpoint = pidRuntime.previousPidSetpoint[FD_YAW];
        
        float rollActual = gyro.gyroADCf[FD_ROLL];
        float pitchActual = gyro.gyroADCf[FD_PITCH];
        float yawActual = gyro.gyroADCf[FD_YAW];

        // 发送数据：roll期望, pitch期望, yaw期望, roll实际, pitch实际, yaw实际
        antcLogSendUserDatafloat6(1, rollSetpoint, pitchSetpoint, yawSetpoint, 
                                   rollActual, pitchActual, yawActual);
    }
#else
    UNUSED(currentTimeUs);
    UNUSED(sendInterval);
#endif
}

// 发送Group2数据：姿态估计快照数据（gyro 3个 + acc 3个 + 姿态角度 3个）
// gyroX, gyroY, gyroZ, accX, accY, accZ, rollAngle, pitchAngle, yawAngle
static void antcLogSendGroup2ImuAttitudeSnapshot(timeUs_t currentTimeUs, timeDelta_t sendInterval) {
#if ANTC_LOG_ENABLE_GROUP2 && defined(USE_ACC)
    static timeUs_t lastSendTime = 0;
    
    if (lastSendTime == 0 || cmpTimeUs(currentTimeUs, lastSendTime) >= sendInterval) {
        lastSendTime = currentTimeUs;
        
        // 使用全局快照数据
        antcLogSendUserDatafloat9(2, 
                                  imuAttitudeSnapshot.gyroX,
                                  imuAttitudeSnapshot.gyroY,
                                  imuAttitudeSnapshot.gyroZ,
                                  imuAttitudeSnapshot.accX,
                                  imuAttitudeSnapshot.accY,
                                  imuAttitudeSnapshot.accZ,
                                  imuAttitudeSnapshot.rollAngle,
                                  imuAttitudeSnapshot.pitchAngle,
                                  imuAttitudeSnapshot.yawAngle);
    }
#else
    UNUSED(currentTimeUs);
    UNUSED(sendInterval);
#endif
}

// 发送Group3数据：原始传感器数据快照（对齐前的原始gyro和acc数据）
// gyroRawX, gyroRawY, gyroRawZ, accRawX, accRawY, accRawZ
static void antcLogSendGroup3RawSensorSnapshot(timeUs_t currentTimeUs, timeDelta_t sendInterval) {
#if ANTC_LOG_ENABLE_GROUP3
    static timeUs_t lastSendTime = 0;
    
    if (lastSendTime == 0 || cmpTimeUs(currentTimeUs, lastSendTime) >= sendInterval) {
        lastSendTime = currentTimeUs;
        
        // 使用全局原始传感器快照数据
        antcLogSendUserDatafloat6(3, 
                                  imuRawSensorSnapshot.gyroRawX,
                                  imuRawSensorSnapshot.gyroRawY,
                                  imuRawSensorSnapshot.gyroRawZ,
                                  imuRawSensorSnapshot.accRawX,
                                  imuRawSensorSnapshot.accRawY,
                                  imuRawSensorSnapshot.accRawZ);
    }
#else
    UNUSED(currentTimeUs);
    UNUSED(sendInterval);
#endif
}

// 发送Group4数据：角度环快照数据（pitch轴的4个值）
// pitchAngleTarget, pitchCurrentAngle, pitchErrorAngleGain, pitchAngleFeedforward
static void antcLogSendGroup4AngleLevelSnapshot(timeUs_t currentTimeUs, timeDelta_t sendInterval) {
#if ANTC_LOG_ENABLE_GROUP4
    static timeUs_t lastSendTime = 0;
    
    if (lastSendTime == 0 || cmpTimeUs(currentTimeUs, lastSendTime) >= sendInterval) {
        lastSendTime = currentTimeUs;
        
        // 使用全局角度环快照数据（只发送pitch轴数据）
        antcLogSendUserDatafloat4(4, 
                                  pidAngleLevelSnapshot.pitchAngleTarget,
                                  pidAngleLevelSnapshot.pitchCurrentAngle,
                                  pidAngleLevelSnapshot.pitchErrorAngleGain,
                                  pidAngleLevelSnapshot.pitchAngleFeedforward);
    }
#else
    UNUSED(currentTimeUs);
    UNUSED(sendInterval);
#endif
}

// 任务函数，每20ms调用一次
void antcLogTask(timeUs_t currentTimeUs) {
    // 调用各个数据组的发送函数，每个组可以独立控制频率
    antcLogSendGroup1PidRateData(currentTimeUs, ANTC_LOG_GROUP1_INTERVAL_US);
    antcLogSendGroup2ImuAttitudeSnapshot(currentTimeUs, ANTC_LOG_GROUP2_INTERVAL_US);
    antcLogSendGroup3RawSensorSnapshot(currentTimeUs, ANTC_LOG_GROUP3_INTERVAL_US);
    antcLogSendGroup4AngleLevelSnapshot(currentTimeUs, ANTC_LOG_GROUP4_INTERVAL_US);
}

// 初始化函数
void antcLogInit(void) {
    antcLogUartInit();
}

#endif // ANTC_LOG

