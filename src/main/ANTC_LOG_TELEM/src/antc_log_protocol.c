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

// 任务函数，每20ms调用一次
void antcLogTask(timeUs_t currentTimeUs) {
    static timeUs_t lastSendTime = 0;
    const timeDelta_t sendInterval = 1000; // 20ms = 20000us

    if (lastSendTime == 0 || cmpTimeUs(currentTimeUs, lastSendTime) >= sendInterval) {
        lastSendTime = currentTimeUs;

        // 获取内环PID的期望角速度和实际角速度
        // 期望角速度：pidRuntime.previousPidSetpoint[axis]
        // 实际角速度：gyro.gyroADCf[axis]
        
        float rollSetpoint = pidRuntime.previousPidSetpoint[FD_ROLL];
        float pitchSetpoint = pidRuntime.previousPidSetpoint[FD_PITCH];
        float yawSetpoint = pidRuntime.previousPidSetpoint[FD_YAW];
        
        float rollActual = gyro.gyroADCf[FD_ROLL];
        float pitchActual = gyro.gyroADCf[FD_PITCH];
        float yawActual = gyro.gyroADCf[FD_YAW];

        // 发送数据：roll期望, pitch期望, yaw期望, roll实际, pitch实际, yaw实际
        antcLogSendUserDatafloat6(1, rollSetpoint, pitchSetpoint, yawSetpoint, 
                                   rollActual, pitchActual, yawActual);

#ifdef USE_ACC
        // 获取角度环PID参数
        // Roll角度环参数
        float rollAngleTarget = pidRuntime.angleTarget[AI_ROLL];  // 角度目标（度）
        float rollCurrentAngle = attitude.values.roll / 10.0f;     // 当前角度（度，从decidegrees转换）
        float rollErrorAngle = rollAngleTarget - rollCurrentAngle; // 角度误差（度）
        
        // Pitch角度环参数
        float pitchAngleTarget = pidRuntime.angleTarget[AI_PITCH]; // 角度目标（度）
        float pitchCurrentAngle = attitude.values.pitch / 10.0f;   // 当前角度（度，从decidegrees转换）
        float pitchErrorAngle = pitchAngleTarget - pitchCurrentAngle; // 角度误差（度）
        
        // 角度环PID参数
        float angleGain = pidRuntime.angleGain;                    // 角度环增益
        float angleLimit = (float)currentPidProfile->angle_limit; // 角度限制（度）
        float angleFeedforwardGain = pidRuntime.angleFeedforwardGain; // 角度前馈增益
        float angleEarthRef = pidRuntime.angleEarthRef;            // 地球参考增益
        
        // 发送角度环PID参数：roll目标, roll当前, roll误差, pitch目标, pitch当前, pitch误差
        antcLogSendUserDatafloat6(2, rollAngleTarget, rollCurrentAngle, rollErrorAngle,
                                   pitchAngleTarget, pitchCurrentAngle, pitchErrorAngle);
        
        // 发送角度环PID配置参数：角度增益, 角度限制, 前馈增益, 地球参考增益, roll角度模式, pitch角度模式
        float rollInAngleMode = pidRuntime.axisInAngleMode[FD_ROLL] ? 1.0f : 0.0f;
        float pitchInAngleMode = pidRuntime.axisInAngleMode[FD_PITCH] ? 1.0f : 0.0f;
        antcLogSendUserDatafloat6(3, angleGain, angleLimit, angleFeedforwardGain,
                                   angleEarthRef, rollInAngleMode, pitchInAngleMode);
#endif // USE_ACC
    }
}

// 初始化函数
void antcLogInit(void) {
    antcLogUartInit();
}

#endif // ANTC_LOG

