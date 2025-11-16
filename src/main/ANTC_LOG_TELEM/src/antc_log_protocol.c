#include "platform.h"

#ifdef ANTC_LOG

#include <string.h>
#include "antc_log_protocol.h"
#include "antc_log_uart.h"
#include "flight/pid.h"
#include "sensors/gyro.h"
#include "common/axis.h"
#include "common/time.h"

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
    }
}

// 初始化函数
void antcLogInit(void) {
    antcLogUartInit();
}

#endif // ANTC_LOG

