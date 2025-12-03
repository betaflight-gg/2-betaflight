#ifndef __ANTC_LOG_PROTOCOL_H__
#define __ANTC_LOG_PROTOCOL_H__

#include <stdint.h>
#include "common/time.h"

// 协议起始字节
#define ANTC_LOG_UP_BYTE1 0xAA
#define ANTC_LOG_UP_BYTE2 0xAA

// 用户数据组ID
#define ANTC_LOG_UP_USER_DATA1 0xF1

// 协议包结构
typedef struct {
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[128];
} antc_log_atkp_t;

// 字节提取宏
#define BYTE0(dwTemp) (*((uint8_t *)(&dwTemp)))
#define BYTE1(dwTemp) (*((uint8_t *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((uint8_t *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((uint8_t *)(&dwTemp) + 3))

// 接口函数声明
void antcLogSendUserDatafloat4(uint8_t group, float a, float b, float c, float d);
void antcLogSendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f);
void antcLogSendUserDatafloat9(uint8_t group, float a, float b, float c, float d, float e, float f, float g, float h, float i);
void antcLogInit(void);
void antcLogTask(timeUs_t currentTimeUs);

#endif /* __ANTC_LOG_PROTOCOL_H__ */

