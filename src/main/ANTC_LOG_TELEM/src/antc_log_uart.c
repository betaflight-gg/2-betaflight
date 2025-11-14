#include "platform.h"

#ifdef ANTC_LOG

#include <string.h>
#include "antc_log_uart.h"
#include "antc_log_protocol.h"
#include "io/serial.h"
#include "drivers/serial.h"

#define ANTC_LOG_MAX_DATA_SIZE 128
#define ANTC_LOG_PROTOCOL_HEAD_SIZE 6
#define ANTC_LOG_BUF_SIZE (ANTC_LOG_MAX_DATA_SIZE + ANTC_LOG_PROTOCOL_HEAD_SIZE)

static serialPort_t *antcLogSerialPort = NULL;

// 串口初始化
void antcLogUartInit(void) {
    // 打开串口1，波特率500000
    // 直接使用SERIAL_PORT_USART1
    antcLogSerialPort = openSerialPort(
        SERIAL_PORT_USART1,
        FUNCTION_NONE,
        NULL,
        NULL,
        baudRates[BAUD_500000],
        MODE_TX,
        SERIAL_NOT_INVERTED);
}

// 发送协议包
void antcLogUartSend(antc_log_atkp_t *p) {
    if (antcLogSerialPort == NULL || p == NULL) {
        return;
    }

    if (p->dataLen > ANTC_LOG_MAX_DATA_SIZE) {
        return;
    }

    uint8_t sendBuffer[ANTC_LOG_BUF_SIZE];
    uint8_t cksum = 0;
    int dataSize;

    sendBuffer[0] = ANTC_LOG_UP_BYTE1;
    sendBuffer[1] = ANTC_LOG_UP_BYTE2;
    sendBuffer[2] = p->msgID;
    sendBuffer[3] = p->dataLen;

    memcpy(&sendBuffer[4], p->data, p->dataLen);
    dataSize = p->dataLen + 4;

    // 计算校验和
    for (int i = 0; i < dataSize; i++) {
        cksum += sendBuffer[i];
    }
    sendBuffer[dataSize] = cksum;
    dataSize++;

    // 发送数据
    serialWriteBuf(antcLogSerialPort, sendBuffer, dataSize);
}

#endif // ANTC_LOG

