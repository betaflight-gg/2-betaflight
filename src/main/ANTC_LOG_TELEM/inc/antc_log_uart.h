#ifndef __ANTC_LOG_UART_H__
#define __ANTC_LOG_UART_H__

#include "antc_log_protocol.h"

// 串口初始化
void antcLogUartInit(void);

// 发送协议包
void antcLogUartSend(antc_log_atkp_t *p);

#endif /* __ANTC_LOG_UART_H__ */

