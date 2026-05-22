#ifndef CODE_VOFA_H_
#define CODE_VOFA_H_

#include "zf_common_headfile.h"

/*
 * VOFA 无线调参模块头文件。
 * 这里只暴露模块对外可调用的入口，接收缓冲和解析状态仍然保留在 vofa.c 内部。
 */

// ==================== UART 硬件配置 ====================
#define UART_INDEX              (UART_1)
#define UART_BAUDRATE           (115200)
#define UART_TX_PIN             (UART1_TX_P04_1)
#define UART_RX_PIN             (UART1_RX_P04_0)

// ==================== 模块对外接口 ====================
void Init_uart(void);
void Send_test(void);
void uart_rx_interrupt_handler(void);
void USART_PID_Adjust(void);
void send_date_back(void);
void Vofa_ClearRxState(void);

#endif
