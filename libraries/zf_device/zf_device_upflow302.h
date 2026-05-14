#ifndef _ZF_DEVICE_UPFLOW302_H_
#define _ZF_DEVICE_UPFLOW302_H_

#include "zf_common_headfile.h"

#define UP_FLOW_302_UART_INDEX        (UART_2)
#define UP_FLOW_302_BAUDRATE          (19200)
#define UP_FLOW_302_UART_TX_PIN       (UART2_TX_P10_1)
#define UP_FLOW_302_UART_RX_PIN       (UART2_RX_P10_0)

#define UP_FLOW_302_DATA_LEN          (14U)
#define UP_FLOW_302_FRAME_START       (0xFEU)
#define UP_FLOW_302_DEVICE_ID         (0x0AU)
#define UP_FLOW_302_TIMEOUT_COUNT     (0x00FFU)

typedef struct
{
    uint8 head;
    uint8 device_id;
    int16 upflow302_x;
    int16 upflow302_y;
    int16 upflow302_us;
    int16 upflow302_us_a;
    uint8 upflow302_valid;
    uint8 upflow302_version;
    uint8 sum_check;
    uint8 sum_end;
} upflow302_receive_struct;

extern upflow302_receive_struct upflow302_receive;
extern uint8 upflow302_receiver_data[UP_FLOW_302_DATA_LEN];
extern uint8 upflow302_finsh_flag;
extern uint8 upflow302_state_flag;
extern uint16 upflow302_response_time;
extern int16 upflow302_x;
extern int16 upflow302_y;

void upflow302_receive_callback(void);
void upflow302_receive_init(void);

#endif