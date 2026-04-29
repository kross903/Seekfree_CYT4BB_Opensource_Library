#ifndef _core_link_h_
#define _core_link_h_

#include "zf_common_headfile.h"

#define CORE_LINK_UART_INDEX            (UART_4)
#define CORE_LINK_UART_BAUDRATE         (115200)
#define CORE_LINK_UART_TX_PIN           (UART4_TX_P14_1)
#define CORE_LINK_UART_RX_PIN           (UART4_RX_P14_0)

#define CORE_LINK_RX_FIFO_SIZE          (256)
#define CORE_LINK_PAYLOAD_MAX_LEN       (64)
#define CORE_LINK_PARAM_DECIMAL_NUM     (3)

#define CORE_LINK_SOF1                  (0xA5)
#define CORE_LINK_SOF2                  (0x5A)
extern volatile uint8 Car_Copy_Screen_Flag;//20ms

typedef enum
{
    CORE_LINK_PROTOCOL_TEXT        = 0x01,                                      // Text payload, such as "HELLO"
    CORE_LINK_PROTOCOL_PARAM_ASCII = 0x02,                                      // Param payload, such as "R1=1.234"
    CORE_LINK_PROTOCOL_DRONE_STATUS = 0x10,                                     // Fixed-length drone status summary
}core_link_protocol_enum;

typedef struct
{
    uint8 command;
    uint8 item;
    float value;
}core_link_param_packet_struct;

typedef struct
{
    uint8 armed;
    uint8 control_mode;
    uint8 low_voltage;
    uint8 beacon_count;
    uint16 voltage_mv;
    int16 target_throttle;
    int16 base_throttle;
    int16 roll_ddeg;
    int16 pitch_ddeg;
    int16 yaw_ddeg;
    int16 tof_height_dcm;
    int16 fused_height_dcm;
} core_link_drone_status_packet_struct;

void core_link_init             (void);                                         // Init UART4, FIFO, and parser
void core_link_task             (void);                                         // Run receive, unpack, and dispatch in main loop
void core_link_uart4_rx_handler (void);                                         // Call from uart4_isr to push rx bytes into FIFO

void core_link_send_frame       (uint8 protocol_id, const uint8 *payload, uint8 length);   // Send generic frame
void core_link_send_text        (const char *text);                             // Send text payload frame
void core_link_send_param_ascii (uint8 command, uint8 item, float value);       // Send ASCII param payload like "R1=1.234"
void core_link_send_drone_status(const core_link_drone_status_packet_struct *status);

/* 指定 UART 的版本：帧格式与上面完全一致，仅改变物理通道。
 * 用于把同一份协议同时下发给非 UART4 的第二块主控板。 */
void core_link_send_frame_via       (uart_index_enum uart_n, uint8 protocol_id, const uint8 *payload, uint8 length);
void core_link_send_param_ascii_via (uart_index_enum uart_n, uint8 command, uint8 item, float value);

void core_link_on_frame_received (uint8 protocol_id, const uint8 *payload, uint8 length);       // Optional generic callback for new protocol ids
void core_link_on_text_received  (const uint8 *text, uint8 length);                              // Optional text callback
void core_link_on_param_received (const core_link_param_packet_struct *param);                   // Optional param callback

#endif
