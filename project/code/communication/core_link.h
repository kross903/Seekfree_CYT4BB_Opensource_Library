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
}core_link_protocol_enum;

typedef struct
{
    uint8 command;
    uint8 item;
    float value;
}core_link_param_packet_struct;

void core_link_init             (void);                                         // Init UART4, FIFO, and parser
void core_link_task             (void);                                         // Run receive, unpack, and dispatch in main loop
void core_link_uart4_rx_handler (void);                                         // Call from uart4_isr to push rx bytes into FIFO

void core_link_send_frame       (uint8 protocol_id, const uint8 *payload, uint8 length);   // Send generic frame
void core_link_send_text        (const char *text);                             // Send text payload frame
void core_link_send_param_ascii (uint8 command, uint8 item, float value);       // Send ASCII param payload like "R1=1.234"

void core_link_on_frame_received (uint8 protocol_id, const uint8 *payload, uint8 length);       // Optional generic callback for new protocol ids
void core_link_on_text_received  (const uint8 *text, uint8 length);                              // Optional text callback
void core_link_on_param_received (const core_link_param_packet_struct *param);                   // Optional param callback

#endif
