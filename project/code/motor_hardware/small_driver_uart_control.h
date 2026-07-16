#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_6        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART6_TX_P03_1)

#define SMALL_DRIVER_TX                         (UART6_RX_P03_0)

#define SMALL_DRIVER_FRAME_LEN                  11U
#define SMALL_DRIVER_SPEED_FUNCTION             0x02U
#define SMALL_DRIVER_ESC_STATUS_FUNCTION        0x07U
#define SMALL_DRIVER_ESC_STATUS_VERSION         0x01U
#define SMALL_DRIVER_ESC_STATUS_STALE_MS        500U

typedef enum
{
    SMALL_DRIVER_ESC_FAULT_NONE = 0,
    SMALL_DRIVER_ESC_FAULT_VOLTAGE = 1,
    SMALL_DRIVER_ESC_FAULT_LOCKED = 2,
    SMALL_DRIVER_ESC_FAULT_DISTURB = 3,
    SMALL_DRIVER_ESC_FAULT_START_FAIL = 4,
    SMALL_DRIVER_ESC_FAULT_TEMPERATURE_LOW = 5,
    SMALL_DRIVER_ESC_FAULT_TEMPERATURE_HIGH = 6,
    SMALL_DRIVER_ESC_FAULT_OVER_CURRENT = 7,
    SMALL_DRIVER_ESC_FAULT_LOW_SPEED_RESTART = 8
} SmallDriverEscFaultCode;

typedef struct
{
    uint8 sequence;
    uint8 fault_code;
    uint8 motor_mask;
} SmallDriverEscFaultEvent;

typedef struct
{
    uint32 status_frame_count;
    uint32 speed_frame_count;
    uint32 checksum_error_count;
    uint32 unknown_frame_count;
    uint32 fault_event_count;
    uint16 age_ms;
    uint8 valid;
    uint8 stale;
    uint8 protocol_version;
    uint8 event_sequence;
    uint8 last_fault_code;
    uint8 last_fault_motor_mask;
    uint8 last_event_sequence;
    uint8 last_event_code;
    uint8 last_event_motor_mask;
    uint8 active_fault[4];
} SmallDriverEscStatus;

typedef struct
{
    uint8 send_data_buffer[11];                 // 发送缓冲数组

    uint8 receive_data_buffer[11];              // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_speed_data[4];                // 接收到的电机速度数据

}small_device_value_struct;

typedef struct
{
    uint32 queued_frame_count;
    uint32 sent_frame_count;
    uint32 overwritten_frame_count;
    uint32 zero_frame_count;
    uint32 fifo_error_count;
    uint16 pending_age_us;
    uint16 max_pending_age_us;
    uint8 pending;
} SmallDriverUartDiag;

extern small_device_value_struct motor_value;



void uart_control_callback(void);                                   // 无刷驱动 串口接收回调函数

void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4);      // 无刷驱动 设置电机占空比
void small_driver_service_tx(uint16 elapsed_us);
void small_driver_force_zero(void);
void small_driver_get_diag(SmallDriverUartDiag *diag);
void small_driver_get_esc_status(SmallDriverEscStatus *status);
uint8 small_driver_take_fault_event(SmallDriverEscFaultEvent *event);
uint8 small_driver_esc_has_active_fault(void);
void small_driver_tick_5ms(void);

void small_driver_get_speed(void);                                  // 无刷驱动 获取速度信息

void small_driver_uart_init(void);                                  // 无刷驱动 串口通讯初始化


#endif
