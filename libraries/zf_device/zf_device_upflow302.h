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
#define UP_FLOW_302_VALID_VALUE       (245U)
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

typedef struct
{
    int16 flow_x;
    int16 flow_y;
    int16 frame_dt_us;
    uint32 frame_seq;
    uint32 frame_start_cycle;
    uint32 gyro_window_cycles;
    float gyro_x_delta_rad;
    float gyro_y_delta_rad;
    float gyro_z_delta_rad;
    float yaw_mid_deg;
    uint8 gyro_window_valid;
    uint8 yaw_window_valid;
} upflow302_frame_t;

typedef uint8 (*upflow302_gyro_snapshot_callback_t)(uint32 target_cycle,
                                                     float *total_x_rad,
                                                     float *total_y_rad,
                                                     float *total_z_rad,
                                                     float *yaw_deg);

extern upflow302_receive_struct upflow302_receive;
extern uint8 upflow302_receiver_data[UP_FLOW_302_DATA_LEN];
extern uint8 upflow302_finsh_flag;
extern uint8 upflow302_state_flag;
extern uint16 upflow302_response_time;
extern int16 upflow302_x;
extern int16 upflow302_y;
extern uint32 upflow302_frame_seq;
extern uint32 upflow302_rx_cycle;
extern int16 upflow302_frame_dt_us;

void upflow302_receive_callback(void);
void upflow302_receive_init(void);
void upflow302_set_gyro_snapshot_callback(upflow302_gyro_snapshot_callback_t callback);
uint8 upflow302_pop_frame(upflow302_frame_t *frame);
uint8 upflow302_pop_latest_frame(upflow302_frame_t *frame);
uint16 upflow302_get_dropped_frame_count(void);

#endif
