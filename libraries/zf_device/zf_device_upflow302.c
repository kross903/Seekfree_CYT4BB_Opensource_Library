#include "zf_device_upflow302.h"

#define UPFLOW302_FRAME_QUEUE_LEN    (4U)

uint8 upflow302_receiver_data[UP_FLOW_302_DATA_LEN] = {0};
uint8 upflow302_finsh_flag = 0;
uint8 upflow302_state_flag = 0;
uint16 upflow302_response_time = 0;
int16 upflow302_x = 0;
int16 upflow302_y = 0;
upflow302_receive_struct upflow302_receive = {0};
uint32 upflow302_frame_seq = 0U;
uint32 upflow302_rx_cycle = 0U;
int16 upflow302_frame_dt_us = 0;

static upflow302_frame_t upflow302_frame_queue[UPFLOW302_FRAME_QUEUE_LEN] = {0};
static volatile uint8 upflow302_frame_queue_head = 0U;
static volatile uint8 upflow302_frame_queue_tail = 0U;
static volatile uint16 upflow302_queue_overflow_count = 0U;
static volatile uint16 upflow302_superseded_frame_count = 0U;
static upflow302_gyro_snapshot_callback_t upflow302_gyro_snapshot_callback = NULL;
static uint32 upflow302_pending_frame_start_cycle = 0U;
static float upflow302_pending_gyro_total_x_rad = 0.0f;
static float upflow302_pending_gyro_total_y_rad = 0.0f;
static float upflow302_pending_gyro_total_z_rad = 0.0f;
static float upflow302_pending_yaw_deg = 0.0f;
static uint8 upflow302_pending_gyro_snapshot_valid = 0U;
static uint32 upflow302_previous_frame_start_cycle = 0U;
static float upflow302_previous_gyro_total_x_rad = 0.0f;
static float upflow302_previous_gyro_total_y_rad = 0.0f;
static float upflow302_previous_gyro_total_z_rad = 0.0f;
static float upflow302_previous_yaw_deg = 0.0f;
static uint8 upflow302_previous_gyro_snapshot_valid = 0U;

static void upflow302_queue_frame(int16 flow_x,
                                  int16 flow_y,
                                  int16 frame_dt_us,
                                  uint32 frame_seq,
                                  uint32 frame_start_cycle,
                                  uint32 gyro_window_cycles,
                                  float gyro_x_delta_rad,
                                  float gyro_y_delta_rad,
                                  float gyro_z_delta_rad,
                                  float yaw_mid_deg,
                                  uint8 gyro_window_valid,
                                  uint8 yaw_window_valid)
{
    uint8 next_head = (uint8)((upflow302_frame_queue_head + 1U) % UPFLOW302_FRAME_QUEUE_LEN);

    if(next_head == upflow302_frame_queue_tail)
    {
        /* SPSC队列满时丢弃新帧，生产者不修改消费者拥有的tail。 */
        if(upflow302_queue_overflow_count < 0xFFFFU)
        {
            upflow302_queue_overflow_count++;
        }
        return;
    }

    upflow302_frame_queue[upflow302_frame_queue_head].flow_x = flow_x;
    upflow302_frame_queue[upflow302_frame_queue_head].flow_y = flow_y;
    upflow302_frame_queue[upflow302_frame_queue_head].frame_dt_us = frame_dt_us;
    upflow302_frame_queue[upflow302_frame_queue_head].frame_seq = frame_seq;
    upflow302_frame_queue[upflow302_frame_queue_head].frame_start_cycle = frame_start_cycle;
    upflow302_frame_queue[upflow302_frame_queue_head].gyro_window_cycles = gyro_window_cycles;
    upflow302_frame_queue[upflow302_frame_queue_head].gyro_x_delta_rad = gyro_x_delta_rad;
    upflow302_frame_queue[upflow302_frame_queue_head].gyro_y_delta_rad = gyro_y_delta_rad;
    upflow302_frame_queue[upflow302_frame_queue_head].gyro_z_delta_rad = gyro_z_delta_rad;
    upflow302_frame_queue[upflow302_frame_queue_head].yaw_mid_deg = yaw_mid_deg;
    upflow302_frame_queue[upflow302_frame_queue_head].gyro_window_valid = gyro_window_valid;
    upflow302_frame_queue[upflow302_frame_queue_head].yaw_window_valid = yaw_window_valid;
    __DMB();
    upflow302_frame_queue_head = next_head;
}

static void upflow302_capture_frame_start(void)
{
    upflow302_pending_frame_start_cycle = DWT->CYCCNT;
    upflow302_pending_gyro_snapshot_valid = 0U;

    if(upflow302_gyro_snapshot_callback != NULL)
    {
        upflow302_pending_gyro_snapshot_valid =
            upflow302_gyro_snapshot_callback(upflow302_pending_frame_start_cycle,
                                             &upflow302_pending_gyro_total_x_rad,
                                             &upflow302_pending_gyro_total_y_rad,
                                             &upflow302_pending_gyro_total_z_rad,
                                             &upflow302_pending_yaw_deg);
    }
}

static uint8 upflow302_finish_gyro_window(float *gyro_x_delta_rad,
                                          float *gyro_y_delta_rad,
                                          float *gyro_z_delta_rad,
                                          uint32 *gyro_window_cycles,
                                          float *yaw_mid_deg,
                                          uint8 *yaw_window_valid)
{
    uint8 window_valid = 0U;

    if(gyro_x_delta_rad == NULL || gyro_y_delta_rad == NULL ||
       gyro_z_delta_rad == NULL || gyro_window_cycles == NULL ||
       yaw_mid_deg == NULL || yaw_window_valid == NULL)
    {
        return 0U;
    }

    *gyro_x_delta_rad = 0.0f;
    *gyro_y_delta_rad = 0.0f;
    *gyro_z_delta_rad = 0.0f;
    *gyro_window_cycles = 0U;
    *yaw_mid_deg = 0.0f;
    *yaw_window_valid = 0U;

    if(upflow302_pending_gyro_snapshot_valid && upflow302_previous_gyro_snapshot_valid)
    {
        *gyro_x_delta_rad =
            upflow302_pending_gyro_total_x_rad - upflow302_previous_gyro_total_x_rad;
        *gyro_y_delta_rad =
            upflow302_pending_gyro_total_y_rad - upflow302_previous_gyro_total_y_rad;
        *gyro_z_delta_rad =
            upflow302_pending_gyro_total_z_rad - upflow302_previous_gyro_total_z_rad;
        *gyro_window_cycles =
            upflow302_pending_frame_start_cycle - upflow302_previous_frame_start_cycle;
        window_valid = 1U;

        {
            float yaw_delta_deg = upflow302_pending_yaw_deg - upflow302_previous_yaw_deg;

            while(yaw_delta_deg > 180.0f) yaw_delta_deg -= 360.0f;
            while(yaw_delta_deg < -180.0f) yaw_delta_deg += 360.0f;
            *yaw_mid_deg = upflow302_previous_yaw_deg + 0.5f * yaw_delta_deg;
            while(*yaw_mid_deg > 180.0f) *yaw_mid_deg -= 360.0f;
            while(*yaw_mid_deg < -180.0f) *yaw_mid_deg += 360.0f;
            *yaw_window_valid = 1U;
        }
    }

    /* 每个完整协议帧都推进时间基线，坏数据不会污染下一帧的窗口长度。 */
    upflow302_previous_frame_start_cycle = upflow302_pending_frame_start_cycle;
    upflow302_previous_gyro_total_x_rad = upflow302_pending_gyro_total_x_rad;
    upflow302_previous_gyro_total_y_rad = upflow302_pending_gyro_total_y_rad;
    upflow302_previous_gyro_total_z_rad = upflow302_pending_gyro_total_z_rad;
    upflow302_previous_yaw_deg = upflow302_pending_yaw_deg;
    upflow302_previous_gyro_snapshot_valid = upflow302_pending_gyro_snapshot_valid;
    return window_valid;
}

static void upflow302_parse_byte(uint8 receive_data)
{
    static vuint8 length = 0;
    uint8 parity_bit_sum = 0;
    uint8 parity_bit = 0;

    if(0U == length)
    {
        if(UP_FLOW_302_FRAME_START != receive_data)
        {
            return;
        }
        upflow302_capture_frame_start();
    }

    upflow302_receiver_data[length++] = receive_data;

    if((2U == length) && (UP_FLOW_302_DEVICE_ID != upflow302_receiver_data[1]))
    {
        length = 0;
        return;
    }

    if(UP_FLOW_302_DATA_LEN <= length)
    {
        float gyro_x_delta_rad = 0.0f;
        float gyro_y_delta_rad = 0.0f;
        float gyro_z_delta_rad = 0.0f;
        float yaw_mid_deg = 0.0f;
        uint32 gyro_window_cycles = 0U;
        uint8 yaw_window_valid = 0U;
        uint8 gyro_window_valid =
            upflow302_finish_gyro_window(&gyro_x_delta_rad,
                                         &gyro_y_delta_rad,
                                         &gyro_z_delta_rad,
                                         &gyro_window_cycles,
                                         &yaw_mid_deg,
                                         &yaw_window_valid);

        parity_bit = upflow302_receiver_data[12];
        upflow302_receiver_data[12] = 0;

        for(uint8 index = 2U; index < 12U; index++)
        {
            parity_bit_sum ^= upflow302_receiver_data[index];
        }

        upflow302_receiver_data[12] = parity_bit;

        if(parity_bit_sum == parity_bit)
        {
            upflow302_state_flag = 1;
            upflow302_response_time = 0;
            memcpy((uint8 *)&upflow302_receive, (uint8 *)upflow302_receiver_data, sizeof(upflow302_receive));
            upflow302_x = upflow302_receive.upflow302_x;
            upflow302_y = upflow302_receive.upflow302_y;
            upflow302_rx_cycle = upflow302_pending_frame_start_cycle;
            upflow302_frame_dt_us = upflow302_receive.upflow302_us;
            if(UP_FLOW_302_VALID_VALUE == upflow302_receive.upflow302_valid)
            {
                upflow302_frame_seq++;
                upflow302_queue_frame(upflow302_receive.upflow302_x,
                                      upflow302_receive.upflow302_y,
                                      upflow302_receive.upflow302_us,
                                      upflow302_frame_seq,
                                      upflow302_pending_frame_start_cycle,
                                      gyro_window_cycles,
                                      gyro_x_delta_rad,
                                      gyro_y_delta_rad,
                                      gyro_z_delta_rad,
                                      yaw_mid_deg,
                                      gyro_window_valid,
                                      yaw_window_valid);
                upflow302_finsh_flag = 1;
            }
            else
            {
                upflow302_finsh_flag =
                    (uint8)(upflow302_frame_queue_head != upflow302_frame_queue_tail);
            }
        }
        else
        {
            upflow302_finsh_flag =
                (uint8)(upflow302_frame_queue_head != upflow302_frame_queue_tail);
            upflow302_state_flag = 0;
        }

        length = 0;
    }
}

void upflow302_receive_callback(void)
{
    uint8 receive_data = 0;

    while(uart_query_byte(UP_FLOW_302_UART_INDEX, &receive_data))
    {
        upflow302_parse_byte(receive_data);
    }
}

void upflow302_receive_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U)
    {
        DWT->LAR = 0xC5ACCE55UL;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    upflow302_finsh_flag = 0;
    upflow302_state_flag = 0;
    upflow302_response_time = 0;
    upflow302_frame_seq = 0U;
    upflow302_rx_cycle = 0U;
    upflow302_frame_dt_us = 0;
    upflow302_frame_queue_head = 0U;
    upflow302_frame_queue_tail = 0U;
    upflow302_queue_overflow_count = 0U;
    upflow302_superseded_frame_count = 0U;
    upflow302_pending_frame_start_cycle = 0U;
    upflow302_pending_gyro_total_x_rad = 0.0f;
    upflow302_pending_gyro_total_y_rad = 0.0f;
    upflow302_pending_gyro_total_z_rad = 0.0f;
    upflow302_pending_yaw_deg = 0.0f;
    upflow302_pending_gyro_snapshot_valid = 0U;
    upflow302_previous_frame_start_cycle = 0U;
    upflow302_previous_gyro_total_x_rad = 0.0f;
    upflow302_previous_gyro_total_y_rad = 0.0f;
    upflow302_previous_gyro_total_z_rad = 0.0f;
    upflow302_previous_yaw_deg = 0.0f;
    upflow302_previous_gyro_snapshot_valid = 0U;
    memset(upflow302_receiver_data, 0, sizeof(upflow302_receiver_data));
    memset((uint8 *)&upflow302_receive, 0, sizeof(upflow302_receive));

    uart_init(UP_FLOW_302_UART_INDEX,
              UP_FLOW_302_BAUDRATE,
              UP_FLOW_302_UART_TX_PIN,
              UP_FLOW_302_UART_RX_PIN);
    uart_rx_interrupt(UP_FLOW_302_UART_INDEX, 1);
}

void upflow302_set_gyro_snapshot_callback(upflow302_gyro_snapshot_callback_t callback)
{
    upflow302_gyro_snapshot_callback = callback;
    upflow302_previous_gyro_snapshot_valid = 0U;
}

uint8 upflow302_pop_frame(upflow302_frame_t *frame)
{
    uint8 tail;

    if(frame == NULL)
    {
        return 0U;
    }

    tail = upflow302_frame_queue_tail;
    if(tail == upflow302_frame_queue_head)
    {
        upflow302_finsh_flag = 0U;
        return 0U;
    }

    __DMB();
    *frame = upflow302_frame_queue[tail];
    __DMB();
    upflow302_frame_queue_tail = (uint8)((tail + 1U) % UPFLOW302_FRAME_QUEUE_LEN);
    if(upflow302_frame_queue_tail == upflow302_frame_queue_head)
    {
        upflow302_finsh_flag = 0U;
    }
    return 1U;
}

uint8 upflow302_pop_latest_frame(upflow302_frame_t *frame)
{
    upflow302_frame_t newer_frame;

    if(!upflow302_pop_frame(frame))
    {
        return 0U;
    }

    while(upflow302_pop_frame(&newer_frame))
    {
        *frame = newer_frame;
        if(upflow302_superseded_frame_count < 0xFFFFU)
        {
            upflow302_superseded_frame_count++;
        }
    }
    return 1U;
}

uint16 upflow302_get_dropped_frame_count(void)
{
    uint32 total = (uint32)upflow302_queue_overflow_count +
                   (uint32)upflow302_superseded_frame_count;
    return (total > 0xFFFFU) ? 0xFFFFU : (uint16)total;
}
