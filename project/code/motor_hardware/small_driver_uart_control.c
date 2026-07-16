#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 定义通讯参数结构体

static uint8 s_pending_frame[SMALL_DRIVER_FRAME_LEN] = {0};
static uint8 s_pending_valid = 0U;
static SmallDriverUartDiag s_uart_diag = {0};
static SmallDriverEscStatus s_esc_status = {0};
static SmallDriverEscFaultEvent s_esc_fault_event = {0};
static uint8 s_esc_fault_event_pending = 0U;
static uint8 s_esc_event_sequence_synced = 0U;
static uint8 s_esc_last_event_sequence = 0U;
static uint8 s_esc_had_active_fault = 0U;
static uint8 s_esc_latched_sequence_valid = 0U;

static uint8 small_driver_active_fault_mask(const uint8 active_fault[4])
{
    uint8 mask = 0U;
    uint8 i;

    for(i = 0U; i < 4U; i++)
    {
        if(active_fault[i] != (uint8)SMALL_DRIVER_ESC_FAULT_NONE)
        {
            mask |= (uint8)(1U << i);
        }
    }
    return mask;
}

static void small_driver_publish_fault_event(uint8 sequence, uint8 fault_code, uint8 motor_mask)
{
    uint8 valid_motor_mask = (uint8)(motor_mask & 0x0FU);

    if(fault_code <= (uint8)SMALL_DRIVER_ESC_FAULT_NONE ||
       fault_code > (uint8)SMALL_DRIVER_ESC_FAULT_LOW_SPEED_RESTART ||
       valid_motor_mask == 0U)
    {
        return;
    }

    if(!s_esc_latched_sequence_valid || sequence != s_esc_status.last_event_sequence)
    {
        s_esc_status.last_event_sequence = sequence;
        s_esc_status.last_event_code = fault_code;
        s_esc_status.last_event_motor_mask = valid_motor_mask;
        if(s_esc_status.fault_event_count < 0xFFFFFFFFUL)
        {
            s_esc_status.fault_event_count++;
        }
        s_esc_latched_sequence_valid = 1U;
    }

    if(!s_esc_fault_event_pending)
    {
        s_esc_fault_event.sequence = sequence;
        s_esc_fault_event.fault_code = fault_code;
        s_esc_fault_event.motor_mask = valid_motor_mask;
        s_esc_fault_event_pending = 1U;
    }
}

static void small_driver_parse_speed_frame(const uint8 frame[SMALL_DRIVER_FRAME_LEN])
{
    uint8 i;

    for(i = 0U; i < 4U; i++)
    {
        uint16 raw_speed = (uint16)(((uint16)frame[i * 2U + 2U] << 8) |
                                    (uint16)frame[i * 2U + 3U]);
        motor_value.receive_speed_data[i] = (int16)raw_speed;
    }
    s_esc_status.speed_frame_count++;
}

static void small_driver_parse_status_frame(const uint8 frame[SMALL_DRIVER_FRAME_LEN])
{
    uint8 i;
    uint8 active_mask;
    uint8 first_active_code = 0U;
    uint8 first_active_mask = 0U;

    if(frame[2] != SMALL_DRIVER_ESC_STATUS_VERSION)
    {
        s_esc_status.unknown_frame_count++;
        return;
    }

    s_esc_status.protocol_version = frame[2];
    s_esc_status.event_sequence = frame[3];
    s_esc_status.last_fault_code = frame[4];
    s_esc_status.last_fault_motor_mask = (uint8)(frame[5] & 0x0FU);
    for(i = 0U; i < 4U; i++)
    {
        s_esc_status.active_fault[i] = frame[i + 6U];
    }
    s_esc_status.valid = 1U;
    s_esc_status.stale = 0U;
    s_esc_status.age_ms = 0U;
    s_esc_status.status_frame_count++;

    active_mask = small_driver_active_fault_mask(s_esc_status.active_fault);
    if(!s_esc_event_sequence_synced)
    {
        s_esc_event_sequence_synced = 1U;
        s_esc_last_event_sequence = frame[3];
        if(active_mask != 0U)
        {
            for(i = 0U; i < 4U; i++)
            {
                if(s_esc_status.active_fault[i] != 0U)
                {
                    first_active_code = s_esc_status.active_fault[i];
                    break;
                }
            }
            for(i = 0U; i < 4U; i++)
            {
                if(s_esc_status.active_fault[i] == first_active_code)
                {
                    first_active_mask |= (uint8)(1U << i);
                }
            }
            small_driver_publish_fault_event(frame[3], first_active_code, first_active_mask);
        }
    }
    else if(frame[4] != 0U && frame[3] != s_esc_last_event_sequence)
    {
        s_esc_last_event_sequence = frame[3];
        small_driver_publish_fault_event(frame[3], frame[4], frame[5]);
    }

    if(active_mask != 0U && !s_esc_had_active_fault)
    {
        for(i = 0U; i < 4U; i++)
        {
            if(s_esc_status.active_fault[i] != 0U)
            {
                first_active_code = s_esc_status.active_fault[i];
                break;
            }
        }
        for(i = 0U; i < 4U; i++)
        {
            if(s_esc_status.active_fault[i] == first_active_code)
            {
                first_active_mask |= (uint8)(1U << i);
            }
        }
        small_driver_publish_fault_event(frame[3], first_active_code, first_active_mask);
    }
    s_esc_had_active_fault = (uint8)(active_mask != 0U);
}

static void small_driver_dispatch_rx_frame(const uint8 frame[SMALL_DRIVER_FRAME_LEN])
{
    if(frame[1] == SMALL_DRIVER_SPEED_FUNCTION)
    {
        small_driver_parse_speed_frame(frame);
    }
    else if(frame[1] == SMALL_DRIVER_ESC_STATUS_FUNCTION)
    {
        small_driver_parse_status_frame(frame);
    }
    else
    {
        s_esc_status.unknown_frame_count++;
    }
}

static void small_driver_resync_rx_buffer(void)
{
    uint8 next_start = 1U;

    while(next_start < SMALL_DRIVER_FRAME_LEN &&
          motor_value.receive_data_buffer[next_start] != 0xA5U)
    {
        next_start++;
    }
    if(next_start < SMALL_DRIVER_FRAME_LEN)
    {
        uint8 remaining = (uint8)(SMALL_DRIVER_FRAME_LEN - next_start);
        memmove(motor_value.receive_data_buffer,
                &motor_value.receive_data_buffer[next_start],
                remaining);
        motor_value.receive_data_count = remaining;
    }
    else
    {
        motor_value.receive_data_count = 0U;
    }
}

static void small_driver_build_duty_frame(uint8 frame[SMALL_DRIVER_FRAME_LEN],
                                          int16 motor_duty_1,
                                          int16 motor_duty_2,
                                          int16 motor_duty_3,
                                          int16 motor_duty_4)
{
    int16 duty[4];
    uint8 i;

    duty[0] = motor_duty_1;
    duty[1] = motor_duty_2;
    duty[2] = motor_duty_3;
    duty[3] = motor_duty_4;
    frame[0] = 0xA5U;
    frame[1] = 0x01U;

    for(i = 0U; i < 4U; i++)
    {
        frame[i * 2U + 2U] = (uint8)(((uint16)duty[i] >> 8) & 0xFFU);
        frame[i * 2U + 3U] = (uint8)((uint16)duty[i] & 0xFFU);
    }

    frame[10] = 0U;
    for(i = 0U; i < 10U; i++)
    {
        frame[10] += frame[i];
    }
}

static void small_driver_queue_frame(const uint8 frame[SMALL_DRIVER_FRAME_LEN])
{
    if(s_pending_valid)
    {
        s_uart_diag.overwritten_frame_count++;
    }
    memcpy(s_pending_frame, frame, SMALL_DRIVER_FRAME_LEN);
    s_pending_valid = 1U;
    s_uart_diag.pending = 1U;
    s_uart_diag.pending_age_us = 0U;
    s_uart_diag.queued_frame_count++;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;

    while(uart_query_byte(SMALL_DRIVER_UART, &receive_data))
    {
        uint8 i;

        if(motor_value.receive_data_count == 0U)
        {
            if(receive_data != 0xA5U)
            {
                continue;
            }
            motor_value.receive_data_buffer[0] = receive_data;
            motor_value.receive_data_count = 1U;
            continue;
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count++] = receive_data;
        if(motor_value.receive_data_count >= SMALL_DRIVER_FRAME_LEN)
        {
            motor_value.sum_check_data = 0U;
            for(i = 0U; i < 10U; i++)
            {
                motor_value.sum_check_data += motor_value.receive_data_buffer[i];
            }

            if(motor_value.sum_check_data == motor_value.receive_data_buffer[10])
            {
                small_driver_dispatch_rx_frame(motor_value.receive_data_buffer);
                motor_value.receive_data_count = 0U;
            }
            else
            {
                s_esc_status.checksum_error_count++;
                small_driver_resync_rx_buffer();
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 参数说明     right_duty      右侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000, 1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4)
{
    small_driver_build_duty_frame(motor_value.send_data_buffer,
                                  motor_duty_1,
                                  motor_duty_2,
                                  motor_duty_3,
                                  motor_duty_4);
    small_driver_queue_frame(motor_value.send_data_buffer);
}

void small_driver_service_tx(uint16 elapsed_us)
{
    volatile stc_SCB_t *uart = get_scb_module(SMALL_DRIVER_UART);
    uint32 copied;

    if(!s_pending_valid)
    {
        return;
    }

    if(!Cy_SCB_UART_IsTxComplete(uart))
    {
        uint32 next_age_us = (uint32)s_uart_diag.pending_age_us + elapsed_us;
        s_uart_diag.pending_age_us = (uint16)((next_age_us > 0xFFFFU) ? 0xFFFFU : next_age_us);
        if(s_uart_diag.pending_age_us > s_uart_diag.max_pending_age_us)
        {
            s_uart_diag.max_pending_age_us = s_uart_diag.pending_age_us;
        }
        return;
    }

    copied = Cy_SCB_UART_PutArray(uart, s_pending_frame, SMALL_DRIVER_FRAME_LEN);
    if(copied != SMALL_DRIVER_FRAME_LEN)
    {
        Cy_SCB_UART_ClearTxFifo(uart);
        s_uart_diag.fifo_error_count++;
        return;
    }

    s_pending_valid = 0U;
    s_uart_diag.pending = 0U;
    s_uart_diag.pending_age_us = 0U;
    s_uart_diag.sent_frame_count++;
}

void small_driver_force_zero(void)
{
    uint8 zero_frame[SMALL_DRIVER_FRAME_LEN];

    small_driver_build_duty_frame(zero_frame, 0, 0, 0, 0);
    small_driver_queue_frame(zero_frame);
    s_uart_diag.zero_frame_count++;
}

void small_driver_get_diag(SmallDriverUartDiag *diag)
{
    if(diag != NULL)
    {
        uint32 irq_state = __get_PRIMASK();

        __disable_irq();
        *diag = s_uart_diag;
        if(irq_state == 0U)
        {
            __enable_irq();
        }
    }
}

void small_driver_get_esc_status(SmallDriverEscStatus *status)
{
    if(status != NULL)
    {
        uint32 irq_state = __get_PRIMASK();

        __disable_irq();
        *status = s_esc_status;
        if(irq_state == 0U)
        {
            __enable_irq();
        }
    }
}

uint8 small_driver_take_fault_event(SmallDriverEscFaultEvent *event)
{
    uint32 irq_state = __get_PRIMASK();
    uint8 available;

    __disable_irq();
    available = s_esc_fault_event_pending;
    if(available)
    {
        if(event != NULL)
        {
            *event = s_esc_fault_event;
        }
        s_esc_fault_event_pending = 0U;
    }
    if(irq_state == 0U)
    {
        __enable_irq();
    }
    return available;
}

uint8 small_driver_esc_has_active_fault(void)
{
    uint32 irq_state = __get_PRIMASK();
    uint8 active;

    __disable_irq();
    active = (uint8)(small_driver_active_fault_mask(s_esc_status.active_fault) != 0U);
    if(irq_state == 0U)
    {
        __enable_irq();
    }
    return active;
}

void small_driver_tick_5ms(void)
{
    if(s_esc_status.valid)
    {
        if(s_esc_status.age_ms <= (uint16)(0xFFFFU - 5U))
        {
            s_esc_status.age_ms = (uint16)(s_esc_status.age_ms + 5U);
        }
        else
        {
            s_esc_status.age_ms = 0xFFFFU;
        }
        s_esc_status.stale = (uint8)(s_esc_status.age_ms >= SMALL_DRIVER_ESC_STATUS_STALE_MS);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[6] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[7] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[8] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[9] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[10] = 0xA7;                                        // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);         // 发送获取转速数据的 字节包 数据
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, SMALL_DRIVER_FRAME_LEN);         // 清除缓冲区数据

    memset(motor_value.receive_data_buffer, 0, 11);                          // 清除缓冲区数据

    memset(motor_value.receive_speed_data, 0, 8);                            // 清除接收数据

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;
    memset(s_pending_frame, 0, sizeof(s_pending_frame));
    memset(&s_uart_diag, 0, sizeof(s_uart_diag));
    memset(&s_esc_status, 0, sizeof(s_esc_status));
    memset(&s_esc_fault_event, 0, sizeof(s_esc_fault_event));
    s_pending_valid = 0U;
    s_esc_fault_event_pending = 0U;
    s_esc_event_sequence_synced = 0U;
    s_esc_last_event_sequence = 0U;
    s_esc_had_active_fault = 0U;
    s_esc_latched_sequence_valid = 0U;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化

    small_driver_force_zero();                                                                  // 设置0占空比
    small_driver_service_tx(0U);

    small_driver_get_speed();                                                                   // 获取实时速度数据
}
