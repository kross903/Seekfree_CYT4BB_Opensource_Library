#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 定义通讯参数结构体

#define SMALL_DRIVER_FRAME_LEN          11U
#define SMALL_DRIVER_SERVICE_PERIOD_US  500U

static uint8 s_pending_frame[SMALL_DRIVER_FRAME_LEN] = {0};
static uint8 s_pending_valid = 0U;
static SmallDriverUartDiag s_uart_diag = {0};

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
    uint8 receive_data;                                                                     // 定义临时变量

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前接收内容中是否正确包含帧头
        {
            motor_value.receive_data_count = 0;                                             // 未收到帧头或者未正确包含帧头则重新接收
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // 保存串口数据

        if(motor_value.receive_data_count >= 11)                                            // 判断是否接收到指定数量的数据
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {
                motor_value.sum_check_data = 0;                                             // 清除校验位数据

                for(int i = 0; i < 10; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[10])       // 校验数据准确性
                {
                    for(int i = 0; i < 4; i ++)
                    {
                        motor_value.receive_speed_data[i]   = (((int)motor_value.receive_data_buffer[i * 2 + 2] << 8) | (int)motor_value.receive_data_buffer[i * 2 + 3]);
                    }

                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清除缓冲区数据
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清除缓冲区数据
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // 清除缓冲区计数值

                memset(motor_value.receive_data_buffer, 0, 11);                             // 清除缓冲区数据
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

void small_driver_service_tx(void)
{
    volatile stc_SCB_t *uart = get_scb_module(SMALL_DRIVER_UART);
    uint32 copied;

    if(!s_pending_valid)
    {
        return;
    }

    if(!Cy_SCB_UART_IsTxComplete(uart))
    {
        uint32 next_age_us = (uint32)s_uart_diag.pending_age_us + SMALL_DRIVER_SERVICE_PERIOD_US;
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
    s_pending_valid = 0U;
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
    small_driver_service_tx();

    small_driver_get_speed();                                                                   // 获取实时速度数据
}
