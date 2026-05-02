#include "communication/vofa.h"
#include "common/project_headfile.h"
#include <math.h>
#include <string.h>

/*
 * VOFA 无线调参模块。
 * UART 接收缓冲、组包过程、解析辅助以及回传模式状态都由本文件私有维护，
 * 不再通过 vofa.h 向外暴露内部实现细节。
 */

/*
 * VOFA 串口调参模块:
 * 负责串口收发、命令解析，以及把上位机调参命令转换成 Flight_Control_* 调用。
 */

#define VOFA_PACKET_MAX_LEN        64
#define VOFA_PACKET_QUEUE_DEPTH    4

static uint8 uart_get_data[VOFA_PACKET_MAX_LEN];
static uint8 fifo_get_data[VOFA_PACKET_MAX_LEN];
static uint8 get_data = 0;
static fifo_struct uart_data_fifo;

static void send_four_date(float date_1, float date_2, float date_3, float date_4,
                           float date_5, float date_6, float date_7, float date_8);

// 0: 默认飞行状态回传  1: PID 参数回读  2: 定位/高度观察  3: VOFA 无线链路测试
static int send_back_flag = 0;
static uint8 send_back_command = 'R';

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 把“串口收到字节”和“主循环执行命令”拆开，避免旧命令反复执行。
 * 2. 给 VOFA 命令增加一个很轻量的包队列，尽量保持原有主流程不变。
 */
static uint8 vofa_partial_packet[VOFA_PACKET_MAX_LEN] = {0};
static uint8 vofa_partial_length = 0;
static uint8 vofa_packet_queue[VOFA_PACKET_QUEUE_DEPTH][VOFA_PACKET_MAX_LEN] = {{0}};
static uint8 vofa_packet_queue_head = 0;
static uint8 vofa_packet_queue_tail = 0;
static uint8 vofa_packet_queue_count = 0;
static uint8 vofa_packet_ready = 0;

static float build_vofa_test_channel(void)
{
    static float test_value = 0.0f;
    static float step = 2.5f;

    test_value += step;

    if(test_value >= 100.0f)
    {
        test_value = 100.0f;
        step = -2.5f;
    }
    else if(test_value <= 0.0f)
    {
        test_value = 0.0f;
        step = 2.5f;
    }

    return test_value;
}

static void set_angle_pid_value(uint8 axis, uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    Flight_Control_Get_Angle_PID(axis, &kp, &ki, &kd);

    switch(item)
    {
        case '1': kp = value; break;
        case '2': ki = value; break;
        case '3': kd = value; break;
        default:  return;
    }

    Flight_Control_Set_Angle_PID(axis, kp, ki, kd);
}

static void set_gyro_pid_value(uint8 axis, uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float out_limit = 0.0f;
    float int_limit = 0.0f;

    switch(item)
    {
        case '1':
        case '2':
        case '3':
            Flight_Control_Get_Gyro_PID(axis, &kp, &ki, &kd);
            if(item == '1')
            {
                kp = value;
            }
            else if(item == '2')
            {
                ki = value;
            }
            else
            {
                kd = value;
            }
            Flight_Control_Set_Gyro_PID(axis, kp, ki, kd);
            break;

        case '4':
            Flight_Control_Get_Gyro_Limits(axis, &out_limit, &int_limit);
            Flight_Control_Set_Gyro_Limits(axis, out_limit, value);
            break;

        default:
            break;
    }
}

static void set_height_pid_value(uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    Flight_Control_Get_Height_PID(&kp, &ki, &kd);

    switch(item)
    {
        case '1': kp = value; break;
        case '2': ki = value; break;
        case '3': kd = value; break;
        default:  return;
    }

    Flight_Control_Set_Height_PID(kp, ki, kd);
}

static void set_joystick_pid_value(uint8 axis, uint8 item, float value)
{
    PID_Controller *pid = NULL;

    if(axis == AXIS_ROLL)
    {
        pid = &joystick_pid_roll;
    }
    else if(axis == AXIS_PITCH)
    {
        pid = &joystick_pid_pitch;
    }
    else
    {
        return;
    }

    switch(item)
    {
        case '1': pid->Kp = value; break;
        case '2': pid->Ki = value; break;
        case '3': pid->Kd = value; break;
        case '4': pid->output_limit = value; break;
        case '5': pid->integral_limit = value; break;
        default:  return;
    }
}
    float roll_compensation = 0.0f;
    float pitch_compensation = 0.0f;
    float yaw_compensation = 0.0f;
static void set_angle_compensation_value(uint8 item, float value)
{


    Flight_Control_Get_Angle_Compensation(&roll_compensation, &pitch_compensation, &yaw_compensation);

    switch(item)
    {
        case '1': roll_compensation = value; break;
        case '2': pitch_compensation = value; break;
        case '3': yaw_compensation = value; break;
        default:  return;
    }

    Flight_Control_Set_Angle_Compensation(roll_compensation, pitch_compensation, yaw_compensation);
}

/*
 * VOFA 调参：陀螺去旋转增益。
 *   item = 'x'/'X': 设置 K_X（flow_x_rad 对 gyro_y_delta_rad 的去旋转增益）
 *   item = 'y'/'Y': 设置 K_Y（flow_y_rad 对 gyro_x_delta_rad 的去旋转增益）
 * 范围由 PMW_Flow_SetGyroCompGain() 内部夹位，这里负值才表示“不变更”，
 * 所以已 has_value 的正常调参直接传 value 即可。
 */
static void set_pmw_flow_gyro_gain_value(uint8 item, float value)
{
    if(value < 0.0f)
    {
        return;
    }

    switch(item)
    {
        case 'x':
        case 'X':
            PMW_Flow_SetGyroCompGain(value, -1.0f);
            break;
        case 'y':
        case 'Y':
            PMW_Flow_SetGyroCompGain(-1.0f, value);
            break;
        default:
            break;
    }
}

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
static void send_pid_data_back(uint8 command)
{
    float value_1 = 0.0f;
    float value_2 = 0.0f;
    float value_3 = 0.0f;

    switch(command)
    {
        case 'R':
            Flight_Control_Get_Angle_PID(AXIS_ROLL, &value_1, &value_2, &value_3);
            break;

        case 'r':
            Flight_Control_Get_Gyro_PID(AXIS_ROLL, &value_1, &value_2, &value_3);
            break;

        case 'P':
            Flight_Control_Get_Angle_PID(AXIS_PITCH, &value_1, &value_2, &value_3);
            break;

        case 'p':
            Flight_Control_Get_Gyro_PID(AXIS_PITCH, &value_1, &value_2, &value_3);
            break;

        case 'Y':
            Flight_Control_Get_Angle_PID(AXIS_YAW, &value_1, &value_2, &value_3);
            break;

        case 'y':
            Flight_Control_Get_Gyro_PID(AXIS_YAW, &value_1, &value_2, &value_3);
            break;

        case 'H':
            Flight_Control_Get_Height_PID(&value_1, &value_2, &value_3);
            break;

        case 'J':
            value_1 = joystick_pid_roll.Kp;
            value_2 = joystick_pid_roll.Ki;
            value_3 = joystick_pid_roll.Kd;
            send_four_date(value_1, value_2, value_3,
                           joystick_pid_roll.output_limit,
                           joystick_pid_roll.integral_limit,
                           0, 0, 0);
            return;

        case 'K':
            value_1 = joystick_pid_pitch.Kp;
            value_2 = joystick_pid_pitch.Ki;
            value_3 = joystick_pid_pitch.Kd;
            send_four_date(value_1, value_2, value_3,
                           joystick_pid_pitch.output_limit,
                           joystick_pid_pitch.integral_limit,
                           0, 0, 0);
            return;

        case 'S':
            value_1 = Flight_Control_Get_Target_Throttle();
            break;

        case 'T':
            Flight_Control_Get_Target_Angle(&roll, &pitch, &yaw);
            Flight_Control_Get_Angle_Compensation(&roll_compensation, &pitch_compensation, NULL);
            send_four_date(current_roll, current_pitch, current_yaw, icm42688_acc_x,
                icm42688_acc_y, icm42688_acc_z,
                roll_compensation, pitch_compensation);

            return;

        case 'G':
        {
            float gx_now = 0.0f;
            float gy_now = 0.0f;
            PMW_Flow_GetGyroCompGain(&gx_now, &gy_now);
            send_four_date(gx_now, gy_now, 0, 0, 0, 0, 0, 0);
            return;
        }

        default:
            return;
    }

    send_four_date(value_1, value_2, value_3, 0,
                   0, 0,
                   0, 0);
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. `fifo_get_data` 只保存当前正在执行的一条完整命令包。
 * 2. 通过小队列缓存后续命令，避免收到多条完整命令时相互覆盖。
 */
static void enqueue_vofa_packet(const uint8 *packet, uint8 length)
{
    if(packet == NULL || length == 0)
    {
        return;
    }

    if(vofa_packet_queue_count >= VOFA_PACKET_QUEUE_DEPTH)
    {
        vofa_packet_queue_head++;
        if(vofa_packet_queue_head >= VOFA_PACKET_QUEUE_DEPTH)
        {
            vofa_packet_queue_head = 0;
        }
        vofa_packet_queue_count--;
    }

    memset(vofa_packet_queue[vofa_packet_queue_tail], 0, VOFA_PACKET_MAX_LEN);
    memcpy(vofa_packet_queue[vofa_packet_queue_tail], packet, length);

    vofa_packet_queue_tail++;
    if(vofa_packet_queue_tail >= VOFA_PACKET_QUEUE_DEPTH)
    {
        vofa_packet_queue_tail = 0;
    }
    vofa_packet_queue_count++;
}

static uint8 load_next_vofa_packet(void)
{
    if(vofa_packet_ready)
    {
        return 1;
    }

    if(vofa_packet_queue_count == 0)
    {
        return 0;
    }

    memset(fifo_get_data, 0, sizeof(fifo_get_data));
    memcpy(fifo_get_data, vofa_packet_queue[vofa_packet_queue_head], VOFA_PACKET_MAX_LEN);
    memset(vofa_packet_queue[vofa_packet_queue_head], 0, VOFA_PACKET_MAX_LEN);

    vofa_packet_queue_head++;
    if(vofa_packet_queue_head >= VOFA_PACKET_QUEUE_DEPTH)
    {
        vofa_packet_queue_head = 0;
    }
    vofa_packet_queue_count--;
    vofa_packet_ready = 1;
    return 1;
}

static void clear_active_vofa_packet(void)
{
    memset(fifo_get_data, 0, sizeof(fifo_get_data));
    vofa_packet_ready = 0;
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 只有在收到终止符 `!` 后，才把当前字节流视为一条完整命令。
 * 2. 自动丢弃超长包和换行噪声，避免缓冲区残留旧命令。
 */
static void collect_vofa_byte(uint8 byte)
{
    if(byte == '\r' || byte == '\n')
    {
        return;
    }

    if(vofa_partial_length >= (VOFA_PACKET_MAX_LEN - 1))
    {
        vofa_partial_length = 0;
    }

    vofa_partial_packet[vofa_partial_length++] = byte;

    if(byte == '!')
    {
        enqueue_vofa_packet(vofa_partial_packet, vofa_partial_length);
        memset(vofa_partial_packet, 0, sizeof(vofa_partial_packet));
        vofa_partial_length = 0;
    }
}

static void send_four_date(float date_1, float date_2, float date_3, float date_4,
                           float date_5, float date_6, float date_7, float date_8)
{
    char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    float byte[8] = {0};

    byte[0] = date_1;
    byte[1] = date_2;
    byte[2] = date_3;
    byte[3] = date_4;
    byte[4] = date_5;
    byte[5] = date_6;
    byte[6] = date_7;
    byte[7] = date_8;

    uart_write_buffer(UART_INDEX, (const uint8 *)byte, 4 * 8);
    uart_write_buffer(UART_INDEX, (const uint8 *)tail, 4);
}

void Init_uart(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, VOFA_PACKET_MAX_LEN);
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    uart_rx_interrupt(UART_INDEX, 1);

    uart_write_string(UART_INDEX, "UART Text.");
    uart_write_byte(UART_INDEX, '\r');
    uart_write_byte(UART_INDEX, '\n');
}

void Send_test(void)
{
    uint8 rx_buffer[VOFA_PACKET_MAX_LEN] = {0};
    uint32 fifo_data_count = 0;
    uint32 read_count = 0;
    uint32 index = 0;

    fifo_data_count = fifo_used(&uart_data_fifo);
    if(fifo_data_count != 0)
    {
        read_count = fifo_data_count;
        fifo_read_buffer(&uart_data_fifo, rx_buffer, &read_count, FIFO_READ_AND_CLEAN);

        for(index = 0; index < read_count; index++)
        {
            collect_vofa_byte(rx_buffer[index]);
        }
    }

    system_delay_ms(10);
}

void uart_rx_interrupt_handler(void)
{
//  get_data = uart_read_byte(UART_INDEX);
    uart_query_byte(UART_INDEX, &get_data);
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 解析函数显式返回成功/失败，避免坏包被默认解析成 0。
 * 2. 只有在 `=` 和 `!` 都完整存在时，才允许把数据交给控制层使用。
 */
static uint8 Get_Data(float *value)
{
    uint8 data_start_num = 0;
    uint8 data_end_num = 0;
    uint8 minus_flag = 0;
    uint8 has_point = 0;
    uint8 valid = 0;
    uint8 cursor = 0;
    uint8 has_digit = 0;
    float integer_part = 0.0f;
    float fractional_part = 0.0f;
    float fractional_scale = 0.1f;

    if(value == NULL)
    {
        return 0;
    }

    // 先扫描当前命令包，找到数字区间。
    // 例如 "T2=0.21!" 中，真正的数据部分是 '=' 后、'!' 前的 "0.21"。
    for(cursor = 0; cursor < VOFA_PACKET_MAX_LEN; cursor++)
    {
        if(fifo_get_data[cursor] == '=')
        {
            data_start_num = cursor + 1;
            valid |= 0x01;
        }
        if(fifo_get_data[cursor] == '!')
        {
            data_end_num = cursor;
            valid |= 0x02;
            break;
        }
    }

    if(valid != 0x03 || data_start_num >= data_end_num)
    {
        return 0;
    }

    cursor = data_start_num;
    // 允许数字带一个前导负号，例如 "-0.25"。
    if(fifo_get_data[cursor] == '-')
    {
        minus_flag = 1;
        cursor++;
    }

    /*
     * 下面开始逐字符手动解析数字。
     *
     * 1. 在遇到小数点之前，按“整数部分”处理：
     *    integer_part = integer_part * 10 + 当前数字
     *    例如 "12"：
     *      读到 '1' -> 0 * 10 + 1 = 1
     *      读到 '2' -> 1 * 10 + 2 = 12
     *
     * 2. 遇到小数点之后，按“小数部分”处理：
     *    第一位小数权重是 0.1
     *    第二位小数权重是 0.01
     *    第三位小数权重是 0.001
     *
     * 这里的实现方法是：
     *    - 看到 '.' 时，把 fractional_scale 设为 1.0f
     *    - 每次加入 digit * (fractional_scale * 0.1f)
     *    - 然后再把 fractional_scale 乘 0.1f
     *
     * 例如 "0.21"：
     *    读到小数点后，fractional_scale = 1.0
     *    读到 '2'：2 * (1.0 * 0.1) = 0.2
     *    再把 fractional_scale 变成 0.1
     *    读到 '1'：1 * (0.1 * 0.1) = 0.01
     *    所以小数部分最终是 0.2 + 0.01 = 0.21
     */
    for(; cursor < data_end_num; cursor++)
    {
        uint8 current_char = fifo_get_data[cursor];

        if(current_char == '.')
        {
            // 如果已经进过一次小数部分，又再次遇到 '.'，说明数据格式错误。
            if(has_point)
            {
                return 0;
            }
            // 从这里开始，后续字符按小数部分处理。
            has_point = 1;
            continue;
        }

        if(current_char < '0' || current_char > '9')
        {
            return 0;
        }

        has_digit = 1;
        if(!has_point)
        {
            // 还在整数部分，旧值先左移一位十进制，再加上当前数字。
            integer_part = integer_part * 10.0f + (float)(current_char - '0');
        }
        else
        {
            // 已进入小数部分，当前位按 0.1 / 0.01 / 0.001 ... 这样的权重累加。
            fractional_part += (float)(current_char - '0') * fractional_scale;
            fractional_scale *= 0.1f;
        }
    }

    if(!has_digit)
    {
        return 0;
    }

    *value = integer_part + fractional_part;
    if(minus_flag)
    {
        *value = -(*value);
    }

    return 1;
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 每次主循环最多消费一条完整命令，消费后立刻清空当前活动包。
 * 2. 保持原有外部调用方式不变，尽量不影响当前可运行主流程。
 */
void USART_PID_Adjust(void)
{
    float data_get = 0.0f;
    uint8 has_value = 0;
    uint8 command = 0;
    uint8 item = 0;

    if(!load_next_vofa_packet())
    {
        return;
    }

    command = fifo_get_data[0];
    item = fifo_get_data[1];
    has_value = Get_Data(&data_get);

    switch(command)
    {
        case 'R':
            if(has_value)
            {
                set_angle_pid_value(AXIS_ROLL, item, data_get);
            }
            send_back_command = 'R';
            break;

        case 'r':
            if(has_value)
            {
                set_gyro_pid_value(AXIS_ROLL, item, data_get);
            }
            send_back_command = 'r';
            break;

        case 'P':
            if(has_value)
            {
                set_angle_pid_value(AXIS_PITCH, item, data_get);
            }
            send_back_command = 'P';
            break;

        case 'p':
            if(has_value)
            {
                set_gyro_pid_value(AXIS_PITCH, item, data_get);
            }
            send_back_command = 'p';
            break;

        case 'Y':
            if(has_value)
            {
                set_angle_pid_value(AXIS_YAW, item, data_get);
            }
            send_back_command = 'Y';
            break;

        case 'y':
            if(has_value)
            {
                set_gyro_pid_value(AXIS_YAW, item, data_get);
            }
            send_back_command = 'y';
            break;

        case 'S':
            if(has_value)
            {
                Flight_Control_Set_Throttle(data_get);
            }
            send_back_command = 'S';
            break;

        case 'T':
            if(has_value)
            {
                set_angle_compensation_value(item, data_get);
            }
            send_back_command = 'T';
            break;

        case 'J':
            if(has_value)
            {
                set_joystick_pid_value(AXIS_ROLL, item, data_get);
            }
            send_back_command = 'J';
            break;

        case 'K':
            if(has_value)
            {
                set_joystick_pid_value(AXIS_PITCH, item, data_get);
            }
            send_back_command = 'K';
            break;

        case 'F':
            switch(item)
            {
                case '0': send_back_flag = 0; break;
                case '1': send_back_flag = 1; break;
                case '2': send_back_flag = 2; break;
                case '3': send_back_flag = 3; break;
                case '4': send_back_flag = 4; break;
                case '5': send_back_flag = 5; break;
                case '6': send_back_flag = 6; break;
                case '7': send_back_flag = 7; break;
                default: break;
            }

            break;

        case '0':
            if(item == '0')
            {
                Flight_Control_Disarm();
            }
            break;

        case 'H':
            if(has_value)
            {
                set_height_pid_value(item, data_get);
            }
            send_back_command = 'H';
            break;

        case 'G':
            if(has_value)
            {
                set_pmw_flow_gyro_gain_value(item, data_get);
            }
            send_back_command = 'G';
            break;

        default:
            break;
    }

    clear_active_vofa_packet();
}

void send_date_back(void)
{
    switch(send_back_flag)
    {
        case 0:
        {
            PMWFlowState_t pmw_flow_state = {0};
            PMW_Flow_GetState(&pmw_flow_state);

            send_four_date(pmw_flow_state.flow_x_rad,
                           pmw_flow_state.gyro_flow_y_delta_rad,
                           pmw_flow_state.flow_comp_x_rad,
                           pmw_flow_state.velocity_y_raw_cms,
                           pmw_flow_state.flow_y_rad,
                           pmw_flow_state.gyro_flow_x_delta_rad,
                           pmw_flow_state.flow_comp_y_rad,
                           pmw_flow_state.velocity_x_raw_cms);
            break;
        }

        case 1:
            send_pid_data_back(send_back_command);
            break;

        case 2:
            send_four_date(ir_pose_estimate.x, g_height_result.height_cm,
                           ir_pose_estimate.z, tof_height_true_filt * 100.0f,
                           0, current_roll,
                           current_pitch, current_yaw);
            break;

        case 3:
            send_four_date(build_vofa_test_channel(), current_roll,
                           current_pitch, current_yaw,
                           g_height_result.height_cm, tof_height_true_filt * 100.0f,
                           ir_pose_estimate.z, HeightSpeedPID_output);
            break;

        case 4:
            send_four_date((float)icm42688_gyro_x_r, (float)icm42688_gyro_y_r,
                           (float)icm42688_gyro_z_r, icm42688_gyro_x,
                           icm42688_gyro_y, icm42688_gyro_z,
                           angle_roll_compensation, angle_pitch_compensation);
            break;

        case 5:
            send_four_date((float)icm42688_acc_x_r, (float)icm42688_acc_y_r,
                           (float)icm42688_acc_z_r, icm42688_acc_x,
                           icm42688_acc_y, icm42688_acc_z,
                           angle_roll_compensation, angle_pitch_compensation);
            break;

        case 6:
            send_four_date(current_roll, target_angle_roll,
                           current_gyro_roll, target_roll_rate,
                           current_pitch, target_angle_pitch,
                           current_gyro_pitch, target_pitch_rate);
            break;

        case 7:
            send_four_date((float)adc_x, (float)adc_y,
                           (float)adc_x_filtered, (float)adc_y_filtered,
                           adc_x_nomalize, adc_y_nomalize,
                           target_angle_roll, target_angle_pitch);
            break;

        default:
            break;
    }
}
