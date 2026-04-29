#include "common/project_headfile.h"

extern volatile float car_bat_voltage;

#define CORE_LINK_USER_KEY_QUEUE_SIZE       (8U)
#define CORE_LINK_USER_KEY_ACTIVE_THRESHOLD (0.5f)

/* ====================== 遥感 X/Y 转发到第二主控板 ======================
 * 小车通过 core_link (UART4) 把摇杆 adc_x/adc_y 推给飞控；这里把同一对值
 * 用 *相同* 的 core_link 帧格式（SOF1/SOF2 + protocol_id + len + payload + xor），
 * 通过另一条 UART 原样转发给第二块主控板。对面只要跑同一份 core_link 解析，
 * 在 core_link_on_param_received 里看到 'A''1'/'A''2' 即可，无需任何改动。
 *
 * 默认 UART_3 + P17_2(TX) / P17_1(RX)，115200 8N1，只发不收。若硬件用的是
 * UART3 另一组引脚 (P13_1/P13_0) 或别的串口，改下面四个宏即可。RX 中断
 * 始终不开，cm7_0_isr.c::uart3_isr 保留为空。 */
#define XY_LINK_UART_INDEX      (UART_3)
#define XY_LINK_UART_BAUDRATE   (115200)
#define XY_LINK_UART_TX_PIN     (UART3_TX_P17_2)
#define XY_LINK_UART_RX_PIN     (UART3_RX_P17_1)

static volatile uint8 s_core_link_user_key_queue[CORE_LINK_USER_KEY_QUEUE_SIZE] = {0};
static volatile uint8 s_core_link_user_key_queue_head = 0;
static volatile uint8 s_core_link_user_key_queue_tail = 0;
static volatile uint8 s_core_link_user_monitor_exit_request = 0;
static char s_core_link_user_text_receive[CORE_LINK_PAYLOAD_MAX_LEN + 1] = "none";

static uint8 core_link_user_queue_next_index (uint8 index)
{
    index ++;
    if(index >= CORE_LINK_USER_KEY_QUEUE_SIZE)
    {
        index = 0;
    }

    return index;
}

static void core_link_user_push_key_event (uint8 key)
{
    uint8 next_tail = 0;

    if(0 == key)
    {
        return;
    }

    next_tail = core_link_user_queue_next_index(s_core_link_user_key_queue_tail);
    if(next_tail == s_core_link_user_key_queue_head)
    {
        return;
    }

    s_core_link_user_key_queue[s_core_link_user_key_queue_tail] = key;
    s_core_link_user_key_queue_tail = next_tail;
}

static uint8 core_link_user_pop_key_event (uint8 *key)
{
    if((NULL == key) || (s_core_link_user_key_queue_head == s_core_link_user_key_queue_tail))
    {
        return 0;
    }

    *key = s_core_link_user_key_queue[s_core_link_user_key_queue_head];
    s_core_link_user_key_queue_head = core_link_user_queue_next_index(s_core_link_user_key_queue_head);

    return 1;
}

static void xy_link_init (void)
{
    uart_init(XY_LINK_UART_INDEX,
              XY_LINK_UART_BAUDRATE,
              XY_LINK_UART_TX_PIN,
              XY_LINK_UART_RX_PIN);
    /* 不调用 uart_rx_interrupt：本通道只发不收，避免和空 uart3_isr 串扰。 */
}

static void xy_link_send_axis (uint8 axis_item, int value)
{
    /* 用 core_link 自身的 ASCII 参数帧（"A1=value" / "A2=value"）转发，跟车端
     * 推给飞控的格式完全一致；接收侧 'A''1' 写 adc_x，'A''2' 写 adc_y。
     * 一次只发一轴，跟车端的发送节奏对齐，避免双倍带宽。 */
    core_link_send_param_ascii_via(XY_LINK_UART_INDEX, 'A', axis_item, (float)value);
}

void core_link_user_init (void)
{
    s_core_link_user_key_queue_head = 0;
    s_core_link_user_key_queue_tail = 0;
    s_core_link_user_monitor_exit_request = 0;
    memset((void *)s_core_link_user_key_queue, 0, sizeof(s_core_link_user_key_queue));
    memset(s_core_link_user_text_receive, 0, sizeof(s_core_link_user_text_receive));

    xy_link_init();
}

void core_link_user_task (void)
{
    uint8 key = 0;

    if(0 != key_flag)
    {
        return;
    }

    if(core_link_user_pop_key_event(&key))
    {
        key_flag = key;
    }
}

void core_link_user_poll (void)
{
    core_link_task();
    core_link_user_task();
}

uint8 core_link_user_consume_monitor_exit_request (void)
{
    if(!s_core_link_user_monitor_exit_request)
    {
        return 0;
    }

    s_core_link_user_monitor_exit_request = 0;
    return 1;
}

void core_link_user_send_drone_status_from_snapshot (const MonitorRuntimeSnapshot *snapshot)
{
    core_link_drone_status_packet_struct status = {0};

    if(NULL == snapshot)
    {
        return;
    }

    status.armed = snapshot->armed;
    status.control_mode = snapshot->control_mode;
    status.low_voltage = snapshot->low_voltage;
    status.beacon_count = (uint8)((snapshot->beacon_count < 0) ? 0 : snapshot->beacon_count);
    // status.voltage_mv = (uint16)(snapshot->voltage * 1000.0f);
    status.voltage_mv = (uint16)(snapshot->voltage * 1000.0f);
    status.target_throttle = (int16)snapshot->target_throttle;
    status.base_throttle = (int16)snapshot->base_throttle;
    status.roll_ddeg = (int16)(snapshot->roll * 10.0f);
    status.pitch_ddeg = (int16)(snapshot->pitch * 10.0f);
    status.yaw_ddeg = (int16)(snapshot->yaw * 10.0f);
    status.tof_height_dcm = (int16)(snapshot->tof_height_cm * 10.0f);
    status.fused_height_dcm = (int16)(snapshot->fused_height_cm * 10.0f);

    core_link_send_drone_status(&status);
}

void core_link_on_text_received (const uint8 *text, uint8 length)
{
    uint8 copy_length = 0;

    if(NULL == text)
    {
        return;
    }

    copy_length = (length < CORE_LINK_PAYLOAD_MAX_LEN) ? length : CORE_LINK_PAYLOAD_MAX_LEN;
    memcpy(s_core_link_user_text_receive, text, copy_length);
    s_core_link_user_text_receive[copy_length] = 0;
}

void core_link_on_param_received (const core_link_param_packet_struct *param)
{
    int32 adc_value = 0;

    if(NULL == param)
    {
        return;
    }

    if('B' == param->command && 'V' == param->item)
    {
        /* 小车发来的电池电压，单位 V。小车与飞机共用同一块电池，直接用于监视页显示。 */
        if(param->value >= 9.0f && param->value <= 12.6f)
        {
            car_bat_voltage = param->value;
        }
        return;
    }

    if('A' == param->command)
    {
        adc_value = (int32)(param->value + ((param->value >= 0.0f) ? 0.5f : -0.5f));

        switch(param->item)
        {
            case '1':
                adc_x = (int)adc_value;
                xy_link_send_axis('1', adc_x);
                break;

            case '2':
                adc_y = (int)adc_value;
                xy_link_send_axis('2', adc_y);
                break;

            default:
                break;
        }

        return;
    }

    if(('K' != param->command) || (CORE_LINK_USER_KEY_ACTIVE_THRESHOLD > param->value))
    {
        return;
    }

    switch(param->item)
    {
        case '1':
            core_link_user_push_key_event(Button_1);
            break;

        case '2':
            core_link_user_push_key_event(Button_2);
            break;

        case '3':
            core_link_user_push_key_event(Button_3);
            break;

        case '4':
            core_link_user_push_key_event(Button_4);
            break;

        case 'M':
            if(menu_is_monitor_view())
            {
                s_core_link_user_monitor_exit_request = 1;
            }
            else
            {
                core_link_user_push_key_event(Button_4);
            }
            break;

        default:
            break;
    }
}
