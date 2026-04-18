#include "common/project_headfile.h"

#define CORE_LINK_USER_KEY_QUEUE_SIZE       (8U)
#define CORE_LINK_USER_KEY_ACTIVE_THRESHOLD (0.5f)

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

void core_link_user_init (void)
{
    s_core_link_user_key_queue_head = 0;
    s_core_link_user_key_queue_tail = 0;
    s_core_link_user_monitor_exit_request = 0;
    memset((void *)s_core_link_user_key_queue, 0, sizeof(s_core_link_user_key_queue));
    memset(s_core_link_user_text_receive, 0, sizeof(s_core_link_user_text_receive));
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

    if('A' == param->command)
    {
        adc_value = (int32)(param->value + ((param->value >= 0.0f) ? 0.5f : -0.5f));

        switch(param->item)
        {
            case '1':
                adc_x = (int)adc_value;
                break;

            case '2':
                adc_y = (int)adc_value;
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
