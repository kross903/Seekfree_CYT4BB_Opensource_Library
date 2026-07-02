#include "common/project_headfile.h"
#include <math.h>

extern volatile float car_bat_voltage;

#define CORE_LINK_USER_KEY_QUEUE_SIZE       (8U)
#define CORE_LINK_USER_KEY_ACTIVE_THRESHOLD (0.5f)
#define CORE_LINK_USER_SCREEN_SWITCH_THRESHOLD (0.5f)
#define CORE_LINK_USER_BEACON_MAP_PERIOD_MS (50U)
#define CORE_LINK_USER_BEACON_TIMEOUT_MS    (200U)
#define CORE_LINK_USER_BEACON_ID_MATCH_THRESHOLD_CM (80.0f)

static volatile uint8 s_core_link_user_key_queue[CORE_LINK_USER_KEY_QUEUE_SIZE] = {0};
static volatile uint8 s_core_link_user_key_queue_head = 0;
static volatile uint8 s_core_link_user_key_queue_tail = 0;
static volatile uint8 s_core_link_user_monitor_exit_request = 0;
static volatile uint8 s_core_link_user_screen_mirror_enabled = 0;
static volatile uint8 s_core_link_user_screen_mirror_enabled_changed = 0;
static volatile uint8 s_core_link_user_key_flag_remote = 0;
static char s_core_link_user_text_receive[CORE_LINK_PAYLOAD_MAX_LEN + 1] = "none";
static uint16 s_core_link_user_beacon_tx_elapsed_ms = 0U;
static uint16 s_core_link_user_beacon_uplink_age_ms = 60000U;
static uint8 s_core_link_user_beacon_uplink_valid = 0U;
static uint8 s_core_link_user_car_yaw_valid = 0U;
static float s_core_link_user_car_yaw_deg = 0.0f;
static core_link_beacon_map_uplink_packet_struct s_core_link_user_beacon_uplink = {0};

static uint8 core_link_user_queue_next_index (uint8 index)
{
    index ++;
    if(index >= CORE_LINK_USER_KEY_QUEUE_SIZE)
    {
        index = 0;
    }

    return index;
}

static int16 core_link_user_read_i16_le(const uint8 *data)
{
    return (int16)((uint16)data[0] | ((uint16)data[1] << 8));
}

static void core_link_user_write_i16_le(uint8 *data, int16 value)
{
    if(NULL == data)
    {
        return;
    }

    data[0] = (uint8)((uint16)value & 0xFFU);
    data[1] = (uint8)(((uint16)value >> 8) & 0xFFU);
}

static float core_link_user_wrap_angle(float angle_deg)
{
    while(angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float core_link_user_angle_distance(float a_deg, float b_deg)
{
    return fabsf(core_link_user_wrap_angle(a_deg - b_deg));
}

static float core_link_user_distance_squared_cm(float x1_cm, float y1_cm, float x2_cm, float y2_cm)
{
    float dx = x1_cm - x2_cm;
    float dy = y1_cm - y2_cm;

    return dx * dx + dy * dy;
}

static int16 core_link_user_cm_to_mm_i16(float cm)
{
    float mm = cm * 10.0f;

    if(mm > 32767.0f)
    {
        mm = 32767.0f;
    }
    else if(mm < -32768.0f)
    {
        mm = -32768.0f;
    }

    return (int16)((mm >= 0.0f) ? (mm + 0.5f) : (mm - 0.5f));
}

static int16 core_link_user_deg_to_ddeg_i16(float deg)
{
    float ddeg = core_link_user_wrap_angle(deg) * 10.0f;

    if(ddeg > 32767.0f)
    {
        ddeg = 32767.0f;
    }
    else if(ddeg < -32768.0f)
    {
        ddeg = -32768.0f;
    }

    return (int16)((ddeg >= 0.0f) ? (ddeg + 0.5f) : (ddeg - 0.5f));
}

static uint8 core_link_user_get_uplink_car_yaw_deg(float *yaw_deg)
{
    if((NULL == yaw_deg)
       || !s_core_link_user_beacon_uplink_valid
       || (0U == s_core_link_user_beacon_uplink.car_visible))
    {
        return 0U;
    }

    *yaw_deg = core_link_user_wrap_angle(((float)s_core_link_user_beacon_uplink.car_yaw_ddeg) * 0.1f);
    return 1U;
}

static uint8 core_link_user_estimate_car_yaw_deg(float *yaw_deg)
{
    float center_x = 0.0f;
    float center_y = 0.0f;
    float cov_xx = 0.0f;
    float cov_xy = 0.0f;
    float cov_yy = 0.0f;
    float heading_deg;
    float alt_heading_deg;
    float reference_yaw_deg = 0.0f;
    uint8 reference_yaw_valid = 0U;
    uint8 i;

    if(NULL == yaw_deg || !g_vehicle_marker_pose.valid || (g_vehicle_marker_pose.detected_point_count < 3U))
    {
        return 0U;
    }

    for(i = 0U; i < IR_SQUARE_POINT_COUNT; ++i)
    {
        center_x += g_vehicle_marker_pose.world_pts[i].x;
        center_y += g_vehicle_marker_pose.world_pts[i].y;
    }

    center_x *= 0.25f;
    center_y *= 0.25f;

    for(i = 0U; i < IR_SQUARE_POINT_COUNT; ++i)
    {
        float dx = g_vehicle_marker_pose.world_pts[i].x - center_x;
        float dy = g_vehicle_marker_pose.world_pts[i].y - center_y;

        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }

    heading_deg = atan2f(2.0f * cov_xy, cov_xx - cov_yy) * 0.5f * RAD2DEG;
    heading_deg = core_link_user_wrap_angle(heading_deg);
    alt_heading_deg = core_link_user_wrap_angle(heading_deg + 180.0f);

    reference_yaw_valid = core_link_user_get_uplink_car_yaw_deg(&reference_yaw_deg);
    if(!reference_yaw_valid && s_core_link_user_car_yaw_valid)
    {
        reference_yaw_deg = s_core_link_user_car_yaw_deg;
        reference_yaw_valid = 1U;
    }

    if(reference_yaw_valid)
    {
        if(core_link_user_angle_distance(alt_heading_deg, reference_yaw_deg)
           < core_link_user_angle_distance(heading_deg, reference_yaw_deg))
        {
            heading_deg = alt_heading_deg;
        }
    }

    s_core_link_user_car_yaw_deg = heading_deg;
    s_core_link_user_car_yaw_valid = 1U;
    *yaw_deg = heading_deg;
    return 1U;
}

static void core_link_user_send_beacon_map_downlink(const core_link_beacon_map_packet_struct *packet)
{
    uint8 payload[CORE_LINK_BEACON_MAP_PAYLOAD_LEN] = {0};
    uint8 i;

    if(NULL == packet)
    {
        return;
    }

    payload[0] = packet->car_visible;
    core_link_user_write_i16_le(&payload[1], packet->car_x_mm);
    core_link_user_write_i16_le(&payload[3], packet->car_y_mm);
    core_link_user_write_i16_le(&payload[5], packet->car_yaw_ddeg);

    for(i = 0U; i < CORE_LINK_BEACON_MAP_COUNT; ++i)
    {
        uint8 offset = (uint8)(7U + i * 6U);

        payload[offset] = packet->beacon[i].id;
        core_link_user_write_i16_le(&payload[offset + 1U], packet->beacon[i].x_mm);
        core_link_user_write_i16_le(&payload[offset + 3U], packet->beacon[i].y_mm);
        payload[offset + 5U] = packet->beacon[i].state;
    }

    core_link_send_frame(CORE_LINK_PROTOCOL_BEACON_MAP, payload, CORE_LINK_BEACON_MAP_PAYLOAD_LEN);
}

static void core_link_user_build_beacon_map_downlink(core_link_beacon_map_packet_struct *packet)
{
    VisionRuntimeDiag_t diag = {0};
    VisionRuntimeBeaconSlot_t slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    uint8 uplink_slot_used[CORE_LINK_BEACON_MAP_COUNT] = {0};
    float car_yaw_deg = 0.0f;
    uint8 car_visible = 0U;
    uint8 yaw_valid = 0U;
    uint8 i;

    if(NULL == packet)
    {
        return;
    }

    memset(packet, 0, sizeof(*packet));
    VisionRuntime_GetDiag(&diag);
    VisionRuntime_GetBeaconSlots(slots, VISION_RUNTIME_BEACON_SLOT_COUNT);

    car_visible = (uint8)((g_vehicle_marker_pose.valid && !g_vehicle_marker_position_lost_flag) ? 1U : 0U);
    yaw_valid = core_link_user_estimate_car_yaw_deg(&car_yaw_deg);
    if(car_visible && yaw_valid)
    {
        packet->car_visible = 1U;
        packet->car_x_mm = core_link_user_cm_to_mm_i16(g_vehicle_marker_pose.x);
        /* UAV side has migrated to X-forward / Y-left semantics.
         * The packet layout is unchanged; downstream consumers must not assume
         * legacy "Y-right positive" without an explicit compatibility layer. */
        packet->car_y_mm = core_link_user_cm_to_mm_i16(g_vehicle_marker_pose.y);
        packet->car_yaw_ddeg = core_link_user_deg_to_ddeg_i16(car_yaw_deg);
    }

    for(i = 0U; i < CORE_LINK_BEACON_MAP_COUNT; ++i)
    {
        if(slots[i].valid)
        {
            uint8 best_uplink_slot = 0xFFU;
            float best_distance_sq = CORE_LINK_USER_BEACON_ID_MATCH_THRESHOLD_CM
                                   * CORE_LINK_USER_BEACON_ID_MATCH_THRESHOLD_CM;
            uint8 j;

            if(s_core_link_user_beacon_uplink_valid)
            {
                for(j = 0U; j < CORE_LINK_BEACON_MAP_COUNT; ++j)
                {
                    float uplink_x_cm;
                    float uplink_y_cm;
                    float distance_sq;

                    if(uplink_slot_used[j]
                       || (0U == s_core_link_user_beacon_uplink.beacon[j].state)
                       || (0U == s_core_link_user_beacon_uplink.beacon[j].id)
                       || (s_core_link_user_beacon_uplink.beacon[j].id > 5U))
                    {
                        continue;
                    }

                    uplink_x_cm = ((float)s_core_link_user_beacon_uplink.beacon[j].x_mm) * 0.1f;
                    uplink_y_cm = ((float)s_core_link_user_beacon_uplink.beacon[j].y_mm) * 0.1f;
                    distance_sq = core_link_user_distance_squared_cm(slots[i].x_cm,
                                                                     slots[i].y_cm,
                                                                     uplink_x_cm,
                                                                     uplink_y_cm);
                    if(distance_sq <= best_distance_sq)
                    {
                        best_distance_sq = distance_sq;
                        best_uplink_slot = j;
                    }
                }
            }

            if(best_uplink_slot < CORE_LINK_BEACON_MAP_COUNT)
            {
                packet->beacon[i].id = s_core_link_user_beacon_uplink.beacon[best_uplink_slot].id;
                uplink_slot_used[best_uplink_slot] = 1U;
            }
            else
            {
                packet->beacon[i].id = 0U;
            }

            packet->beacon[i].x_mm = core_link_user_cm_to_mm_i16(slots[i].x_cm);
            packet->beacon[i].y_mm = core_link_user_cm_to_mm_i16(slots[i].y_cm);
            packet->beacon[i].state = 1U;
        }
    }

    (void)diag;
}

static void core_link_user_send_beacon_map_downlink_if_due(void)
{
    core_link_beacon_map_packet_struct packet;

    if(s_core_link_user_beacon_tx_elapsed_ms < CORE_LINK_USER_BEACON_MAP_PERIOD_MS)
    {
        return;
    }

    s_core_link_user_beacon_tx_elapsed_ms -= CORE_LINK_USER_BEACON_MAP_PERIOD_MS;
    core_link_user_build_beacon_map_downlink(&packet);
    core_link_user_send_beacon_map_downlink(&packet);
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

static void core_link_user_clear_remote_events(void)
{
    s_core_link_user_key_queue_head = 0;
    s_core_link_user_key_queue_tail = 0;
    s_core_link_user_monitor_exit_request = 0;
    memset((void *)s_core_link_user_key_queue, 0, sizeof(s_core_link_user_key_queue));
}

void core_link_user_init (void)
{
    core_link_user_clear_remote_events();
    s_core_link_user_key_flag_remote = 0;
    s_core_link_user_screen_mirror_enabled = 0;
    s_core_link_user_screen_mirror_enabled_changed = 0;
    s_core_link_user_beacon_tx_elapsed_ms = 0U;
    s_core_link_user_beacon_uplink_age_ms = 60000U;
    s_core_link_user_beacon_uplink_valid = 0U;
    s_core_link_user_car_yaw_valid = 0U;
    memset((void *)&s_core_link_user_beacon_uplink, 0, sizeof(s_core_link_user_beacon_uplink));
    memset(s_core_link_user_text_receive, 0, sizeof(s_core_link_user_text_receive));
}

void core_link_user_task (void)
{
    uint8 key = 0;

    if(0 != key_flag)
    {
        return;
    }

    s_core_link_user_key_flag_remote = 0;
    if(core_link_user_pop_key_event(&key))
    {
        key_flag = key;
        s_core_link_user_key_flag_remote = 1;
    }
}

void core_link_user_poll (void)
{
    core_link_task();
    core_link_user_task();
}

void core_link_user_tick (uint16 elapsed_ms)
{
    if(s_core_link_user_beacon_tx_elapsed_ms <= (uint16)(60000U - elapsed_ms))
    {
        s_core_link_user_beacon_tx_elapsed_ms += elapsed_ms;
    }
    else
    {
        s_core_link_user_beacon_tx_elapsed_ms = 60000U;
    }

    if(s_core_link_user_beacon_uplink_age_ms <= (uint16)(60000U - elapsed_ms))
    {
        s_core_link_user_beacon_uplink_age_ms += elapsed_ms;
    }
    else
    {
        s_core_link_user_beacon_uplink_age_ms = 60000U;
    }

    if(s_core_link_user_beacon_uplink_age_ms > CORE_LINK_USER_BEACON_TIMEOUT_MS)
    {
        s_core_link_user_beacon_uplink_valid = 0U;
    }
}

void core_link_user_service_beacon_link(void)
{
    core_link_user_send_beacon_map_downlink_if_due();
}

uint8 core_link_user_is_screen_mirror_enabled (void)
{
    return s_core_link_user_screen_mirror_enabled ? 1U : 0U;
}

uint8 core_link_user_consume_screen_mirror_enabled_changed (void)
{
    if(!s_core_link_user_screen_mirror_enabled_changed)
    {
        return 0U;
    }

    s_core_link_user_screen_mirror_enabled_changed = 0;
    return 1U;
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

void ips200_mirror_send_packet (const uint8 *payload, uint8 length)
{
    if(!s_core_link_user_screen_mirror_enabled)
    {
        return;
    }

    core_link_send_frame(CORE_LINK_PROTOCOL_SCREEN_CMD, payload, length);
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
    uint8 next_screen_mirror_enabled = 0;

    if(NULL == param)
    {
        return;
    }

    if(('S' == param->command) && ('1' == param->item))
    {
        next_screen_mirror_enabled = (param->value >= CORE_LINK_USER_SCREEN_SWITCH_THRESHOLD) ? 1U : 0U;
        if(s_core_link_user_screen_mirror_enabled != next_screen_mirror_enabled)
        {
            s_core_link_user_screen_mirror_enabled = next_screen_mirror_enabled;
            s_core_link_user_screen_mirror_enabled_changed = 1U;
        }
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

void core_link_on_frame_received (uint8 protocol_id, const uint8 *payload, uint8 length)
{
    uint8 i;

    if((CORE_LINK_PROTOCOL_BEACON_MAP_UPLINK == protocol_id)
       && (CORE_LINK_BEACON_MAP_UPLINK_PAYLOAD_LEN == length)
       && (NULL != payload))
    {
        memset((void *)&s_core_link_user_beacon_uplink, 0, sizeof(s_core_link_user_beacon_uplink));
        s_core_link_user_beacon_uplink.car_visible = payload[0];
        s_core_link_user_beacon_uplink.car_x_mm = core_link_user_read_i16_le(&payload[1]);
        s_core_link_user_beacon_uplink.car_y_mm = core_link_user_read_i16_le(&payload[3]);
        s_core_link_user_beacon_uplink.car_yaw_ddeg = core_link_user_read_i16_le(&payload[5]);

        for(i = 0U; i < CORE_LINK_BEACON_MAP_COUNT; ++i)
        {
            uint8 offset = (uint8)(7U + i * 6U);

            s_core_link_user_beacon_uplink.beacon[i].id = payload[offset];
            s_core_link_user_beacon_uplink.beacon[i].x_mm = core_link_user_read_i16_le(&payload[offset + 1U]);
            s_core_link_user_beacon_uplink.beacon[i].y_mm = core_link_user_read_i16_le(&payload[offset + 3U]);
            s_core_link_user_beacon_uplink.beacon[i].state = payload[offset + 5U];
        }

        s_core_link_user_beacon_uplink_valid = 1U;
        s_core_link_user_beacon_uplink_age_ms = 0U;
    }
}
