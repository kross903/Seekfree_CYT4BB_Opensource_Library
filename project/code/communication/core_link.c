#include "common/project_headfile.h"

typedef enum
{
    CORE_LINK_PARSE_WAIT_SOF1 = 0,
    CORE_LINK_PARSE_WAIT_SOF2,
    CORE_LINK_PARSE_WAIT_PROTOCOL,
    CORE_LINK_PARSE_WAIT_LENGTH,
    CORE_LINK_PARSE_WAIT_PAYLOAD,
    CORE_LINK_PARSE_WAIT_CHECKSUM,
}core_link_parse_state_enum;

static uint8                       core_link_uart_rx_byte = 0;
static uint8                       core_link_rx_fifo_buffer[CORE_LINK_RX_FIFO_SIZE];
static fifo_struct                 core_link_rx_fifo;

static core_link_parse_state_enum  core_link_parse_state = CORE_LINK_PARSE_WAIT_SOF1;
static uint8                       core_link_protocol_id = 0;
static uint8                       core_link_payload_length = 0;
static uint8                       core_link_payload_index = 0;
static uint8                       core_link_checksum = 0;
static uint8                       core_link_payload[CORE_LINK_PAYLOAD_MAX_LEN] = {0};

static void core_link_write_i16_le(uint8 *data, int16 value)
{
    if(NULL == data)
    {
        return;
    }

    data[0] = (uint8)((uint16)value & 0xFFU);
    data[1] = (uint8)(((uint16)value >> 8) & 0xFFU);
}

static uint8 core_link_calc_checksum (uint8 protocol_id, uint8 length, const uint8 *payload)
{
    uint8 checksum = protocol_id ^ length;
    uint8 i = 0;

    for(i = 0; i < length; i ++)
    {
        checksum ^= payload[i];
    }

    return checksum;
}

static uint8 core_link_parse_float (const uint8 *data, uint8 length, float *value)
{
    uint8 index = 0;
    uint8 has_digit = 0;
    uint8 has_point = 0;
    uint8 minus_flag = 0;
    float integer_part = 0.0f;
    float fractional_part = 0.0f;
    float fractional_scale = 0.1f;

    if((NULL == data) || (NULL == value) || (0 == length))
    {
        return 0;
    }

    if('-' == data[index])
    {
        minus_flag = 1;
        index ++;
    }
    else if('+' == data[index])
    {
        index ++;
    }

    for(; index < length; index ++)
    {
        uint8 ch = data[index];

        if('.' == ch)
        {
            if(has_point)
            {
                return 0;
            }
            has_point = 1;
            continue;
        }

        if((ch < '0') || (ch > '9'))
        {
            return 0;
        }

        has_digit = 1;
        if(!has_point)
        {
            integer_part = integer_part * 10.0f + (float)(ch - '0');
        }
        else
        {
            fractional_part += (float)(ch - '0') * fractional_scale;
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

static uint8 core_link_parse_param_ascii (const uint8 *payload, uint8 length, core_link_param_packet_struct *param)
{
    if((NULL == payload) || (NULL == param))
    {
        return 0;
    }

    if((length < 4) || ('=' != payload[2]))
    {
        return 0;
    }

    param->command = payload[0];
    param->item = payload[1];

    return core_link_parse_float(&payload[3], (uint8)(length - 3), &param->value);
}

static void core_link_reset_parser (void)
{
    core_link_parse_state = CORE_LINK_PARSE_WAIT_SOF1;
    core_link_protocol_id = 0;
    core_link_payload_length = 0;
    core_link_payload_index = 0;
    core_link_checksum = 0;
    memset(core_link_payload, 0, sizeof(core_link_payload));
}

static void core_link_dispatch_frame (uint8 protocol_id, const uint8 *payload, uint8 length)
{
    core_link_param_packet_struct param = {0};

    core_link_on_frame_received(protocol_id, payload, length);

    switch(protocol_id)
    {
        case CORE_LINK_PROTOCOL_TEXT:
        {
            core_link_on_text_received(payload, length);
        }break;

        case CORE_LINK_PROTOCOL_PARAM_ASCII:
        {
            if(core_link_parse_param_ascii(payload, length, &param))
            {
                core_link_on_param_received(&param);
            }
        }break;

        default:
        {
        }break;
    }
}

static void core_link_parse_byte (uint8 byte)
{
    switch(core_link_parse_state)
    {
        case CORE_LINK_PARSE_WAIT_SOF1:
        {
            if(CORE_LINK_SOF1 == byte)
            {
                core_link_parse_state = CORE_LINK_PARSE_WAIT_SOF2;
            }
        }break;

        case CORE_LINK_PARSE_WAIT_SOF2:
        {
            if(CORE_LINK_SOF2 == byte)
            {
                core_link_parse_state = CORE_LINK_PARSE_WAIT_PROTOCOL;
            }
            else if(CORE_LINK_SOF1 != byte)
            {
                core_link_reset_parser();
            }
        }break;

        case CORE_LINK_PARSE_WAIT_PROTOCOL:
        {
            core_link_protocol_id = byte;
            core_link_parse_state = CORE_LINK_PARSE_WAIT_LENGTH;
        }break;

        case CORE_LINK_PARSE_WAIT_LENGTH:
        {
            core_link_payload_length = byte;
            core_link_payload_index = 0;

            if(core_link_payload_length > CORE_LINK_PAYLOAD_MAX_LEN)
            {
                core_link_reset_parser();
            }
            else if(0 == core_link_payload_length)
            {
                core_link_parse_state = CORE_LINK_PARSE_WAIT_CHECKSUM;
            }
            else
            {
                core_link_parse_state = CORE_LINK_PARSE_WAIT_PAYLOAD;
            }
        }break;

        case CORE_LINK_PARSE_WAIT_PAYLOAD:
        {
            core_link_payload[core_link_payload_index ++] = byte;

            if(core_link_payload_index >= core_link_payload_length)
            {
                core_link_parse_state = CORE_LINK_PARSE_WAIT_CHECKSUM;
            }
        }break;

        case CORE_LINK_PARSE_WAIT_CHECKSUM:
        {
            core_link_checksum = core_link_calc_checksum(core_link_protocol_id,
                                                         core_link_payload_length,
                                                         core_link_payload);
            if(core_link_checksum == byte)
            {
                core_link_dispatch_frame(core_link_protocol_id,
                                         core_link_payload,
                                         core_link_payload_length);
            }
            core_link_reset_parser();
        }break;

        default:
        {
            core_link_reset_parser();
        }break;
    }
}

void core_link_init (void)
{
    fifo_init(&core_link_rx_fifo, FIFO_DATA_8BIT, core_link_rx_fifo_buffer, CORE_LINK_RX_FIFO_SIZE);

    uart_init(CORE_LINK_UART_INDEX,
              CORE_LINK_UART_BAUDRATE,
              CORE_LINK_UART_TX_PIN,
              CORE_LINK_UART_RX_PIN);
    uart_rx_interrupt(CORE_LINK_UART_INDEX, 1);

    core_link_reset_parser();
}

void core_link_task (void)
{
    uint8 rx_buffer[32];
    uint32 fifo_data_count = 0;
    uint32 read_count = 0;
    uint32 index = 0;

    fifo_data_count = fifo_used(&core_link_rx_fifo);
    if(0 == fifo_data_count)
    {
        return;
    }

    while(fifo_data_count)
    {
        read_count = (fifo_data_count > sizeof(rx_buffer)) ? sizeof(rx_buffer) : fifo_data_count;
        fifo_read_buffer(&core_link_rx_fifo, rx_buffer, &read_count, FIFO_READ_AND_CLEAN);

        for(index = 0; index < read_count; index ++)
        {
            core_link_parse_byte(rx_buffer[index]);
        }

        fifo_data_count = fifo_used(&core_link_rx_fifo);
    }
}

void core_link_uart4_rx_handler (void)
{
    while(uart_query_byte(CORE_LINK_UART_INDEX, &core_link_uart_rx_byte))
    {
        fifo_write_buffer(&core_link_rx_fifo, &core_link_uart_rx_byte, 1);
    }
}

void core_link_send_frame (uint8 protocol_id, const uint8 *payload, uint8 length)
{
    uint8 checksum = 0;

    if(length > CORE_LINK_PAYLOAD_MAX_LEN)
    {
        return;
    }

    if((length > 0) && (NULL == payload))
    {
        return;
    }

    checksum = core_link_calc_checksum(protocol_id, length, payload);

    uart_write_byte(CORE_LINK_UART_INDEX, CORE_LINK_SOF1);
    uart_write_byte(CORE_LINK_UART_INDEX, CORE_LINK_SOF2);
    uart_write_byte(CORE_LINK_UART_INDEX, protocol_id);
    uart_write_byte(CORE_LINK_UART_INDEX, length);

    if(length)
    {
        uart_write_buffer(CORE_LINK_UART_INDEX, payload, length);
    }

    uart_write_byte(CORE_LINK_UART_INDEX, checksum);
}

void core_link_send_text (const char *text)
{
    uint32 text_length = 0;

    if(NULL == text)
    {
        return;
    }

    text_length = strlen(text);
    if(text_length > CORE_LINK_PAYLOAD_MAX_LEN)
    {
        text_length = CORE_LINK_PAYLOAD_MAX_LEN;
    }

    core_link_send_frame(CORE_LINK_PROTOCOL_TEXT, (const uint8 *)text, (uint8)text_length);
}

void core_link_send_param_ascii (uint8 command, uint8 item, float value)
{
    char payload[CORE_LINK_PAYLOAD_MAX_LEN];

    sprintf(payload, "%c%c=%.*f", command, item, CORE_LINK_PARAM_DECIMAL_NUM, value);
    core_link_send_frame(CORE_LINK_PROTOCOL_PARAM_ASCII, (const uint8 *)payload, (uint8)strlen(payload));
}

void core_link_send_drone_status (const core_link_drone_status_packet_struct *status)
{
    if(NULL == status)
    {
        return;
    }

    core_link_send_frame(CORE_LINK_PROTOCOL_DRONE_STATUS,
                         (const uint8 *)status,
                         (uint8)sizeof(core_link_drone_status_packet_struct));
}

void core_link_send_beacon_map_uplink(const core_link_beacon_map_uplink_packet_struct *packet)
{
    uint8 payload[CORE_LINK_BEACON_MAP_UPLINK_PAYLOAD_LEN] = {0};
    uint8 i;

    if(NULL == packet)
    {
        return;
    }

    payload[0] = packet->car_visible;
    core_link_write_i16_le(&payload[1], packet->car_x_mm);
    core_link_write_i16_le(&payload[3], packet->car_y_mm);
    core_link_write_i16_le(&payload[5], packet->car_absolute_yaw_ddeg);

    for(i = 0U; i < CORE_LINK_BEACON_MAP_COUNT; ++i)
    {
        uint8 offset = (uint8)(7U + i * 6U);

        payload[offset] = packet->beacon[i].id;
        core_link_write_i16_le(&payload[offset + 1U], packet->beacon[i].x_mm);
        core_link_write_i16_le(&payload[offset + 3U], packet->beacon[i].y_mm);
        payload[offset + 5U] = packet->beacon[i].state;
    }

    payload[37] = packet->motion_flags;
    payload[38] = packet->motion_sequence;
    core_link_write_i16_le(&payload[39], packet->car_forward_speed_dcms);
    core_link_write_i16_le(&payload[41], packet->car_left_speed_dcms);

    core_link_send_frame(CORE_LINK_PROTOCOL_BEACON_MAP_UPLINK,
                         payload,
                         CORE_LINK_BEACON_MAP_UPLINK_PAYLOAD_LEN);
}

__weak void core_link_on_frame_received (uint8 protocol_id, const uint8 *payload, uint8 length)
{
    (void)protocol_id;
    (void)payload;
    (void)length;
}

__weak void core_link_on_text_received (const uint8 *text, uint8 length)
{
    (void)text;
    (void)length;
}

__weak void core_link_on_param_received (const core_link_param_packet_struct *param)
{
    (void)param;
}
