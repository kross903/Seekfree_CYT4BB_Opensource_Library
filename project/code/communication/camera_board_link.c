#include "common/project_headfile.h"

#define CAMERA_BOARD_RX_FIFO_SIZE       (512U)
#define CAMERA_BOARD_DWT_LAR_UNLOCK     (0xC5ACCE55UL)
#define CAMERA_BOARD_STATE_HISTORY_SIZE (32U)

typedef struct
{
    uint16 sequence;
    uint16 flags;
    uint32 master_time_us;
    float yaw_deg;
} CameraBoardStateHistoryEntry_t;

typedef enum
{
    CAMERA_BOARD_PARSE_SOF1 = 0,
    CAMERA_BOARD_PARSE_SOF2,
    CAMERA_BOARD_PARSE_VERSION,
    CAMERA_BOARD_PARSE_TYPE,
    CAMERA_BOARD_PARSE_LENGTH_LOW,
    CAMERA_BOARD_PARSE_LENGTH_HIGH,
    CAMERA_BOARD_PARSE_SEQUENCE_LOW,
    CAMERA_BOARD_PARSE_SEQUENCE_HIGH,
    CAMERA_BOARD_PARSE_PAYLOAD,
    CAMERA_BOARD_PARSE_CRC_LOW,
    CAMERA_BOARD_PARSE_CRC_HIGH,
} CameraBoardParseState_t;

static volatile uint8 s_rx_fifo[CAMERA_BOARD_RX_FIFO_SIZE];
static volatile uint32 s_rx_cycle[CAMERA_BOARD_RX_FIFO_SIZE];
static volatile uint16 s_rx_head = 0U;
static volatile uint16 s_rx_tail = 0U;

static CameraBoardParseState_t s_parse_state = CAMERA_BOARD_PARSE_SOF1;
static uint8 s_parse_type = 0U;
static uint16 s_parse_length = 0U;
static uint16 s_parse_sequence = 0U;
static uint16 s_parse_index = 0U;
static uint16 s_parse_crc_received = 0U;
static uint32 s_parse_last_byte_cycle = 0U;
static uint8 s_parse_crc_data[6U + CAMERA_BOARD_PROTOCOL_MAX_PAYLOAD];
static uint8 s_parse_payload[CAMERA_BOARD_PROTOCOL_MAX_PAYLOAD];

static uint8 s_tx_buffer[CAMERA_BOARD_PROTOCOL_MAX_FRAME];
static uint16 s_tx_length = 0U;
static uint16 s_tx_offset = 0U;
static uint16 s_tx_sequence = 0U;

static uint8 s_time_ready = 0U;
static uint32 s_time_last_cycle = 0U;
static uint32 s_time_cycle_remainder = 0U;
static uint64 s_time_us = 0ULL;

static uint8 s_mode_active = 0U;
static uint16 s_state_sequence = 0U;
static uint16 s_reset_generation = 0U;
static uint32 s_last_state_tx_us = 0U;
static uint32 s_last_result_rx_us = 0U;
static uint32 s_last_status_rx_us = 0U;
static uint16 s_last_rx_sequence = 0U;
static uint8 s_last_rx_sequence_valid = 0U;
static CameraBoardVisionFrame_t s_latest_frame;
static uint8 s_latest_frame_valid = 0U;
static uint8 s_latest_frame_received = 0U;
static CameraBoardStateHistoryEntry_t
    s_state_history[CAMERA_BOARD_STATE_HISTORY_SIZE];
static uint8 s_state_history_head = 0U;
static uint8 s_state_history_count = 0U;
static CameraBoardLinkDiag_t s_diag;

static uint8 CameraBoardLink_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return 0U;
    }
    DWT->LAR = CAMERA_BOARD_DWT_LAR_UNLOCK;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return 1U;
}

uint32 CameraBoardLink_TimeNowUs(void)
{
    uint32 now_cycle;
    uint32 delta_cycle;
    uint32 cycles_per_us;
    uint64 accumulated_cycle;

    if(!CameraBoardLink_DwtInit() || (SystemCoreClock < 1000000U))
    {
        return (uint32)s_time_us;
    }

    now_cycle = DWT->CYCCNT;
    if(!s_time_ready)
    {
        s_time_last_cycle = now_cycle;
        s_time_ready = 1U;
        return (uint32)s_time_us;
    }

    delta_cycle = now_cycle - s_time_last_cycle;
    s_time_last_cycle = now_cycle;
    cycles_per_us = SystemCoreClock / 1000000U;
    accumulated_cycle = (uint64)s_time_cycle_remainder + (uint64)delta_cycle;
    s_time_us += accumulated_cycle / cycles_per_us;
    s_time_cycle_remainder = (uint32)(accumulated_cycle % cycles_per_us);
    return (uint32)s_time_us;
}

static uint16 CameraBoardLink_ReadU16(const uint8 *data)
{
    return (uint16)((uint16)data[0] | ((uint16)data[1] << 8));
}

static int16 CameraBoardLink_ReadI16(const uint8 *data)
{
    return (int16)CameraBoardLink_ReadU16(data);
}

static uint32 CameraBoardLink_ReadU32(const uint8 *data)
{
    return (uint32)data[0] |
           ((uint32)data[1] << 8) |
           ((uint32)data[2] << 16) |
           ((uint32)data[3] << 24);
}

static void CameraBoardLink_WriteU16(uint8 *data, uint16 value)
{
    data[0] = (uint8)(value & 0xFFU);
    data[1] = (uint8)((value >> 8) & 0xFFU);
}

static void CameraBoardLink_WriteI16(uint8 *data, int16 value)
{
    CameraBoardLink_WriteU16(data, (uint16)value);
}

static void CameraBoardLink_WriteU32(uint8 *data, uint32 value)
{
    data[0] = (uint8)(value & 0xFFU);
    data[1] = (uint8)((value >> 8) & 0xFFU);
    data[2] = (uint8)((value >> 16) & 0xFFU);
    data[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint16 CameraBoardLink_Crc16(const uint8 *data, uint16 length)
{
    uint16 crc = 0xFFFFU;
    uint16 index;
    uint8 bit;

    for(index = 0U; index < length; index++)
    {
        crc ^= (uint16)data[index] << 8;
        for(bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ?
                  (uint16)((crc << 1) ^ 0x1021U) :
                  (uint16)(crc << 1);
        }
    }
    return crc;
}

static int16 CameraBoardLink_FloatToI16(float value, float scale)
{
    float scaled = value * scale;
    if(!isfinite(scaled)) return 0;
    if(scaled > 32767.0f) return 32767;
    if(scaled < -32768.0f) return -32768;
    return (int16)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static uint16 CameraBoardLink_FloatToU16(float value, float scale)
{
    float scaled = value * scale;
    if(!isfinite(scaled) || (scaled <= 0.0f)) return 0U;
    if(scaled > 65535.0f) return 65535U;
    return (uint16)(scaled + 0.5f);
}

static void CameraBoardLink_ResetParser(void)
{
    s_parse_state = CAMERA_BOARD_PARSE_SOF1;
    s_parse_length = 0U;
    s_parse_index = 0U;
    s_parse_crc_received = 0U;
    s_parse_last_byte_cycle = 0U;
}

static uint8 CameraBoardLink_ParseGapTimedOut(uint32 byte_cycle)
{
    uint32 timeout_cycles;

    if((s_parse_state == CAMERA_BOARD_PARSE_SOF1) ||
       (SystemCoreClock == 0U))
    {
        return 0U;
    }
    timeout_cycles = (uint32)(((uint64)SystemCoreClock *
                               CAMERA_BOARD_PARSE_TIMEOUT_US) / 1000000ULL);
    return ((uint32)(byte_cycle - s_parse_last_byte_cycle) > timeout_cycles) ?
           1U : 0U;
}

static void CameraBoardLink_ResetSession(uint8 invalidate_frame)
{
    uint32 irq_state = __get_PRIMASK();
    __disable_irq();
    s_rx_head = 0U;
    s_rx_tail = 0U;
    if(irq_state == 0U) __enable_irq();

    CameraBoardLink_ResetParser();
    s_tx_length = 0U;
    s_tx_offset = 0U;
    s_last_rx_sequence_valid = 0U;
    s_last_result_rx_us = 0U;
    s_last_status_rx_us = 0U;
    s_diag.online = 0U;
    s_diag.rear_camera_ready = 0U;
    s_diag.rear_camera_core_alive = 0U;
    s_diag.rear_time_sync_valid = 0U;
    s_diag.rear_extrinsic_nominal = 0U;
    s_diag.rear_extrinsic_valid = 0U;
    s_diag.last_result_age_ms = 0xFFFFU;
    s_diag.last_status_age_ms = 0xFFFFU;
    s_diag.rear_state_age_ms = 0xFFFFU;
    if(invalidate_frame)
    {
        memset(&s_latest_frame, 0, sizeof(s_latest_frame));
        s_latest_frame_valid = 0U;
        s_latest_frame_received = 0U;
    }
}

static uint8 CameraBoardLink_StartFrame(uint8 type,
                                        const uint8 *payload,
                                        uint16 payload_length)
{
    volatile stc_SCB_t *uart = get_scb_module(CAMERA_BOARD_UART_INDEX);
    uint16 crc;

    if((payload_length > CAMERA_BOARD_PROTOCOL_MAX_PAYLOAD) ||
       ((payload_length > 0U) && (payload == NULL)) ||
       (s_tx_length != 0U) || !Cy_SCB_UART_IsTxComplete(uart))
    {
        return 0U;
    }

    s_tx_sequence++;
    s_tx_buffer[0] = CAMERA_BOARD_PROTOCOL_SOF1;
    s_tx_buffer[1] = CAMERA_BOARD_PROTOCOL_SOF2;
    s_tx_buffer[2] = CAMERA_BOARD_PROTOCOL_VERSION;
    s_tx_buffer[3] = type;
    CameraBoardLink_WriteU16(&s_tx_buffer[4], payload_length);
    CameraBoardLink_WriteU16(&s_tx_buffer[6], s_tx_sequence);
    if(payload_length > 0U)
    {
        memcpy(&s_tx_buffer[8], payload, payload_length);
    }
    crc = CameraBoardLink_Crc16(&s_tx_buffer[2], (uint16)(6U + payload_length));
    CameraBoardLink_WriteU16(&s_tx_buffer[8U + payload_length], crc);
    s_tx_length = (uint16)(payload_length + CAMERA_BOARD_PROTOCOL_FRAME_OVERHEAD);
    s_tx_offset = 0U;
    return 1U;
}

static void CameraBoardLink_ServiceTx(void)
{
    volatile stc_SCB_t *uart;
    uint32 copied;

    if(s_tx_length == 0U) return;
    uart = get_scb_module(CAMERA_BOARD_UART_INDEX);
    copied = Cy_SCB_UART_PutArray(uart,
                                  &s_tx_buffer[s_tx_offset],
                                  (uint32)(s_tx_length - s_tx_offset));
    s_tx_offset = (uint16)(s_tx_offset + (uint16)copied);
    if(s_tx_offset >= s_tx_length)
    {
        s_tx_length = 0U;
        s_tx_offset = 0U;
        s_diag.tx_frame_count++;
    }
}

static void CameraBoardLink_DecodeRearResult(const uint8 *payload, uint16 length)
{
    CameraBoardVisionFrame_t frame;
    uint8 count;
    uint8 index;

    if(length < CAMERA_BOARD_REAR_RESULT_HEADER_LEN)
    {
        s_diag.protocol_error_count++;
        return;
    }
    count = payload[26];
    if((count > CAMERA_BOARD_MAX_BEACON_COUNT) ||
       (length != (uint16)(CAMERA_BOARD_REAR_RESULT_HEADER_LEN +
                           count * CAMERA_BOARD_REAR_BEACON_WIRE_LEN)))
    {
        s_diag.protocol_error_count++;
        return;
    }

    memset(&frame, 0, sizeof(frame));
    frame.camera_frame_seq = CameraBoardLink_ReadU32(&payload[0]);
    frame.capture_time_us = CameraBoardLink_ReadU32(&payload[4]);
    frame.state_seq_used = CameraBoardLink_ReadU16(&payload[8]);
    frame.reset_generation = CameraBoardLink_ReadU16(&payload[10]);
    frame.result_flags = CameraBoardLink_ReadU16(&payload[12]);
    frame.car_x_mm = CameraBoardLink_ReadI16(&payload[14]);
    frame.car_y_mm = CameraBoardLink_ReadI16(&payload[16]);
    frame.car_raw_x_mm = CameraBoardLink_ReadI16(&payload[18]);
    frame.car_raw_y_mm = CameraBoardLink_ReadI16(&payload[20]);
    frame.recognition_time_us = CameraBoardLink_ReadU16(&payload[22]);
    frame.state_age_us = CameraBoardLink_ReadU16(&payload[24]);
    frame.beacon_count = count;
    for(index = 0U; index < count; index++)
    {
        uint16 offset = (uint16)(CAMERA_BOARD_REAR_RESULT_HEADER_LEN +
                                 index * CAMERA_BOARD_REAR_BEACON_WIRE_LEN);
        frame.beacon[index].x_mm = CameraBoardLink_ReadI16(&payload[offset]);
        frame.beacon[index].y_mm = CameraBoardLink_ReadI16(&payload[offset + 2U]);
        frame.beacon[index].detection_confidence = payload[offset + 4U];
        frame.beacon[index].position_sigma_cm = payload[offset + 5U];
        frame.beacon[index].flags = payload[offset + 6U];
    }

    s_latest_frame = frame;
    s_latest_frame_received = 1U;
    s_latest_frame_valid = (uint8)(
        ((frame.result_flags & CAMERA_BOARD_RESULT_FLAG_TIME_SYNC) != 0U) &&
        ((frame.result_flags & CAMERA_BOARD_RESULT_FLAG_STATE_FRESH) != 0U));
    s_last_result_rx_us = CameraBoardLink_TimeNowUs();
}

static void CameraBoardLink_DecodeRearStatus(const uint8 *payload, uint16 length)
{
    uint16 flags;

    if(length != CAMERA_BOARD_REAR_STATUS_PAYLOAD_LEN)
    {
        s_diag.protocol_error_count++;
        return;
    }

    flags = CameraBoardLink_ReadU16(&payload[4]);
    s_diag.rear_camera_ready =
        (flags & CAMERA_BOARD_STATUS_FLAG_CAMERA_READY) ? 1U : 0U;
    s_diag.rear_camera_core_alive =
        (flags & CAMERA_BOARD_STATUS_FLAG_CAMERA_CORE) ? 1U : 0U;
    s_diag.rear_time_sync_valid =
        (flags & CAMERA_BOARD_STATUS_FLAG_TIME_SYNC) ? 1U : 0U;
    s_diag.rear_extrinsic_nominal =
        (flags & CAMERA_BOARD_STATUS_FLAG_EXTRINSIC_NOMINAL) ? 1U : 0U;
    s_diag.rear_extrinsic_valid =
        (flags & CAMERA_BOARD_STATUS_FLAG_EXTRINSIC_VALID) ? 1U : 0U;
    s_diag.rear_captured_frame_count = CameraBoardLink_ReadU32(&payload[6]);
    s_diag.rear_dropped_frame_count = CameraBoardLink_ReadU32(&payload[10]);
    s_diag.rear_processed_frame_count = CameraBoardLink_ReadU32(&payload[14]);
    s_diag.rear_ipc_result_count = CameraBoardLink_ReadU32(&payload[18]);
    s_diag.rear_crc_error_count = CameraBoardLink_ReadU16(&payload[22]);
    s_diag.rear_sequence_gap_count = CameraBoardLink_ReadU16(&payload[24]);
    s_diag.rear_rx_overflow_count = CameraBoardLink_ReadU16(&payload[26]);
    s_diag.rear_tx_busy_drop_count = CameraBoardLink_ReadU16(&payload[28]);
    s_diag.rear_state_age_ms = CameraBoardLink_ReadU16(&payload[30]);
    s_diag.rear_sync_error_us = CameraBoardLink_ReadI16(&payload[32]);
    s_last_status_rx_us = CameraBoardLink_TimeNowUs();
}

static void CameraBoardLink_DispatchFrame(uint8 type,
                                          uint16 sequence,
                                          const uint8 *payload,
                                          uint16 length)
{
    if(s_last_rx_sequence_valid)
    {
        uint16 expected = (uint16)(s_last_rx_sequence + 1U);
        if(sequence != expected)
        {
            s_diag.sequence_gap_count += (uint16)(sequence - expected);
        }
    }
    s_last_rx_sequence = sequence;
    s_last_rx_sequence_valid = 1U;
    s_diag.rx_frame_count++;

    if(type == CAMERA_BOARD_MSG_REAR_RESULT)
    {
        CameraBoardLink_DecodeRearResult(payload, length);
    }
    else if(type == CAMERA_BOARD_MSG_REAR_STATUS)
    {
        CameraBoardLink_DecodeRearStatus(payload, length);
    }
    else
    {
        s_diag.protocol_error_count++;
    }
}

static void CameraBoardLink_ParseByte(uint8 byte, uint32 byte_cycle)
{
    uint16 crc_calculated;

    if(CameraBoardLink_ParseGapTimedOut(byte_cycle))
    {
        s_diag.protocol_error_count++;
        CameraBoardLink_ResetParser();
    }

    switch(s_parse_state)
    {
        case CAMERA_BOARD_PARSE_SOF1:
            if(byte == CAMERA_BOARD_PROTOCOL_SOF1)
            {
                s_parse_state = CAMERA_BOARD_PARSE_SOF2;
            }
            break;
        case CAMERA_BOARD_PARSE_SOF2:
            if(byte == CAMERA_BOARD_PROTOCOL_SOF2)
            {
                s_parse_state = CAMERA_BOARD_PARSE_VERSION;
            }
            else if(byte == CAMERA_BOARD_PROTOCOL_SOF1)
            {
                s_parse_state = CAMERA_BOARD_PARSE_SOF2;
            }
            else
            {
                CameraBoardLink_ResetParser();
            }
            break;
        case CAMERA_BOARD_PARSE_VERSION:
            if(byte != CAMERA_BOARD_PROTOCOL_VERSION)
            {
                s_diag.protocol_error_count++;
                CameraBoardLink_ResetParser();
                break;
            }
            s_parse_crc_data[0] = byte;
            s_parse_state = CAMERA_BOARD_PARSE_TYPE;
            break;
        case CAMERA_BOARD_PARSE_TYPE:
            s_parse_type = byte;
            s_parse_crc_data[1] = byte;
            s_parse_state = CAMERA_BOARD_PARSE_LENGTH_LOW;
            break;
        case CAMERA_BOARD_PARSE_LENGTH_LOW:
            s_parse_length = byte;
            s_parse_crc_data[2] = byte;
            s_parse_state = CAMERA_BOARD_PARSE_LENGTH_HIGH;
            break;
        case CAMERA_BOARD_PARSE_LENGTH_HIGH:
            s_parse_length |= (uint16)byte << 8;
            s_parse_crc_data[3] = byte;
            if(s_parse_length > CAMERA_BOARD_PROTOCOL_MAX_PAYLOAD)
            {
                s_diag.protocol_error_count++;
                CameraBoardLink_ResetParser();
                break;
            }
            s_parse_state = CAMERA_BOARD_PARSE_SEQUENCE_LOW;
            break;
        case CAMERA_BOARD_PARSE_SEQUENCE_LOW:
            s_parse_sequence = byte;
            s_parse_crc_data[4] = byte;
            s_parse_state = CAMERA_BOARD_PARSE_SEQUENCE_HIGH;
            break;
        case CAMERA_BOARD_PARSE_SEQUENCE_HIGH:
            s_parse_sequence |= (uint16)byte << 8;
            s_parse_crc_data[5] = byte;
            s_parse_index = 0U;
            s_parse_state = (s_parse_length > 0U) ?
                            CAMERA_BOARD_PARSE_PAYLOAD : CAMERA_BOARD_PARSE_CRC_LOW;
            break;
        case CAMERA_BOARD_PARSE_PAYLOAD:
            s_parse_payload[s_parse_index] = byte;
            s_parse_crc_data[6U + s_parse_index] = byte;
            s_parse_index++;
            if(s_parse_index >= s_parse_length)
            {
                s_parse_state = CAMERA_BOARD_PARSE_CRC_LOW;
            }
            break;
        case CAMERA_BOARD_PARSE_CRC_LOW:
            s_parse_crc_received = byte;
            s_parse_state = CAMERA_BOARD_PARSE_CRC_HIGH;
            break;
        case CAMERA_BOARD_PARSE_CRC_HIGH:
            s_parse_crc_received |= (uint16)byte << 8;
            crc_calculated = CameraBoardLink_Crc16(
                s_parse_crc_data, (uint16)(6U + s_parse_length));
            if(crc_calculated == s_parse_crc_received)
            {
                CameraBoardLink_DispatchFrame(s_parse_type,
                                              s_parse_sequence,
                                              s_parse_payload,
                                              s_parse_length);
            }
            else
            {
                s_diag.crc_error_count++;
            }
            CameraBoardLink_ResetParser();
            break;
        default:
            CameraBoardLink_ResetParser();
            break;
    }
    if(s_parse_state != CAMERA_BOARD_PARSE_SOF1)
    {
        s_parse_last_byte_cycle = byte_cycle;
    }
}

static void CameraBoardLink_ServiceRx(void)
{
    while(s_rx_tail != s_rx_head)
    {
        uint8 byte = s_rx_fifo[s_rx_tail];
        uint32 cycle = s_rx_cycle[s_rx_tail];
        s_rx_tail = (uint16)((s_rx_tail + 1U) % CAMERA_BOARD_RX_FIFO_SIZE);
        CameraBoardLink_ParseByte(byte, cycle);
    }
}

static void CameraBoardLink_SendMainState(void)
{
    uint8 payload[CAMERA_BOARD_MAIN_STATE_PAYLOAD_LEN] = {0};
    uint16 flags = CAMERA_BOARD_MAIN_FLAG_VISION_ENABLE;
    ICM_Health_t imu_health = {0};
    HeightTofSnapshot_t height_snapshot = {0};
    float height_cm;
    uint32 now_us = CameraBoardLink_TimeNowUs();

    HeightManager_GetTofSnapshot(&height_snapshot);
    height_cm = height_snapshot.filtered_height_cm;
    ICM_GetHealth(&imu_health);
    if(imu_health.attitude_valid && isfinite(current_roll) &&
       isfinite(current_pitch) && isfinite(current_yaw))
    {
        flags |= CAMERA_BOARD_MAIN_FLAG_POSE_VALID;
    }
    if(height_snapshot.valid && isfinite(height_cm) &&
       (height_cm > 1.0f) && (height_cm < 1000.0f))
    {
        flags |= CAMERA_BOARD_MAIN_FLAG_HEIGHT_VALID;
    }
    if(!Flight_Takeoff_Is_Active() &&
       !BeaconSearch_IsStartupRecognitionBlocked())
    {
        flags |= CAMERA_BOARD_MAIN_FLAG_BEACON_ENABLE;
    }

    s_state_sequence++;
    CameraBoardLink_WriteU32(&payload[0], now_us);
    CameraBoardLink_WriteU16(&payload[4], s_state_sequence);
    CameraBoardLink_WriteU16(&payload[6], s_reset_generation);
    CameraBoardLink_WriteI16(&payload[8], CameraBoardLink_FloatToI16(current_roll, 100.0f));
    CameraBoardLink_WriteI16(&payload[10], CameraBoardLink_FloatToI16(current_pitch, 100.0f));
    CameraBoardLink_WriteI16(&payload[12], CameraBoardLink_FloatToI16(current_yaw, 100.0f));
    CameraBoardLink_WriteU16(&payload[14], CameraBoardLink_FloatToU16(height_cm, 10.0f));
    CameraBoardLink_WriteU16(&payload[16], flags);

    if(CameraBoardLink_StartFrame(CAMERA_BOARD_MSG_MAIN_STATE,
                                  payload, sizeof(payload)))
    {
        CameraBoardStateHistoryEntry_t *entry =
            &s_state_history[s_state_history_head];

        entry->sequence = s_state_sequence;
        entry->flags = flags;
        entry->master_time_us = now_us;
        entry->yaw_deg = (float)CameraBoardLink_ReadI16(&payload[12]) * 0.01f;
        s_state_history_head = (uint8)((s_state_history_head + 1U) %
                                      CAMERA_BOARD_STATE_HISTORY_SIZE);
        if(s_state_history_count < CAMERA_BOARD_STATE_HISTORY_SIZE)
        {
            s_state_history_count++;
        }
        s_last_state_tx_us = now_us;
    }
    else
    {
        s_diag.tx_busy_drop_count++;
    }
}

void CameraBoardLink_MainInit(void)
{
    memset(&s_diag, 0, sizeof(s_diag));
    memset(&s_latest_frame, 0, sizeof(s_latest_frame));
    s_time_ready = 0U;
    s_time_us = 0ULL;
    s_time_cycle_remainder = 0U;
    s_mode_active = 1U;
    s_state_sequence = 0U;
    s_reset_generation = 1U;
    s_tx_sequence = 0U;
    s_state_history_head = 0U;
    s_state_history_count = 0U;
    memset(s_state_history, 0, sizeof(s_state_history));
    CameraBoardLink_ResetSession(1U);
    uart_init(CAMERA_BOARD_UART_INDEX,
              CAMERA_BOARD_UART_BAUDRATE,
              CAMERA_BOARD_UART_TX_PIN,
              CAMERA_BOARD_UART_RX_PIN);
    uart_rx_interrupt(CAMERA_BOARD_UART_INDEX, 1U);
    (void)CameraBoardLink_TimeNowUs();
    s_last_state_tx_us = CameraBoardLink_TimeNowUs() -
                         CAMERA_BOARD_STATE_PERIOD_US;
}

void CameraBoardLink_MainTask(void)
{
    uint8 mode_changed = CommMux_Service(Flight_Control_Is_Armed());
    uint32 now_us = CameraBoardLink_TimeNowUs();

    if(mode_changed)
    {
        Vofa_ClearRxState();
    }

    s_diag.mode_active = s_mode_active;
    CameraBoardLink_ServiceRx();
    CameraBoardLink_ServiceTx();
    if((uint32)(now_us - s_last_state_tx_us) >= CAMERA_BOARD_STATE_PERIOD_US)
    {
        CameraBoardLink_SendMainState();
        CameraBoardLink_ServiceTx();
    }

    s_diag.last_result_age_ms = (s_last_result_rx_us == 0U) ? 0xFFFFU :
        (uint16)(((uint32)(now_us - s_last_result_rx_us) / 1000U) > 0xFFFFU ?
                 0xFFFFU : ((uint32)(now_us - s_last_result_rx_us) / 1000U));
    s_diag.last_status_age_ms = (s_last_status_rx_us == 0U) ? 0xFFFFU :
        (uint16)(((uint32)(now_us - s_last_status_rx_us) / 1000U) > 0xFFFFU ?
                 0xFFFFU : ((uint32)(now_us - s_last_status_rx_us) / 1000U));
    s_diag.online = (uint8)(s_last_result_rx_us != 0U &&
        ((uint32)(now_us - s_last_result_rx_us) <= CAMERA_BOARD_LINK_TIMEOUT_US));
    if(!s_diag.online)
    {
        s_latest_frame_valid = 0U;
    }
}

void CameraBoardLink_RxIsr(void)
{
    uint8 byte;
    while(uart_query_byte(CAMERA_BOARD_UART_INDEX, &byte))
    {
        uint32 cycle = DWT->CYCCNT;
        uint16 next = (uint16)((s_rx_head + 1U) % CAMERA_BOARD_RX_FIFO_SIZE);
        if(next == s_rx_tail)
        {
            s_diag.rx_overflow_count++;
            continue;
        }
        s_rx_fifo[s_rx_head] = byte;
        s_rx_cycle[s_rx_head] = cycle;
        s_rx_head = next;
    }
}

uint8 CameraBoardLink_GetRearFrame(CameraBoardVisionFrame_t *frame)
{
    if((frame == NULL) || !s_latest_frame_valid || !CameraBoardLink_IsOnline())
    {
        return 0U;
    }
    *frame = s_latest_frame;
    return 1U;
}

uint8 CameraBoardLink_GetLatestRearFrame(CameraBoardVisionFrame_t *frame)
{
    if((frame == NULL) || !s_latest_frame_received)
    {
        return 0U;
    }

    *frame = s_latest_frame;
    return 1U;
}

uint8 CameraBoardLink_GetStateYaw(uint16 state_sequence, float *yaw_deg)
{
    uint8 offset;

    if(yaw_deg == NULL)
    {
        return 0U;
    }
    for(offset = 0U; offset < s_state_history_count; offset++)
    {
        uint8 index = (uint8)((s_state_history_head +
            CAMERA_BOARD_STATE_HISTORY_SIZE - 1U - offset) %
            CAMERA_BOARD_STATE_HISTORY_SIZE);
        const CameraBoardStateHistoryEntry_t *entry = &s_state_history[index];

        if(entry->sequence == state_sequence)
        {
            if((entry->flags & CAMERA_BOARD_MAIN_FLAG_POSE_VALID) == 0U)
            {
                return 0U;
            }
            *yaw_deg = entry->yaw_deg;
            return 1U;
        }
    }
    return 0U;
}

uint16 CameraBoardLink_GetResetGeneration(void)
{
    return s_reset_generation;
}

void CameraBoardLink_GetDiag(CameraBoardLinkDiag_t *diag)
{
    if(diag != NULL) *diag = s_diag;
}

uint8 CameraBoardLink_IsOnline(void)
{
    uint32 now_us = CameraBoardLink_TimeNowUs();
    return (uint8)(s_mode_active && (s_last_result_rx_us != 0U) &&
        ((uint32)(now_us - s_last_result_rx_us) <= CAMERA_BOARD_LINK_TIMEOUT_US));
}
