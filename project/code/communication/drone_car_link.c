#include "communication/drone_car_link.h"
#include "common/project_headfile.h"

/*
 * 串口协议约定
 *
 * 1. 下行命令帧（小车 -> 无人机）
 *    byte0  : 0xA5
 *    byte1  : 0x5A
 *    byte2  : type = 0x01
 *    byte3  : len  = 10
 *    byte4  : flags
 *    byte5  : control_mode
 *    byte6~7   : throttle   (int16, little-endian)
 *    byte8~9   : roll_cd    (int16, centi-degree)
 *    byte10~11 : pitch_cd   (int16, centi-degree)
 *    byte12~13 : yaw_cd     (int16, centi-degree)
 *    byte14 : checksum = type + len + payload bytes
 *
 * 2. 上行状态帧（无人机 -> 小车）
 *    byte0  : 0xA5
 *    byte1  : 0x5A
 *    byte2  : type = 0x81
 *    byte3  : len  = 16
 *    byte4  : flags
 *    byte5  : control_mode
 *    byte6  : emergency_stop_reason
 *    byte7  : reserved
 *    byte8~9   : throttle target
 *    byte10~11 : current roll   (centi-degree)
 *    byte12~13 : current pitch  (centi-degree)
 *    byte14~15 : current yaw    (centi-degree)
 *    byte16~17 : voltage        (centi-volt)
 *    byte18~19 : fused height   (cm)
 *    byte20 : checksum
 *
 * 3. 接收路径
 *    UART4 ISR -> DroneCarLink_UartRxCallback() -> 拼完整命令帧
 *    -> 10ms 服务任务取出命令 -> 更新飞控运行状态
 *
 * 4. 发送路径
 *    10ms 服务任务只置位“需要发状态”
 *    -> 主循环里的 DroneCarLink_MainLoopTask() 真正组包并发送
 */

#define DRONE_CAR_LINK_FRAME_HEAD1          0xA5U
#define DRONE_CAR_LINK_FRAME_HEAD2          0x5AU
#define DRONE_CAR_LINK_MSG_COMMAND          0x01U
#define DRONE_CAR_LINK_MSG_STATUS           0x81U

#define DRONE_CAR_LINK_COMMAND_PAYLOAD_LEN  10U
#define DRONE_CAR_LINK_STATUS_PAYLOAD_LEN   16U
#define DRONE_CAR_LINK_PAYLOAD_MAX_LEN      32U

#define DRONE_CAR_LINK_STATUS_PERIOD_MS     50U
#define DRONE_CAR_LINK_TIMEOUT_MS           300U

typedef enum
{
    /* 逐字节状态机：按 帧头 -> 类型 -> 长度 -> 载荷 -> 校验 的顺序推进。 */
    DRONE_CAR_LINK_RX_WAIT_HEAD1 = 0,
    DRONE_CAR_LINK_RX_WAIT_HEAD2,
    DRONE_CAR_LINK_RX_WAIT_TYPE,
    DRONE_CAR_LINK_RX_WAIT_LEN,
    DRONE_CAR_LINK_RX_WAIT_PAYLOAD,
    DRONE_CAR_LINK_RX_WAIT_CHECKSUM,
} drone_car_link_rx_state_enum;

typedef struct
{
    /* flags 里放解锁、外部角度控制、急停等控制位。 */
    uint8 flags;
    /* 对应现有飞控里的 CONTROL_MODE_*。 */
    uint8 control_mode;
    /* 油门目标，直接映射到飞控油门接口。 */
    int16 throttle;
    /* 外部姿态目标，单位是 0.01 度。 */
    int16 roll_cd;
    int16 pitch_cd;
    int16 yaw_cd;
} drone_car_link_command_frame_t;

/* 初始化完成标志，防止任务在串口尚未启动时误运行。 */
static volatile uint8 drone_car_link_initialized = 0U;
/* ISR 已经拼出一帧完整命令，等待 10ms 服务任务取走。 */
static volatile uint8 drone_car_link_packet_ready = 0U;
static volatile drone_car_link_command_frame_t drone_car_link_pending_command = {0};

/* 最近一次已经应用到飞控的命令快照，便于调试和后续扩展。 */

/* ISR 接收状态机的运行时变量。 */
static volatile uint8 drone_car_link_rx_state = DRONE_CAR_LINK_RX_WAIT_HEAD1;
static volatile uint8 drone_car_link_rx_type = 0U;
static volatile uint8 drone_car_link_rx_len = 0U;
static volatile uint8 drone_car_link_rx_index = 0U;
static volatile uint8 drone_car_link_rx_checksum = 0U;
static volatile uint8 drone_car_link_rx_payload[DRONE_CAR_LINK_PAYLOAD_MAX_LEN] = {0};

static volatile uint8 drone_car_link_external_angle_active = 0U;
static volatile uint8 drone_car_link_link_alive = 0U;
static volatile uint8 drone_car_link_status_send_pending = 0U;

static volatile float drone_car_link_target_roll_deg = 0.0f;
static volatile float drone_car_link_target_pitch_deg = 0.0f;
static volatile float drone_car_link_target_yaw_deg = 0.0f;

/* 链路超时计数与状态回传节拍，都由 10ms 服务任务维护。 */
static uint16 drone_car_link_timeout_ms = DRONE_CAR_LINK_TIMEOUT_MS + 10U;
static uint16 drone_car_link_status_tick_ms = 0U;

/* 丢包、坏包或一帧处理结束后，把接收状态机恢复到“等待帧头”。 */
static void drone_car_link_reset_rx_state(void)
{
    drone_car_link_rx_state = DRONE_CAR_LINK_RX_WAIT_HEAD1;
    drone_car_link_rx_type = 0U;
    drone_car_link_rx_len = 0U;
    drone_car_link_rx_index = 0U;
    drone_car_link_rx_checksum = 0U;
}

/* 简单加和校验。
 * 这里不包含两个帧头，只对 type + len + payload 求和。
 */
static uint8 drone_car_link_calc_checksum(uint8 type, uint8 len, const volatile uint8 *payload)
{
    uint8 checksum = (uint8)(type + len);
    uint8 i = 0U;

    for(i = 0U; i < len; i++)
    {
        checksum = (uint8)(checksum + payload[i]);
    }

    return checksum;
}

/* 从 little-endian 字节流中读取 int16。 */
static int16 drone_car_link_read_i16_le(const volatile uint8 *buffer, uint8 offset)
{
    uint16 value = (uint16)buffer[offset] | ((uint16)buffer[offset + 1U] << 8U);
    return (int16)value;
}

/* 向发送缓冲按 little-endian 写入 uint16。 */
static void drone_car_link_write_u16_le(uint8 *buffer, uint8 offset, uint16 value)
{
    buffer[offset] = (uint8)(value & 0x00FFU);
    buffer[offset + 1U] = (uint8)((value >> 8U) & 0x00FFU);
}

/* int16 和 uint16 在发包层面都按 16bit little-endian 写入。 */
static void drone_car_link_write_i16_le(uint8 *buffer, uint8 offset, int16 value)
{
    drone_car_link_write_u16_le(buffer, offset, (uint16)value);
}

/* 通用限幅函数，用于姿态目标、状态打包等场景。 */
static float drone_car_link_clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }
    else if(value > max_value)
    {
        value = max_value;
    }

    return value;
}

/* 将浮点量限幅并转成无符号 16bit，给状态帧打包使用。 */
static uint16 drone_car_link_clamp_u16(float value, float min_value, float max_value)
{
    value = drone_car_link_clamp_float(value, min_value, max_value);
    return (uint16)(value + 0.5f);
}

/* 把角度值转成 0.01 度单位，便于用 int16 压缩传输。 */
static int16 drone_car_link_float_to_centideg(float value)
{
    float scaled = drone_car_link_clamp_float(value, -327.67f, 327.67f) * 100.0f;

    if(scaled >= 0.0f)
    {
        scaled += 0.5f;
    }
    else
    {
        scaled -= 0.5f;
    }

    return (int16)scaled;
}

/* 当前 ISR 已经收到完整命令载荷时，把它整理成结构体缓存起来。
 * 这里只负责“存”，不直接改飞控状态，避免中断里做太多业务逻辑。
 */
static void drone_car_link_store_pending_command(void)
{
    drone_car_link_command_frame_t command = {0};

    if(drone_car_link_rx_len != DRONE_CAR_LINK_COMMAND_PAYLOAD_LEN)
    {
        return;
    }

    command.flags = drone_car_link_rx_payload[0];
    command.control_mode = drone_car_link_rx_payload[1];
    command.throttle = drone_car_link_read_i16_le(drone_car_link_rx_payload, 2U);
    command.roll_cd = drone_car_link_read_i16_le(drone_car_link_rx_payload, 4U);
    command.pitch_cd = drone_car_link_read_i16_le(drone_car_link_rx_payload, 6U);
    command.yaw_cd = drone_car_link_read_i16_le(drone_car_link_rx_payload, 8U);

    drone_car_link_pending_command = command;
    drone_car_link_packet_ready = 1U;
}

/* 10ms 任务取走一帧待处理命令。
 * 这里短暂关中断，避免 ISR 在复制结构体时同时改写 pending buffer。
 */
static uint8 drone_car_link_fetch_pending_command(drone_car_link_command_frame_t *command)
{
    uint32 interrupt_state = 0U;
    uint8 has_packet = 0U;

    if(command == NULL)
    {
        return 0U;
    }

    interrupt_state = interrupt_global_disable();

    if(drone_car_link_packet_ready)
    {
        *command = drone_car_link_pending_command;
        drone_car_link_packet_ready = 0U;
        has_packet = 1U;
    }

    interrupt_global_enable(interrupt_state);
    return has_packet;
}

/* 把外部命令下发的姿态目标转换成飞控当前使用的角度目标。 */
static void drone_car_link_set_external_targets(float roll_deg, float pitch_deg, float yaw_deg)
{
    drone_car_link_target_roll_deg = drone_car_link_clamp_float(roll_deg, -SAFE_MAX_ANGLE_ROLL, SAFE_MAX_ANGLE_ROLL);
    drone_car_link_target_pitch_deg = drone_car_link_clamp_float(pitch_deg, -SAFE_MAX_ANGLE_PITCH, SAFE_MAX_ANGLE_PITCH);
    drone_car_link_target_yaw_deg = drone_car_link_clamp_float(yaw_deg, -SAFE_MAX_ANGLE_YAW, SAFE_MAX_ANGLE_YAW);

    Flight_Control_Set_Target_Angle(drone_car_link_target_roll_deg,
                                    drone_car_link_target_pitch_deg,
                                    drone_car_link_target_yaw_deg);
}

/* 当链路失效、切换回本地控制或急停时，清掉外部姿态目标状态。 */
static void drone_car_link_clear_external_targets(void)
{
    drone_car_link_external_angle_active = 0U;
    drone_car_link_target_roll_deg = 0.0f;
    drone_car_link_target_pitch_deg = 0.0f;
    drone_car_link_target_yaw_deg = 0.0f;
}

/* 把一帧串口命令真正映射到飞控运行时状态。
 * 这里是链路层和飞控层的主要连接点。
 */
static void drone_car_link_apply_command(const drone_car_link_command_frame_t *command)
{
    uint8 want_arm = 0U;
    uint8 want_external_angle = 0U;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;

    if(command == NULL)
    {
        return;
    }

    drone_car_link_link_alive = 1U;
    drone_car_link_timeout_ms = 0U;

    if(command->flags & DRONE_CAR_LINK_FLAG_ESTOP)
    {
        drone_car_link_clear_external_targets();
        Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_MANUAL_BUTTON);
        return;
    }

    want_arm = ((command->flags & DRONE_CAR_LINK_FLAG_ARMED) != 0U) ? 1U : 0U;
    want_external_angle = ((command->flags & DRONE_CAR_LINK_FLAG_EXTERNAL_ANGLE) != 0U) ? 1U : 0U;

    if(want_arm)
    {
        if(!Flight_Control_Is_Armed())
        {
            Flight_Runtime_Clear_Emergency_Stop_Latch();
            Flight_Control_Arm();
        }
    }
    else if(Flight_Control_Is_Armed())
    {
        Flight_Control_Disarm();
    }

    Flight_Control_Set_Mode(command->control_mode);
    Flight_Control_Set_Throttle((float)command->throttle);

    if(want_external_angle && command->control_mode == CONTROL_MODE_ANGLE)
    {
        roll_deg = (float)command->roll_cd * 0.01f;
        pitch_deg = (float)command->pitch_cd * 0.01f;
        yaw_deg = (float)command->yaw_cd * 0.01f;

        drone_car_link_set_external_targets(roll_deg, pitch_deg, yaw_deg);
        drone_car_link_external_angle_active = 1U;
    }
    else
    {
        drone_car_link_clear_external_targets();
    }
}

/* 失联超时处理。
 * 超时后停止使用外部姿态目标，并在已解锁时触发安全急停。
 */
static void drone_car_link_handle_timeout(void)
{
    if(drone_car_link_timeout_ms <= DRONE_CAR_LINK_TIMEOUT_MS)
    {
        return;
    }

    if(drone_car_link_link_alive)
    {
        drone_car_link_link_alive = 0U;
        drone_car_link_clear_external_targets();

        if(Flight_Control_Is_Armed())
        {
            Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_LINK_TIMEOUT);
        }
    }
}

/* 组装上行状态载荷。
 * 注意这里只组 payload，不负责加帧头和发送。
 */
static void drone_car_link_build_status_payload(uint8 *payload)
{
    uint8 flags = 0U;
    uint16 throttle_value = 0U;
    uint16 voltage_cv = 0U;
    uint16 height_cm = 0U;

    if(payload == NULL)
    {
        return;
    }

    if(Flight_Control_Is_Armed())
    {
        flags |= DRONE_CAR_LINK_FLAG_ARMED;
    }
    if(drone_car_link_link_alive)
    {
        flags |= 0x08U;
    }
    if(drone_car_link_external_angle_active)
    {
        flags |= DRONE_CAR_LINK_FLAG_EXTERNAL_ANGLE;
    }
    if(low_voltage_protect_flag)
    {
        flags |= 0x10U;
    }
    if(emergency_stop_triggered)
    {
        flags |= 0x20U;
    }

    throttle_value = drone_car_link_clamp_u16(Flight_Control_Get_Target_Throttle(), 0.0f, SAFE_MAX_THROTTLE);
    voltage_cv = drone_car_link_clamp_u16(voltage * 100.0f, 0.0f, 65535.0f);
    height_cm = drone_car_link_clamp_u16(g_height_result.height_cm, 0.0f, 65535.0f);

    payload[0] = flags;
    payload[1] = Flight_Control_Get_Mode();
    payload[2] = emergency_stop_reason;
    payload[3] = 0U;

    drone_car_link_write_u16_le(payload, 4U, throttle_value);
    drone_car_link_write_i16_le(payload, 6U, drone_car_link_float_to_centideg(current_roll));
    drone_car_link_write_i16_le(payload, 8U, drone_car_link_float_to_centideg(current_pitch));
    drone_car_link_write_i16_le(payload, 10U, drone_car_link_float_to_centideg(current_yaw));
    drone_car_link_write_u16_le(payload, 12U, voltage_cv);
    drone_car_link_write_u16_le(payload, 14U, height_cm);
}

/* 初始化链路：
 * 1. 清空本模块内部状态
 * 2. 让 UART4 的接收回调指向本模块
 * 3. 配置 UART4 并开启 RX 中断
 */
void DroneCarLink_Init(void)
{
    drone_car_link_reset_rx_state();
    drone_car_link_clear_external_targets();
    drone_car_link_link_alive = 0U;
    drone_car_link_packet_ready = 0U;
    drone_car_link_status_send_pending = 0U;
    drone_car_link_timeout_ms = DRONE_CAR_LINK_TIMEOUT_MS + 10U;
    drone_car_link_status_tick_ms = 0U;

    set_wireless_type(RECEIVER_UART, DroneCarLink_UartRxCallback);
    uart_init(DRONE_CAR_LINK_UART_INDEX,
              DRONE_CAR_LINK_UART_BAUDRATE,
              DRONE_CAR_LINK_UART_TX_PIN,
              DRONE_CAR_LINK_UART_RX_PIN);
    uart_rx_interrupt(DRONE_CAR_LINK_UART_INDEX, 1U);

    drone_car_link_initialized = 1U;
}

/* UART4 接收中断回调。
 * 这里持续把底层串口缓存里的字节取空，并用状态机拼一帧。
 *
 * 为什么一定要在这里取空？
 * 因为底层 UART 驱动的接收缓存很小，如果只等主循环慢慢读，
 * 连续字节流很容易被新数据覆盖。
 */
void DroneCarLink_UartRxCallback(void)
{
    uint8 byte = 0U;
    uint8 expected_checksum = 0U;

    while(uart_query_byte(DRONE_CAR_LINK_UART_INDEX, &byte))
    {
        switch(drone_car_link_rx_state)
        {
            case DRONE_CAR_LINK_RX_WAIT_HEAD1:
                if(byte == DRONE_CAR_LINK_FRAME_HEAD1)
                {
                    drone_car_link_rx_state = DRONE_CAR_LINK_RX_WAIT_HEAD2;
                }
                break;

            case DRONE_CAR_LINK_RX_WAIT_HEAD2:
                if(byte == DRONE_CAR_LINK_FRAME_HEAD2)
                {
                    drone_car_link_rx_state = DRONE_CAR_LINK_RX_WAIT_TYPE;
                }
                else if(byte != DRONE_CAR_LINK_FRAME_HEAD1)
                {
                    drone_car_link_reset_rx_state();
                }
                break;

            case DRONE_CAR_LINK_RX_WAIT_TYPE:
                drone_car_link_rx_type = byte;
                drone_car_link_rx_state = DRONE_CAR_LINK_RX_WAIT_LEN;
                break;

            case DRONE_CAR_LINK_RX_WAIT_LEN:
                drone_car_link_rx_len = byte;
                drone_car_link_rx_index = 0U;

                if(drone_car_link_rx_len == 0U || drone_car_link_rx_len > DRONE_CAR_LINK_PAYLOAD_MAX_LEN)
                {
                    drone_car_link_reset_rx_state();
                }
                else
                {
                    drone_car_link_rx_state = DRONE_CAR_LINK_RX_WAIT_PAYLOAD;
                }
                break;

            case DRONE_CAR_LINK_RX_WAIT_PAYLOAD:
                drone_car_link_rx_payload[drone_car_link_rx_index++] = byte;

                if(drone_car_link_rx_index >= drone_car_link_rx_len)
                {
                    drone_car_link_rx_state = DRONE_CAR_LINK_RX_WAIT_CHECKSUM;
                }
                break;

            case DRONE_CAR_LINK_RX_WAIT_CHECKSUM:
                expected_checksum = drone_car_link_calc_checksum(drone_car_link_rx_type,
                                                                 drone_car_link_rx_len,
                                                                 drone_car_link_rx_payload);

                if(byte == expected_checksum && drone_car_link_rx_type == DRONE_CAR_LINK_MSG_COMMAND)
                {
                    drone_car_link_store_pending_command();
                }

                drone_car_link_reset_rx_state();
                break;

            default:
                drone_car_link_reset_rx_state();
                break;
        }
    }
}

/* 10ms 周期服务任务。
 * 这个任务负责：
 * 1. 把最新命令应用到飞控
 * 2. 维护链路超时
 * 3. 按周期发出“该回传状态了”的请求
 */
void DroneCarLink_Service_Task_10ms(void)
{
    drone_car_link_command_frame_t command = {0};

    if(!drone_car_link_initialized)
    {
        return;
    }

    if(drone_car_link_fetch_pending_command(&command))
    {
        drone_car_link_apply_command(&command);
    }
    else if(drone_car_link_timeout_ms < 65525U)
    {
        drone_car_link_timeout_ms = (uint16)(drone_car_link_timeout_ms + 10U);
    }

    drone_car_link_handle_timeout();

    drone_car_link_status_tick_ms = (uint16)(drone_car_link_status_tick_ms + 10U);
    if(drone_car_link_status_tick_ms >= DRONE_CAR_LINK_STATUS_PERIOD_MS)
    {
        drone_car_link_status_tick_ms = 0U;
        drone_car_link_status_send_pending = 1U;
    }
}

/* 主循环发送任务。
 * 真正的串口发送放在这里，而不是 PIT / UART ISR 里，
 * 因为底层 uart_write_buffer() 是阻塞式的。
 */
void DroneCarLink_MainLoopTask(void)
{
    uint8 frame[2U + 1U + 1U + DRONE_CAR_LINK_STATUS_PAYLOAD_LEN + 1U] = {0};
    uint8 payload[DRONE_CAR_LINK_STATUS_PAYLOAD_LEN] = {0};
    uint8 frame_len = 0U;

    if(!drone_car_link_initialized || !drone_car_link_status_send_pending)
    {
        return;
    }

    drone_car_link_status_send_pending = 0U;

    drone_car_link_build_status_payload(payload);

    frame[0] = DRONE_CAR_LINK_FRAME_HEAD1;
    frame[1] = DRONE_CAR_LINK_FRAME_HEAD2;
    frame[2] = DRONE_CAR_LINK_MSG_STATUS;
    frame[3] = DRONE_CAR_LINK_STATUS_PAYLOAD_LEN;
    memcpy(&frame[4], payload, sizeof(payload));
    frame[4U + DRONE_CAR_LINK_STATUS_PAYLOAD_LEN] = drone_car_link_calc_checksum(DRONE_CAR_LINK_MSG_STATUS,
                                                                                  DRONE_CAR_LINK_STATUS_PAYLOAD_LEN,
                                                                                  payload);

    frame_len = (uint8)(sizeof(frame));
    uart_write_buffer(DRONE_CAR_LINK_UART_INDEX, frame, frame_len);
}

/* 控制环用它来判断：当前姿态目标是否应该由串口链路接管。 */
uint8 DroneCarLink_UsesExternalTargets(void)
{
    return (drone_car_link_external_angle_active && drone_car_link_link_alive) ? 1U : 0U;
}

/* 控制环在需要时读取当前有效的外部姿态目标。 */
void DroneCarLink_Get_Target_Angles(float *roll, float *pitch, float *yaw)
{
    if(roll != NULL)
    {
        *roll = drone_car_link_target_roll_deg;
    }
    if(pitch != NULL)
    {
        *pitch = drone_car_link_target_pitch_deg;
    }
    if(yaw != NULL)
    {
        *yaw = drone_car_link_target_yaw_deg;
    }
}
