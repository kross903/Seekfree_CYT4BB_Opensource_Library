#ifndef _core_link_h_
#define _core_link_h_

#include "zf_common_headfile.h"

#define CORE_LINK_UART_INDEX            (UART_4)
#define CORE_LINK_UART_BAUDRATE         (115200)
#define CORE_LINK_UART_TX_PIN           (UART4_TX_P14_1)
#define CORE_LINK_UART_RX_PIN           (UART4_RX_P14_0)

#define CORE_LINK_RX_FIFO_SIZE          (256)
#define CORE_LINK_PAYLOAD_MAX_LEN       (64)
#define CORE_LINK_PARAM_DECIMAL_NUM     (3)

#define CORE_LINK_SOF1                  (0xA5)
#define CORE_LINK_SOF2                  (0x5A)
extern volatile uint8 Car_Copy_Screen_Flag; // Link scheduler tick flag.

typedef enum
{
    CORE_LINK_PROTOCOL_TEXT        = 0x01,                                      // Text payload, such as "HELLO"
    CORE_LINK_PROTOCOL_PARAM_ASCII = 0x02,                                      // Param payload, such as "R1=1.234"
    CORE_LINK_PROTOCOL_DRONE_STATUS = 0x10,                                     // Fixed-length drone status summary
    CORE_LINK_PROTOCOL_SCREEN_CMD   = 0x11,                                     // IPS200 mirror command stream
    CORE_LINK_PROTOCOL_BEACON_MAP   = 0x12,                                     // Fixed-length beacon map packet
    CORE_LINK_PROTOCOL_BEACON_MAP_UPLINK = 0x13,                                // Fixed-length car uplink beacon map packet
    CORE_LINK_PROTOCOL_BEACON_OBSERVATIONS = 0x14,                              // Variable-count beacon observations
    CORE_LINK_PROTOCOL_CAR_TRAJECTORY = 0x15,                                   // Unified car motion and trajectory state
}core_link_protocol_enum;

typedef enum
{
    CORE_LINK_SCREEN_PACKET_BEGIN = 1,
    CORE_LINK_SCREEN_PACKET_DATA,
    CORE_LINK_SCREEN_PACKET_END,
}core_link_screen_packet_enum;

typedef enum
{
    CORE_LINK_SCREEN_CMD_CLEAR = 1,
    CORE_LINK_SCREEN_CMD_FULL,
    CORE_LINK_SCREEN_CMD_SET_DIR,
    CORE_LINK_SCREEN_CMD_SET_FONT,
    CORE_LINK_SCREEN_CMD_SET_COLOR,
    CORE_LINK_SCREEN_CMD_DRAW_LINE,
    CORE_LINK_SCREEN_CMD_SHOW_CHAR,
    CORE_LINK_SCREEN_CMD_SHOW_STRING,
    CORE_LINK_SCREEN_CMD_SHOW_INT,
    CORE_LINK_SCREEN_CMD_SHOW_UINT,
    CORE_LINK_SCREEN_CMD_SHOW_FLOAT,
    CORE_LINK_SCREEN_CMD_DRAW_POINT,
}core_link_screen_cmd_enum;

typedef struct
{
    uint8 command;
    uint8 item;
    float value;
}core_link_param_packet_struct;

typedef struct
{
    uint8 armed;
    uint8 control_mode;
    uint8 low_voltage;
    uint8 beacon_count;
    uint16 voltage_mv;
    int16 target_throttle;
    int16 base_throttle;
    int16 roll_ddeg;
    int16 pitch_ddeg;
    int16 yaw_ddeg;
    int16 tof_height_dcm;
    int16 fused_height_dcm;
} core_link_drone_status_packet_struct;

#define CORE_LINK_BEACON_MAP_COUNT         (5U)
#define CORE_LINK_BEACON_MAP_PAYLOAD_LEN   (37U)
#define CORE_LINK_BEACON_MAP_UPLINK_PAYLOAD_LEN   (37U)
#define CORE_LINK_BEACON_MAP_UPLINK_LEGACY_PAYLOAD_LEN (43U)
#define CORE_LINK_CAR_MOTION_FLAG_VALID            (0x01U)
#define CORE_LINK_CAR_MOTION_FLAG_SLIP_SUSPECT     (0x02U)
#define CORE_LINK_CAR_MOTION_FLAG_TARGET_VALID     (0x04U)
#define CORE_LINK_CAR_TRAJECTORY_VERSION            (1U)
#define CORE_LINK_CAR_TRAJECTORY_PAYLOAD_LEN        (40U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_MOTION_VALID  (0x01U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_PLAN_VALID    (0x02U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_NAV_TARGET    (0x04U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_PREDICTED     (0x08U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_STOP_ONLY     (0x10U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_FINISHED      (0x20U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_GUIDE         (0x40U)
#define CORE_LINK_CAR_TRAJECTORY_FLAG_POSITION_TARGET (0x80U)
#define CORE_LINK_PREDICTED_BEACON_SOURCE_NONE      (0U)
#define CORE_LINK_PREDICTED_BEACON_SOURCE_CURRENT   (1U)
#define CORE_LINK_PREDICTED_BEACON_SOURCE_NEXT      (2U)
#define CORE_LINK_PREDICTED_BEACON_SOURCE_GUIDE     (3U)
#if CORE_LINK_CAR_TRAJECTORY_PAYLOAD_LEN > CORE_LINK_PAYLOAD_MAX_LEN
#error "Car trajectory payload exceeds core link payload limit"
#endif

typedef struct
{
    uint8 id;
    int16 x_mm;
    int16 y_mm;
    uint8 state;
} core_link_beacon_entry_struct;

typedef struct
{
    uint8 car_visible;
    int16 car_x_mm;
    int16 car_y_mm;
    /* IMU-derived car yaw relative to UAV yaw; never visual marker heading. */
    int16 car_yaw_rel_uav_ddeg;
    core_link_beacon_entry_struct beacon[CORE_LINK_BEACON_MAP_COUNT];
} core_link_beacon_map_packet_struct;

typedef struct
{
    uint8 car_visible;
    int16 car_x_mm;
    int16 car_y_mm;
    /* Car IMU/Mahony yaw in the aligned yaw reference shared with UAV. */
    int16 car_absolute_yaw_ddeg;
    core_link_beacon_entry_struct beacon[CORE_LINK_BEACON_MAP_COUNT];
    /* Present only when receiving the legacy 43-byte 0x13 frame. */
    uint8 motion_flags;
    uint8 motion_sequence;
    int16 car_forward_speed_dcms;
    int16 car_left_speed_dcms;
} core_link_beacon_map_uplink_packet_struct;

typedef struct
{
    uint8 version;
    uint8 flags;
    uint16 sequence;
    uint16 plan_generation;
    uint8 current_segment;
    uint8 predicted_beacon_source;
    int16 car_absolute_yaw_ddeg;
    int16 actual_forward_speed_dcms;
    int16 actual_left_speed_dcms;
    int16 command_forward_speed_dcms;
    int16 command_left_speed_dcms;
    int16 planned_forward_acc_cms2;
    int16 planned_left_acc_cms2;
    int16 planned_forward_jerk_cms3;
    int16 planned_left_jerk_cms3;
    int16 trajectory_error_forward_mm;
    int16 trajectory_error_left_mm;
    int16 target_forward_mm;
    int16 target_left_mm;
    uint8 predicted_beacon_id;
    uint8 reserved;
    int16 predicted_beacon_forward_mm;
    int16 predicted_beacon_left_mm;
} core_link_car_trajectory_packet_struct;

#define CORE_LINK_BEACON_OBSERVATION_MAX_COUNT     (6U)
#define CORE_LINK_BEACON_OBSERVATION_HEADER_LEN    (14U)
#define CORE_LINK_BEACON_OBSERVATION_WIRE_LEN      (8U)
#define CORE_LINK_BEACON_TRACK_MAX_COUNT           (12U)
#define CORE_LINK_BEACON_OBSERVATION_FLAG_LIT      (0x01U)
#define CORE_LINK_BEACON_POSITION_SIGMA_MIN_CM     (5U)
#define CORE_LINK_BEACON_POSITION_SIGMA_MAX_CM     (15U)
#define CORE_LINK_BEACON_FRAME_FLAG_CAR_POSITION_VALID (0x01U)
#define CORE_LINK_BEACON_FRAME_FLAG_CAR_YAW_VALID      (0x02U)
#define CORE_LINK_BEACON_FRAME_FLAG_UAV_YAW_VALID      (0x04U)
#define CORE_LINK_BEACON_FRAME_FLAG_TRACKER_RESET      (0x08U)
#define CORE_LINK_BEACON_FRAME_FLAG_SEARCH_ACTIVE      (0x10U)
#define CORE_LINK_BEACON_FRAME_FLAG_SEARCH_FAILED      (0x20U)
#define CORE_LINK_BEACON_FRAME_FLAG_CAR_RAW_DETECTED   (0x40U)
#define CORE_LINK_BEACON_OBSERVATION_PAYLOAD_MAX_LEN \
    (CORE_LINK_BEACON_OBSERVATION_HEADER_LEN + \
     CORE_LINK_BEACON_OBSERVATION_MAX_COUNT * CORE_LINK_BEACON_OBSERVATION_WIRE_LEN)

#if CORE_LINK_BEACON_OBSERVATION_PAYLOAD_MAX_LEN > CORE_LINK_PAYLOAD_MAX_LEN
#error "Beacon observation payload exceeds core link payload limit"
#endif

/* 0x14 little-endian header: sequence, count, flags, car visibility,
 * car position in UAV X-forward/Y-left, UAV IMU yaw sampled with the vision
 * frame, then IMU-derived car yaw relative to UAV yaw. The relative yaw must
 * never be replaced by visual marker heading_deg. Both IMUs must use the same
 * positive yaw direction and an aligned yaw zero. Track IDs are session-local. */

typedef struct
{
    uint8 track_id;
    int16 x_mm;
    int16 y_mm;
    uint8 detection_confidence;
    uint8 position_sigma_cm;
    uint8 flags;
} BeaconObservation;

typedef struct
{
    uint16 sequence;
    uint8 count;
    uint8 frame_flags;
    uint8 car_visible;
    uint8 reserved;
    int16 car_x_mm;
    int16 car_y_mm;
    int16 uav_yaw_ddeg;
    int16 car_yaw_rel_uav_ddeg;
    BeaconObservation observation[CORE_LINK_BEACON_OBSERVATION_MAX_COUNT];
} core_link_beacon_observation_packet_struct;

void core_link_init             (void);                                         // Init UART4, FIFO, and parser
void core_link_task             (void);                                         // Run receive, unpack, and dispatch in main loop
void core_link_uart4_rx_handler (void);                                         // Call from uart4_isr to push rx bytes into FIFO

void core_link_send_frame       (uint8 protocol_id, const uint8 *payload, uint8 length);   // Send generic frame
void core_link_send_text        (const char *text);                             // Send text payload frame
void core_link_send_param_ascii (uint8 command, uint8 item, float value);       // Send ASCII param payload like "R1=1.234"
void core_link_send_drone_status(const core_link_drone_status_packet_struct *status);
void core_link_send_beacon_map_uplink(const core_link_beacon_map_uplink_packet_struct *packet);

void core_link_on_frame_received (uint8 protocol_id, const uint8 *payload, uint8 length);       // Optional generic callback for new protocol ids
void core_link_on_text_received  (const uint8 *text, uint8 length);                              // Optional text callback
void core_link_on_param_received (const core_link_param_packet_struct *param);                   // Optional param callback

#endif
