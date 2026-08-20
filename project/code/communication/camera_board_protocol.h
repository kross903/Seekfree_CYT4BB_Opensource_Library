#ifndef CAMERA_BOARD_PROTOCOL_H_
#define CAMERA_BOARD_PROTOCOL_H_

#include "zf_common_headfile.h"

#define CAMERA_BOARD_UART_INDEX                 (UART_5)
#define CAMERA_BOARD_UART_BAUDRATE              (115200U)
#define CAMERA_BOARD_UART_TX_PIN                (UART5_TX_P02_1)
#define CAMERA_BOARD_UART_RX_PIN                (UART5_RX_P02_0)

#define CAMERA_BOARD_PROTOCOL_SOF1              (0xA5U)
#define CAMERA_BOARD_PROTOCOL_SOF2              (0x5AU)
#define CAMERA_BOARD_PROTOCOL_VERSION           (0x01U)
#define CAMERA_BOARD_PROTOCOL_MAX_PAYLOAD       (96U)
#define CAMERA_BOARD_PROTOCOL_FRAME_OVERHEAD    (10U)
#define CAMERA_BOARD_PROTOCOL_MAX_FRAME         \
    (CAMERA_BOARD_PROTOCOL_MAX_PAYLOAD + CAMERA_BOARD_PROTOCOL_FRAME_OVERHEAD)

#define CAMERA_BOARD_MSG_MAIN_STATE             (0x01U)
#define CAMERA_BOARD_MSG_REAR_RESULT            (0x81U)
#define CAMERA_BOARD_MSG_REAR_STATUS            (0x82U)

#define CAMERA_BOARD_MAIN_STATE_PAYLOAD_LEN     (20U)
#define CAMERA_BOARD_REAR_RESULT_HEADER_LEN     (27U)
#define CAMERA_BOARD_REAR_BEACON_WIRE_LEN       (7U)
#define CAMERA_BOARD_REAR_STATUS_PAYLOAD_LEN    (34U)
#define CAMERA_BOARD_MAX_BEACON_COUNT           (6U)

#define CAMERA_BOARD_MAIN_FLAG_POSE_VALID       (0x0001U)
#define CAMERA_BOARD_MAIN_FLAG_HEIGHT_VALID     (0x0002U)
#define CAMERA_BOARD_MAIN_FLAG_VISION_ENABLE    (0x0004U)
#define CAMERA_BOARD_MAIN_FLAG_BEACON_ENABLE    (0x0008U)
#define CAMERA_BOARD_MAIN_FLAG_CAR_YAW_VALID    (0x0010U)

#define CAMERA_BOARD_RESULT_FLAG_TIME_SYNC      (0x0001U)
#define CAMERA_BOARD_RESULT_FLAG_STATE_FRESH    (0x0002U)
#define CAMERA_BOARD_RESULT_FLAG_CAR_VALID      (0x0004U)
#define CAMERA_BOARD_RESULT_FLAG_CAR_RAW        (0x0008U)
#define CAMERA_BOARD_RESULT_FLAG_BEACON_VALID   (0x0010U)

#define CAMERA_BOARD_STATUS_FLAG_CAMERA_READY   (0x0001U)
#define CAMERA_BOARD_STATUS_FLAG_CAMERA_CORE    (0x0002U)
#define CAMERA_BOARD_STATUS_FLAG_TIME_SYNC      (0x0004U)
#define CAMERA_BOARD_STATUS_FLAG_STATE_FRESH    (0x0008U)
#define CAMERA_BOARD_STATUS_FLAG_EXTRINSIC_NOMINAL (0x0010U)
#define CAMERA_BOARD_STATUS_FLAG_EXTRINSIC_VALID   (0x0020U)

#define CAMERA_BOARD_STATE_PERIOD_US            (5000U)
#define CAMERA_BOARD_STATE_STALE_US             (30000U)
#define CAMERA_BOARD_LINK_TIMEOUT_US            (100000U)
#define CAMERA_BOARD_STATUS_PERIOD_US           (500000U)
#define CAMERA_BOARD_PARSE_TIMEOUT_US            (20000U)

typedef struct
{
    int16 x_mm;
    int16 y_mm;
    uint8 detection_confidence;
    uint8 position_sigma_cm;
    uint8 flags;
} CameraBoardBeaconObservation_t;

typedef struct
{
    uint32 camera_frame_seq;
    uint32 capture_time_us;
    uint16 state_seq_used;
    uint16 reset_generation;
    uint16 result_flags;
    int16 car_x_mm;
    int16 car_y_mm;
    int16 car_raw_x_mm;
    int16 car_raw_y_mm;
    uint16 recognition_time_us;
    uint16 state_age_us;
    uint8 beacon_count;
    CameraBoardBeaconObservation_t beacon[CAMERA_BOARD_MAX_BEACON_COUNT];
} CameraBoardVisionFrame_t;

#endif /* CAMERA_BOARD_PROTOCOL_H_ */
