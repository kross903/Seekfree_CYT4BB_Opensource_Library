#ifndef CAMERA_BOARD_LINK_H_
#define CAMERA_BOARD_LINK_H_

#include "zf_common_headfile.h"
#include "communication/camera_board_protocol.h"

typedef struct
{
    uint8 mode_active;
    uint8 online;
    uint8 rear_camera_ready;
    uint8 rear_camera_core_alive;
    uint8 rear_time_sync_valid;
    uint16 last_result_age_ms;
    uint16 last_status_age_ms;
    uint16 rear_state_age_ms;
    int16 rear_sync_error_us;
    uint32 tx_frame_count;
    uint32 rx_frame_count;
    uint32 crc_error_count;
    uint32 protocol_error_count;
    uint32 sequence_gap_count;
    uint32 rx_overflow_count;
    uint32 tx_busy_drop_count;
    uint32 rear_captured_frame_count;
    uint32 rear_dropped_frame_count;
    uint32 rear_processed_frame_count;
    uint32 rear_ipc_result_count;
    uint16 rear_crc_error_count;
    uint16 rear_sequence_gap_count;
    uint16 rear_rx_overflow_count;
    uint16 rear_tx_busy_drop_count;
} CameraBoardLinkDiag_t;

void CameraBoardLink_MainInit(void);
void CameraBoardLink_MainTask(void);
void CameraBoardLink_RxIsr(void);
uint32 CameraBoardLink_TimeNowUs(void);

uint8 CameraBoardLink_GetRearFrame(CameraBoardVisionFrame_t *frame);
uint8 CameraBoardLink_GetLatestRearFrame(CameraBoardVisionFrame_t *frame);
void CameraBoardLink_GetDiag(CameraBoardLinkDiag_t *diag);
uint8 CameraBoardLink_IsOnline(void);

#endif /* CAMERA_BOARD_LINK_H_ */
