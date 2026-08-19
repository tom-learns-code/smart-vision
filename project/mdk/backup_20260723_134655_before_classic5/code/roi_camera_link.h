#ifndef __ROI_CAMERA_LINK_H
#define __ROI_CAMERA_LINK_H

#include "zf_common_typedef.h"

#define ROI_CAMERA_LINK_BAUD                  (115200U)
#define ROI_CAMERA_LINK_TYPE_PORT_PROBE       (0x31U)

#define ROI_CAMERA_LINK_OBJECT_NONE           (0U)
#define ROI_CAMERA_LINK_OBJECT_BOX            (1U)
#define ROI_CAMERA_LINK_OBJECT_GOAL           (2U)

#define ROI_CAMERA_LINK_STATUS_OK              (0U)
#define ROI_CAMERA_LINK_STATUS_LOW_CONFIDENCE  (1U)
#define ROI_CAMERA_LINK_STATUS_MODEL_MISSING   (2U)
#define ROI_CAMERA_LINK_STATUS_BAD_SESSION     (3U)
#define ROI_CAMERA_LINK_STATUS_BAD_SLOT        (4U)
#define ROI_CAMERA_LINK_STATUS_BUSY            (5U)
#define ROI_CAMERA_LINK_STATUS_BAD_FRAME       (6U)
#define ROI_CAMERA_LINK_STATUS_INTERNAL_ERROR  (7U)
#define ROI_CAMERA_LINK_STATUS_TIMEOUT         (8U)

typedef enum
{
    ROI_CAMERA_LINK_NO_SESSION = 0,
    ROI_CAMERA_LINK_SESSION_WAIT,
    ROI_CAMERA_LINK_READY,
    ROI_CAMERA_LINK_REQUEST_WAIT,
    ROI_CAMERA_LINK_RESULT_READY,
    ROI_CAMERA_LINK_FAULT
} roi_camera_link_state_t;

typedef struct
{
    uint8 valid;
    uint8 session_id;
    uint8 request_id;
    uint8 object_type;
    uint8 object_slot;
    uint8 result_id;
    uint8 confidence_percent;
    uint8 status;
} roi_camera_link_result_t;

typedef struct
{
    uint8 online;
    uint8 camera_uart_id;
    uint8 sequence;
    uint8 protocol_version;
    uint8 state;
    uint8 session_id;
    uint8 session_ready;
    uint8 box_count;
    uint8 goal_count;
    uint8 pending_request_id;
    uint8 pending_object_type;
    uint8 pending_object_slot;
    uint8 last_status;
    uint8 session_retries;
    uint8 request_retries;
    uint32 fp_build_id;
    uint32 rx_bytes;
    uint32 rx_packets;
    uint32 rx_protocol_packets;
    uint32 tx_packets;
    uint32 matched_results;
    uint32 stale_results;
    uint32 checksum_errors;
    uint32 format_errors;
    uint32 ring_drops;
    uint32 timeouts;
    uint32 last_packet_tick;
    uint32 transaction_start_tick;
    roi_camera_link_result_t last_result;
} roi_camera_link_snapshot_t;

void roi_camera_link_init(void);
void roi_camera_link_poll(void);
void roi_camera_link_rx_byte(uint8 value);
void roi_camera_link_get_snapshot(roi_camera_link_snapshot_t *out);
uint8 roi_camera_link_begin_session(uint8 box_count, uint8 goal_count);
uint8 roi_camera_link_request(uint8 object_type, uint8 object_slot);
uint8 roi_camera_link_take_result(roi_camera_link_result_t *out);
void roi_camera_link_reset_session(void);
uint8 roi_camera_link_session_ready(void);
uint8 roi_camera_link_busy(void);
const char *roi_camera_link_state_name(uint8 state);
const char *roi_camera_link_status_name(uint8 status);

#endif
