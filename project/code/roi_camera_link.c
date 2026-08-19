#include <string.h>
#include "zf_common_headfile.h"
#include "motion_control.h"
#include "roi_camera_link.h"
#include "app_config.h"

#define ROI_CAMERA_LINK_UART_INDEX          (UART_1)
#define ROI_CAMERA_LINK_UART_TX_PIN         (UART1_TX_B12)
#define ROI_CAMERA_LINK_UART_RX_PIN         (UART1_RX_B13)
#define ROI_CAMERA_LINK_HEADER_0            (0xA5U)
#define ROI_CAMERA_LINK_HEADER_1            (0x5AU)
#define ROI_CAMERA_LINK_FP2_DOMAIN          (0xF2U)
#define ROI_CAMERA_LINK_FP2_VERSION         (0x01U)
#define ROI_CAMERA_LINK_PROBE_SIZE          (10U)
#define ROI_CAMERA_LINK_FP2_SIZE            (20U)
#define ROI_CAMERA_LINK_MAX_PACKET_SIZE     (20U)
#define ROI_CAMERA_LINK_RX_RING_SIZE        (256U)
#define ROI_CAMERA_LINK_RX_RING_MASK        (ROI_CAMERA_LINK_RX_RING_SIZE - 1U)
#define ROI_CAMERA_LINK_PARSE_LIMIT         (128U)
#define ROI_CAMERA_LINK_ONLINE_TICKS        (1000U) /* 5 s at 5 ms/tick. */
#define ROI_CAMERA_LINK_TIMEOUT_TICKS       (APP_FP2_RESULT_TIMEOUT_MS / 5U)
#define ROI_CAMERA_LINK_INIT_RETRY_TICKS    (100U)  /* 500 ms. */
#define ROI_CAMERA_LINK_INIT_MAX_RETRIES    (3U)
#define ROI_CAMERA_LINK_REQUEST_RETRY_TICKS (APP_FP2_REQUEST_RETRY_MS / 5U)
#define ROI_CAMERA_LINK_REQUEST_MAX_RETRIES (APP_FP2_REQUEST_MAX_RETRIES)
#define ROI_CAMERA_LINK_MCU_BUILD_ID        (APP_MCU_BUILD_ID)

#define ROI_CAMERA_LINK_FP2_SESSION_INIT    (0x40U)
#define ROI_CAMERA_LINK_FP2_INIT_ACK        (0x41U)
#define ROI_CAMERA_LINK_FP2_IMAGE_REQUEST   (0x42U)
#define ROI_CAMERA_LINK_FP2_DIGIT_REQUEST   (0x43U)
#define ROI_CAMERA_LINK_FP2_RESULT          (0x44U)
#define ROI_CAMERA_LINK_FP2_RESET_SESSION   (0x45U)

static volatile roi_camera_link_snapshot_t g_roi_camera;
static volatile uint8 g_rx_ring[ROI_CAMERA_LINK_RX_RING_SIZE];
static volatile uint16 g_rx_head;
static volatile uint16 g_rx_tail;
static uint8 g_packet[ROI_CAMERA_LINK_MAX_PACKET_SIZE];
static uint8 g_packet_index;
static uint8 g_packet_expected;
static uint8 g_packet_seen;
static uint8 g_tx_sequence;
static uint8 g_next_session_id;
static uint8 g_next_request_id;
static uint32 g_last_init_tx_tick;
static uint32 g_last_request_tx_tick;

static uint8 roi_camera_link_xor(const uint8 *data, uint8 length)
{
    uint8 value = 0U;
    uint8 index;

    for(index = 0U; index < length; index++) value ^= data[index];
    return value;
}

static uint32 roi_camera_link_u32_le(const uint8 *data)
{
    return (uint32)data[0] |
           ((uint32)data[1] << 8) |
           ((uint32)data[2] << 16) |
           ((uint32)data[3] << 24);
}

static void roi_camera_link_put_u32_le(uint8 *data, uint32 value)
{
    data[0] = (uint8)(value & 0xFFU);
    data[1] = (uint8)((value >> 8) & 0xFFU);
    data[2] = (uint8)((value >> 16) & 0xFFU);
    data[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint8 roi_camera_link_ring_read(uint8 *value)
{
    uint16 tail = g_rx_tail;

    if(tail == g_rx_head) return 0U;
    *value = g_rx_ring[tail];
    g_rx_tail = (uint16)((tail + 1U) & ROI_CAMERA_LINK_RX_RING_MASK);
    return 1U;
}

static void roi_camera_link_send_frame(uint8 frame_type, uint8 session_id,
                                       uint8 request_id, uint8 object_type,
                                       uint8 object_slot, uint8 arg0,
                                       uint8 arg1, uint8 status)
{
    uint8 packet[ROI_CAMERA_LINK_FP2_SIZE];

    memset(packet, 0, sizeof(packet));
    packet[0] = ROI_CAMERA_LINK_HEADER_0;
    packet[1] = ROI_CAMERA_LINK_HEADER_1;
    packet[2] = ROI_CAMERA_LINK_FP2_DOMAIN;
    packet[3] = ROI_CAMERA_LINK_FP2_VERSION;
    packet[4] = frame_type;
    packet[5] = session_id;
    packet[6] = request_id;
    packet[7] = object_type;
    packet[8] = object_slot;
    packet[9] = arg0;
    packet[10] = arg1;
    packet[11] = status;
    packet[12] = ++g_tx_sequence;
    roi_camera_link_put_u32_le(&packet[13], ROI_CAMERA_LINK_MCU_BUILD_ID);
    packet[19] = roi_camera_link_xor(packet, 19U);
    uart_write_buffer(ROI_CAMERA_LINK_UART_INDEX, packet, sizeof(packet));
    g_roi_camera.tx_packets++;
}

static void roi_camera_link_send_session_init(void)
{
    roi_camera_link_send_frame(ROI_CAMERA_LINK_FP2_SESSION_INIT,
                               g_roi_camera.session_id, 0U,
                               ROI_CAMERA_LINK_OBJECT_NONE, 0xFFU,
                               g_roi_camera.box_count,
                               g_roi_camera.goal_count,
                               ROI_CAMERA_LINK_STATUS_OK);
    g_last_init_tx_tick = pit_count;
}

static void roi_camera_link_handle_probe(void)
{
    if(roi_camera_link_xor(g_packet, ROI_CAMERA_LINK_PROBE_SIZE - 1U) !=
       g_packet[ROI_CAMERA_LINK_PROBE_SIZE - 1U])
    {
        g_roi_camera.checksum_errors++;
        return;
    }
    g_roi_camera.camera_uart_id = g_packet[3];
    g_roi_camera.sequence = g_packet[4];
    g_roi_camera.fp_build_id = roi_camera_link_u32_le(&g_packet[5]);
    g_roi_camera.last_packet_tick = pit_count;
    g_roi_camera.rx_packets++;
    g_roi_camera.online = 1U;
    g_packet_seen = 1U;
}

static void roi_camera_link_handle_fp2(void)
{
    uint8 frame_type;
    uint8 session_id;
    uint8 request_id;

    if(g_packet[3] != ROI_CAMERA_LINK_FP2_VERSION)
    {
        g_roi_camera.format_errors++;
        return;
    }
    if(roi_camera_link_xor(g_packet, ROI_CAMERA_LINK_FP2_SIZE - 1U) !=
       g_packet[ROI_CAMERA_LINK_FP2_SIZE - 1U])
    {
        g_roi_camera.checksum_errors++;
        return;
    }

    frame_type = g_packet[4];
    session_id = g_packet[5];
    request_id = g_packet[6];
    g_roi_camera.protocol_version = g_packet[3];
    g_roi_camera.sequence = g_packet[12];
    g_roi_camera.fp_build_id = roi_camera_link_u32_le(&g_packet[13]);
    g_roi_camera.last_packet_tick = pit_count;
    g_roi_camera.rx_packets++;
    g_roi_camera.rx_protocol_packets++;
    g_roi_camera.online = 1U;
    g_packet_seen = 1U;

    if(frame_type == ROI_CAMERA_LINK_FP2_INIT_ACK)
    {
        if(g_roi_camera.state != ROI_CAMERA_LINK_SESSION_WAIT ||
           session_id != g_roi_camera.session_id)
        {
            g_roi_camera.stale_results++;
            return;
        }
        g_roi_camera.last_status = g_packet[11];
        if(g_packet[11] == ROI_CAMERA_LINK_STATUS_OK &&
           g_packet[9] == g_roi_camera.box_count &&
           g_packet[10] == g_roi_camera.goal_count)
        {
            g_roi_camera.session_ready = 1U;
            g_roi_camera.state = ROI_CAMERA_LINK_READY;
        }
        else
        {
            g_roi_camera.session_ready = 0U;
            g_roi_camera.state = ROI_CAMERA_LINK_FAULT;
        }
        return;
    }

    if(frame_type != ROI_CAMERA_LINK_FP2_RESULT)
    {
        return;
    }
    if(g_roi_camera.state != ROI_CAMERA_LINK_REQUEST_WAIT ||
       session_id != g_roi_camera.session_id ||
       request_id != g_roi_camera.pending_request_id ||
       g_packet[7] != g_roi_camera.pending_object_type ||
       g_packet[8] != g_roi_camera.pending_object_slot)
    {
        g_roi_camera.stale_results++;
        return;
    }

    g_roi_camera.last_result.valid = 1U;
    g_roi_camera.last_result.session_id = session_id;
    g_roi_camera.last_result.request_id = request_id;
    g_roi_camera.last_result.object_type = g_packet[7];
    g_roi_camera.last_result.object_slot = g_packet[8];
    g_roi_camera.last_result.result_id = g_packet[9];
    g_roi_camera.last_result.confidence_percent = g_packet[10];
    g_roi_camera.last_result.status = g_packet[11];
    g_roi_camera.last_status = g_packet[11];
    g_roi_camera.matched_results++;
    g_roi_camera.state = ROI_CAMERA_LINK_RESULT_READY;
}

static void roi_camera_link_parse_byte(uint8 value)
{
    if(g_packet_index == 0U)
    {
        if(value == ROI_CAMERA_LINK_HEADER_0)
        {
            g_packet[0] = value;
            g_packet_index = 1U;
        }
        return;
    }
    if(g_packet_index == 1U)
    {
        if(value == ROI_CAMERA_LINK_HEADER_1)
        {
            g_packet[1] = value;
            g_packet_index = 2U;
        }
        else if(value == ROI_CAMERA_LINK_HEADER_0)
        {
            g_packet[0] = value;
        }
        else
        {
            g_packet_index = 0U;
        }
        return;
    }

    g_packet[g_packet_index++] = value;
    if(g_packet_index == 3U)
    {
        if(g_packet[2] == ROI_CAMERA_LINK_TYPE_PORT_PROBE)
            g_packet_expected = ROI_CAMERA_LINK_PROBE_SIZE;
        else if(g_packet[2] == ROI_CAMERA_LINK_FP2_DOMAIN)
            g_packet_expected = ROI_CAMERA_LINK_FP2_SIZE;
        else
        {
            g_roi_camera.format_errors++;
            g_packet_index = 0U;
            g_packet_expected = 0U;
            return;
        }
    }
    if(g_packet_expected == 0U || g_packet_index < g_packet_expected) return;

    g_packet_index = 0U;
    if(g_packet_expected == ROI_CAMERA_LINK_PROBE_SIZE)
        roi_camera_link_handle_probe();
    else
        roi_camera_link_handle_fp2();
    g_packet_expected = 0U;
}

void roi_camera_link_init(void)
{
    memset((void *)&g_roi_camera, 0, sizeof(g_roi_camera));
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_packet_index = 0U;
    g_packet_expected = 0U;
    g_packet_seen = 0U;
    g_tx_sequence = 0U;
    g_next_session_id = 0U;
    g_next_request_id = 0U;
    g_last_init_tx_tick = 0U;
    g_last_request_tx_tick = 0U;
    g_roi_camera.state = ROI_CAMERA_LINK_NO_SESSION;

    uart_init(ROI_CAMERA_LINK_UART_INDEX,
              ROI_CAMERA_LINK_BAUD,
              ROI_CAMERA_LINK_UART_TX_PIN,
              ROI_CAMERA_LINK_UART_RX_PIN);
    uart_rx_interrupt(ROI_CAMERA_LINK_UART_INDEX, 1U);
}

void roi_camera_link_rx_byte(uint8 value)
{
    uint16 next;

    g_roi_camera.rx_bytes++;
    next = (uint16)((g_rx_head + 1U) & ROI_CAMERA_LINK_RX_RING_MASK);
    if(next == g_rx_tail)
    {
        g_roi_camera.ring_drops++;
        return;
    }
    g_rx_ring[g_rx_head] = value;
    g_rx_head = next;
}

void roi_camera_link_poll(void)
{
    uint8 value;
    uint16 limit = ROI_CAMERA_LINK_PARSE_LIMIT;
    uint32 elapsed;

    while(limit-- > 0U && roi_camera_link_ring_read(&value))
        roi_camera_link_parse_byte(value);

    if(g_packet_seen &&
       (uint32)(pit_count - g_roi_camera.last_packet_tick) >
           ROI_CAMERA_LINK_ONLINE_TICKS)
        g_roi_camera.online = 0U;

    if(g_roi_camera.state == ROI_CAMERA_LINK_SESSION_WAIT)
    {
        elapsed = (uint32)(pit_count - g_roi_camera.transaction_start_tick);
        if(elapsed > ROI_CAMERA_LINK_TIMEOUT_TICKS)
        {
            g_roi_camera.timeouts++;
            g_roi_camera.last_status = ROI_CAMERA_LINK_STATUS_TIMEOUT;
            g_roi_camera.state = ROI_CAMERA_LINK_FAULT;
        }
        else if((uint32)(pit_count - g_last_init_tx_tick) >=
                    ROI_CAMERA_LINK_INIT_RETRY_TICKS &&
                g_roi_camera.session_retries <
                    ROI_CAMERA_LINK_INIT_MAX_RETRIES)
        {
            g_roi_camera.session_retries++;
            roi_camera_link_send_session_init();
        }
    }
    else if(g_roi_camera.state == ROI_CAMERA_LINK_REQUEST_WAIT)
    {
        elapsed = (uint32)(pit_count - g_roi_camera.transaction_start_tick);
        if(elapsed > ROI_CAMERA_LINK_TIMEOUT_TICKS)
        {
            g_roi_camera.timeouts++;
            g_roi_camera.last_status = ROI_CAMERA_LINK_STATUS_TIMEOUT;
            memset((void *)&g_roi_camera.last_result, 0,
                   sizeof(g_roi_camera.last_result));
            g_roi_camera.last_result.valid = 1U;
            g_roi_camera.last_result.session_id = g_roi_camera.session_id;
            g_roi_camera.last_result.request_id =
                g_roi_camera.pending_request_id;
            g_roi_camera.last_result.object_type =
                g_roi_camera.pending_object_type;
            g_roi_camera.last_result.object_slot =
                g_roi_camera.pending_object_slot;
            g_roi_camera.last_result.result_id = 0xFFU;
            g_roi_camera.last_result.status = ROI_CAMERA_LINK_STATUS_TIMEOUT;
            g_roi_camera.state = ROI_CAMERA_LINK_RESULT_READY;
        }
        else if((uint32)(pit_count - g_last_request_tx_tick) >=
                    ROI_CAMERA_LINK_REQUEST_RETRY_TICKS &&
                g_roi_camera.request_retries <
                    ROI_CAMERA_LINK_REQUEST_MAX_RETRIES)
        {
            uint8 frame_type =
                g_roi_camera.pending_object_type == ROI_CAMERA_LINK_OBJECT_BOX ?
                    ROI_CAMERA_LINK_FP2_IMAGE_REQUEST :
                    ROI_CAMERA_LINK_FP2_DIGIT_REQUEST;

            g_roi_camera.request_retries++;
            roi_camera_link_send_frame(frame_type,
                                       g_roi_camera.session_id,
                                       g_roi_camera.pending_request_id,
                                       g_roi_camera.pending_object_type,
                                       g_roi_camera.pending_object_slot,
                                       0U, 0U,
                                       ROI_CAMERA_LINK_STATUS_OK);
            g_last_request_tx_tick = pit_count;
        }
    }
}

void roi_camera_link_get_snapshot(roi_camera_link_snapshot_t *out)
{
    if(out == 0) return;
    memcpy(out, (const void *)&g_roi_camera, sizeof(*out));
}

uint8 roi_camera_link_begin_session(uint8 box_count, uint8 goal_count)
{
    if(box_count < 1U || box_count > 3U || goal_count != box_count ||
       roi_camera_link_busy()) return 0U;

    g_next_session_id++;
    if(g_next_session_id == 0U) g_next_session_id = 1U;
    g_roi_camera.session_id = g_next_session_id;
    g_roi_camera.box_count = box_count;
    g_roi_camera.goal_count = goal_count;
    g_roi_camera.session_ready = 0U;
    g_roi_camera.session_retries = 0U;
    g_roi_camera.last_status = ROI_CAMERA_LINK_STATUS_OK;
    memset((void *)&g_roi_camera.last_result, 0,
           sizeof(g_roi_camera.last_result));
    g_roi_camera.transaction_start_tick = pit_count;
    g_roi_camera.state = ROI_CAMERA_LINK_SESSION_WAIT;
    roi_camera_link_send_session_init();
    return 1U;
}

uint8 roi_camera_link_request(uint8 object_type, uint8 object_slot)
{
    uint8 frame_type;
    uint8 count;

    if(g_roi_camera.state != ROI_CAMERA_LINK_READY ||
       !g_roi_camera.session_ready) return 0U;
    if(object_type == ROI_CAMERA_LINK_OBJECT_BOX)
    {
        frame_type = ROI_CAMERA_LINK_FP2_IMAGE_REQUEST;
        count = g_roi_camera.box_count;
    }
    else if(object_type == ROI_CAMERA_LINK_OBJECT_GOAL)
    {
        frame_type = ROI_CAMERA_LINK_FP2_DIGIT_REQUEST;
        count = g_roi_camera.goal_count;
    }
    else
    {
        return 0U;
    }
    if(object_slot >= count) return 0U;

    g_next_request_id++;
    if(g_next_request_id == 0U) g_next_request_id = 1U;
    g_roi_camera.pending_request_id = g_next_request_id;
    g_roi_camera.pending_object_type = object_type;
    g_roi_camera.pending_object_slot = object_slot;
    g_roi_camera.request_retries = 0U;
    g_roi_camera.last_status = ROI_CAMERA_LINK_STATUS_OK;
    memset((void *)&g_roi_camera.last_result, 0,
           sizeof(g_roi_camera.last_result));
    g_roi_camera.transaction_start_tick = pit_count;
    g_roi_camera.state = ROI_CAMERA_LINK_REQUEST_WAIT;
    roi_camera_link_send_frame(frame_type, g_roi_camera.session_id,
                               g_roi_camera.pending_request_id,
                               object_type, object_slot, 0U, 0U,
                               ROI_CAMERA_LINK_STATUS_OK);
    g_last_request_tx_tick = pit_count;
    return 1U;
}

uint8 roi_camera_link_take_result(roi_camera_link_result_t *out)
{
    if(out == 0 || g_roi_camera.state != ROI_CAMERA_LINK_RESULT_READY ||
       !g_roi_camera.last_result.valid) return 0U;
    memcpy(out, (const void *)&g_roi_camera.last_result, sizeof(*out));
    memset((void *)&g_roi_camera.last_result, 0,
           sizeof(g_roi_camera.last_result));
    g_roi_camera.state = g_roi_camera.session_ready ?
        ROI_CAMERA_LINK_READY : ROI_CAMERA_LINK_FAULT;
    return 1U;
}

void roi_camera_link_reset_session(void)
{
    if(g_roi_camera.session_id != 0U)
        roi_camera_link_send_frame(ROI_CAMERA_LINK_FP2_RESET_SESSION,
                                   g_roi_camera.session_id, 0U,
                                   ROI_CAMERA_LINK_OBJECT_NONE, 0xFFU,
                                   0U, 0U, ROI_CAMERA_LINK_STATUS_OK);
    g_roi_camera.session_id = 0U;
    g_roi_camera.session_ready = 0U;
    g_roi_camera.box_count = 0U;
    g_roi_camera.goal_count = 0U;
    g_roi_camera.pending_request_id = 0U;
    g_roi_camera.request_retries = 0U;
    memset((void *)&g_roi_camera.last_result, 0,
           sizeof(g_roi_camera.last_result));
    g_roi_camera.state = ROI_CAMERA_LINK_NO_SESSION;
}

uint8 roi_camera_link_session_ready(void)
{
    return (uint8)(g_roi_camera.session_ready &&
                   g_roi_camera.state == ROI_CAMERA_LINK_READY);
}

uint8 roi_camera_link_busy(void)
{
    return (uint8)(g_roi_camera.state == ROI_CAMERA_LINK_SESSION_WAIT ||
                   g_roi_camera.state == ROI_CAMERA_LINK_REQUEST_WAIT);
}

const char *roi_camera_link_state_name(uint8 state)
{
    switch((roi_camera_link_state_t)state)
    {
        case ROI_CAMERA_LINK_NO_SESSION: return "NO_SESSION";
        case ROI_CAMERA_LINK_SESSION_WAIT: return "SESSION_WAIT";
        case ROI_CAMERA_LINK_READY: return "READY";
        case ROI_CAMERA_LINK_REQUEST_WAIT: return "REQUEST_WAIT";
        case ROI_CAMERA_LINK_RESULT_READY: return "RESULT_READY";
        case ROI_CAMERA_LINK_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char *roi_camera_link_status_name(uint8 status)
{
    switch(status)
    {
        case ROI_CAMERA_LINK_STATUS_OK: return "OK";
        case ROI_CAMERA_LINK_STATUS_LOW_CONFIDENCE: return "LOW_CONFIDENCE";
        case ROI_CAMERA_LINK_STATUS_MODEL_MISSING: return "MODEL_UNAVAILABLE";
        case ROI_CAMERA_LINK_STATUS_BAD_SESSION: return "BAD_SESSION";
        case ROI_CAMERA_LINK_STATUS_BAD_SLOT: return "BAD_SLOT";
        case ROI_CAMERA_LINK_STATUS_BUSY: return "BUSY";
        case ROI_CAMERA_LINK_STATUS_BAD_FRAME: return "BAD_FRAME";
        case ROI_CAMERA_LINK_STATUS_INTERNAL_ERROR: return "INTERNAL_ERROR";
        case ROI_CAMERA_LINK_STATUS_TIMEOUT: return "TIMEOUT";
        default: return "UNKNOWN";
    }
}
