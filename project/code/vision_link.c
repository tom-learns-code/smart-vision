#include <stdio.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "motion_control.h"
#include "vision_link.h"
#include "app_config.h"

#define VISION_LINK_HEADER0            (0xA5U)
#define VISION_LINK_HEADER1            (0x5AU)
#define VISION_LINK_POS_MAX_BOXES      (5U)
#define VISION_LINK_POS_MAX_BOMBS      (4U)
#define VISION_LINK_FULL_V1_PAYLOAD_LEN (60U)
#define VISION_LINK_FULL_BOX_COUNT     (26U)
#define VISION_LINK_FULL_BOX_DATA      (VISION_LINK_FULL_BOX_COUNT + 1U)
#define VISION_LINK_FULL_GOAL_COUNT    (VISION_LINK_FULL_BOX_DATA + VISION_LINK_MAX_BOXES * 2U)
#define VISION_LINK_FULL_GOAL_DATA     (VISION_LINK_FULL_GOAL_COUNT + 1U)
#define VISION_LINK_FULL_BOMB_COUNT    (VISION_LINK_FULL_GOAL_DATA + VISION_LINK_MAX_GOALS * 2U)
#define VISION_LINK_FULL_BOMB_DATA     (VISION_LINK_FULL_BOMB_COUNT + 1U)
#define VISION_LINK_FULL_POSE_DATA     (VISION_LINK_FULL_BOMB_DATA + VISION_LINK_MAX_BOMBS * 2U)
#define VISION_LINK_FULL_PAYLOAD_LEN   (VISION_LINK_FULL_POSE_DATA + 6U + 1U)
#define VISION_LINK_POS_PAYLOAD_LEN    (26U)
#define VISION_LINK_PAYLOAD_MAX        (64U)
#define VISION_LINK_RX_BUF_SIZE        (512U)
#define VISION_LINK_RX_PARSE_LIMIT     (64U)
#define VISION_LINK_ONLINE_TIMEOUT     (APP_GLOBAL_VISION_TIMEOUT_MS / 5U)
#define VISION_LINK_REQ_TIMEOUT        (400U)     /* 400 * 5ms = 2s */
#define VISION_LINK_REQ_MAX_RETRY      (3U)
#define VISION_LINK_REQ_FAST_PERIOD    (100U)     /* 100 * 5ms = 500ms before first map */
#define VISION_LINK_REQ_PERIOD         (1000U)    /* 1000 * 5ms = 5s after map is valid */
#define VISION_LINK_REQ_INIT           (0x01U)
#define VISION_LINK_UART_INDEX         (UART_4)
#define VISION_LINK_UART_TX_PIN        (UART4_TX_C16)
#define VISION_LINK_UART_RX_PIN        (UART4_RX_C17)
#define VISION_LINK_USE_UART_POLL      (0U)

#define RX_WAIT_H0                     (0U)
#define RX_WAIT_H1                     (1U)
#define RX_WAIT_TYPE                   (2U)
#define RX_PAYLOAD                     (3U)

static volatile vision_link_snapshot_t g_vis;
static vision_link_map_t g_full_map_cache;
static uint8 rx_state = RX_WAIT_H0;
static uint8 rx_type = 0;
static uint8 rx_payload[VISION_LINK_PAYLOAD_MAX];
static uint8 rx_len = 0;
static uint8 rx_expected = 0;

static uint8 ring_buf[VISION_LINK_RX_BUF_SIZE];
static volatile uint16 ring_write_idx = 0;
static volatile uint16 ring_read_idx = 0;

static uint8 request_pending = 0;
static uint8 request_retry_count = 0;
static uint16 request_timeout_tick = 0;
static uint8 request_reason = VISION_LINK_REQ_INIT;
static uint32 request_last_tick = 0;

static void vision_link_parse_byte(uint8 dat);

static void vision_link_clear_cells(volatile vision_link_cell_t *cells, uint8 count)
{
    uint8 i;

    for(i = 0; i < count; i++)
    {
        cells[i].gx = -1;
        cells[i].gy = -1;
    }
}

static void vision_link_copy_cells(volatile vision_link_cell_t *cells, uint8 max_count, uint8 count, uint8 offset)
{
    uint8 i;

    vision_link_clear_cells(cells, max_count);
    if(count > max_count)
    {
        count = max_count;
    }

    for(i = 0; i < count; i++)
    {
        cells[i].gx = (int8)rx_payload[offset + i * 2U];
        cells[i].gy = (int8)rx_payload[offset + i * 2U + 1U];
    }
}

static uint8 vision_link_xor(const uint8 *data, uint8 len)
{
    uint8 value = 0;
    uint8 i;

    for(i = 0; i < len; i++)
    {
        value ^= data[i];
    }

    return value;
}

static int16 vision_link_i16_le(const uint8 *data)
{
    return (int16)((uint16)data[0] | ((uint16)data[1] << 8));
}

static uint8 vision_link_popcount8(uint8 value)
{
    uint8 count = 0;

    while(value)
    {
        count += (uint8)(value & 1U);
        value >>= 1;
    }

    return count;
}

static void vision_link_parse_full_map(void)
{
    uint8 i;
    uint8 walls = 0;
    uint8 count;
    uint8 payload_len;
    uint8 box_count_offset;
    uint8 box_data_offset;
    uint8 goal_count_offset;
    uint8 goal_data_offset;
    uint8 bomb_count_offset;
    uint8 bomb_data_offset;

    if(rx_type == VISION_LINK_TYPE_FULL_MAP_V2)
    {
        payload_len = VISION_LINK_FULL_PAYLOAD_LEN;
        box_count_offset = VISION_LINK_FULL_BOX_COUNT;
        box_data_offset = VISION_LINK_FULL_BOX_DATA;
        goal_count_offset = VISION_LINK_FULL_GOAL_COUNT;
        goal_data_offset = VISION_LINK_FULL_GOAL_DATA;
        bomb_count_offset = VISION_LINK_FULL_BOMB_COUNT;
        bomb_data_offset = VISION_LINK_FULL_BOMB_DATA;
    }
    else
    {
        payload_len = VISION_LINK_FULL_V1_PAYLOAD_LEN;
        box_count_offset = 26U;
        box_data_offset = 27U;
        goal_count_offset = 35U;
        goal_data_offset = 36U;
        bomb_count_offset = 44U;
        bomb_data_offset = 45U;
    }

    if(vision_link_xor(rx_payload, payload_len - 1U) !=
       rx_payload[payload_len - 1U])
    {
        g_vis.checksum_errors++;
        return;
    }

    g_vis.map_width = rx_payload[0];
    g_vis.map_height = rx_payload[1];

    for(i = 0U; i < VISION_LINK_WALL_BYTES; i++)
    {
        g_vis.wall_bits[i] = rx_payload[2U + i];
    }

    for(i = 2U; i < 26U; i++)
    {
        walls += vision_link_popcount8(rx_payload[i]);
    }

    g_vis.wall_count = walls;
    count = rx_payload[box_count_offset];
    g_vis.box_count = (count > VISION_LINK_MAX_BOXES) ? VISION_LINK_MAX_BOXES : count;
    vision_link_copy_cells(g_vis.boxes, VISION_LINK_MAX_BOXES,
                           g_vis.box_count, box_data_offset);

    count = rx_payload[goal_count_offset];
    g_vis.goal_count = (count > VISION_LINK_MAX_GOALS) ? VISION_LINK_MAX_GOALS : count;
    vision_link_copy_cells(g_vis.goals, VISION_LINK_MAX_GOALS,
                           g_vis.goal_count, goal_data_offset);

    count = rx_payload[bomb_count_offset];
    g_vis.bomb_count = (count > VISION_LINK_MAX_BOMBS) ? VISION_LINK_MAX_BOMBS : count;
    vision_link_copy_cells(g_vis.bombs, VISION_LINK_MAX_BOMBS,
                           g_vis.bomb_count, bomb_data_offset);

    /* Position state is owned by POS_UPDATE, which also carries pose_valid.
       A FULL_MAP must not overwrite it with retained coordinates. */
    g_vis.last_type = rx_type;
    g_vis.last_packet_tick = pit_count;
    g_vis.map_packets++;
    g_vis.map_version++;
    g_vis.map_valid = 1U;

    g_full_map_cache.valid = 1U;
    g_full_map_cache.map_version = g_vis.map_version;
    g_full_map_cache.width = g_vis.map_width;
    g_full_map_cache.height = g_vis.map_height;
    g_full_map_cache.box_count = g_vis.box_count;
    g_full_map_cache.goal_count = g_vis.goal_count;
    g_full_map_cache.bomb_count = g_vis.bomb_count;
    for(i = 0U; i < VISION_LINK_WALL_BYTES; i++)
    {
        g_full_map_cache.wall_bits[i] = g_vis.wall_bits[i];
    }
    for(i = 0U; i < VISION_LINK_MAX_BOXES; i++)
    {
        g_full_map_cache.boxes[i].gx = g_vis.boxes[i].gx;
        g_full_map_cache.boxes[i].gy = g_vis.boxes[i].gy;
    }
    for(i = 0U; i < VISION_LINK_MAX_GOALS; i++)
    {
        g_full_map_cache.goals[i].gx = g_vis.goals[i].gx;
        g_full_map_cache.goals[i].gy = g_vis.goals[i].gy;
    }
    for(i = 0U; i < VISION_LINK_MAX_BOMBS; i++)
    {
        g_full_map_cache.bombs[i].gx = g_vis.bombs[i].gx;
        g_full_map_cache.bombs[i].gy = g_vis.bombs[i].gy;
    }
    request_pending = 0U;
    request_retry_count = 0U;
    request_timeout_tick = 0U;
}

static void vision_link_parse_pos_update(void)
{
    uint8 bomb_index;
    uint8 bomb_capacity;
    uint8 box_count;
    uint8 bomb_count;

    if(rx_len != VISION_LINK_POS_PAYLOAD_LEN)
    {
        g_vis.format_errors++;
        return;
    }

    if(vision_link_xor(rx_payload, rx_len - 1U) != rx_payload[rx_len - 1U])
    {
        g_vis.checksum_errors++;
        return;
    }

    box_count = rx_payload[7];
    if(box_count > VISION_LINK_POS_MAX_BOXES)
    {
        box_count = VISION_LINK_POS_MAX_BOXES;
    }
    vision_link_copy_cells(g_vis.boxes, VISION_LINK_MAX_BOXES, box_count, 8U);

    bomb_index = (uint8)(8U + box_count * 2U);
    if(bomb_index >= (VISION_LINK_POS_PAYLOAD_LEN - 1U))
    {
        g_vis.format_errors++;
        return;
    }

    bomb_count = rx_payload[bomb_index];
    bomb_capacity = (uint8)((VISION_LINK_POS_PAYLOAD_LEN - 1U -
                            (bomb_index + 1U)) / 2U);
    if(bomb_count > VISION_LINK_POS_MAX_BOMBS)
    {
        bomb_count = VISION_LINK_POS_MAX_BOMBS;
    }
    if(bomb_count > bomb_capacity)
    {
        bomb_count = bomb_capacity;
    }
    vision_link_copy_cells(g_vis.bombs, VISION_LINK_MAX_BOMBS, bomb_count, (uint8)(bomb_index + 1U));

    g_vis.pose_valid = (rx_payload[0] & 0x80U) ? 1U : 0U;
    g_vis.frame_id = rx_payload[0] & 0x7FU;
    g_vis.car_x_mm = vision_link_i16_le(&rx_payload[1]);
    g_vis.car_y_mm = vision_link_i16_le(&rx_payload[3]);
    g_vis.car_theta_x10 = vision_link_i16_le(&rx_payload[5]);
    g_vis.box_count = box_count;
    g_vis.bomb_count = bomb_count;
    g_vis.last_type = VISION_LINK_TYPE_POS_UPDATE;
    g_vis.last_packet_tick = pit_count;
    g_vis.pos_packets++;
    request_timeout_tick = 0;
}

static void vision_link_finish_packet(void)
{
    if(rx_type == VISION_LINK_TYPE_FULL_MAP ||
       rx_type == VISION_LINK_TYPE_FULL_MAP_V2)
    {
        vision_link_parse_full_map();
    }
    else if(rx_type == VISION_LINK_TYPE_POS_UPDATE)
    {
        vision_link_parse_pos_update();
    }
    else
    {
        g_vis.format_errors++;
    }
}

static void vision_link_send_map_request(uint8 reason)
{
    uint8 pkt[5];

    pkt[0] = VISION_LINK_HEADER0;
    pkt[1] = VISION_LINK_HEADER1;
    pkt[2] = VISION_LINK_TYPE_MAP_REQUEST;
    pkt[3] = reason;
    pkt[4] = (uint8)(pkt[0] ^ pkt[1] ^ pkt[2] ^ pkt[3]);
    uart_write_buffer(VISION_LINK_UART_INDEX, pkt, sizeof(pkt));
    g_vis.map_request_count++;
}

void vision_link_request_full_map(void)
{
    request_reason = VISION_LINK_REQ_INIT;
    request_pending = 1U;
    request_retry_count = 0U;
    request_timeout_tick = 0U;
    request_last_tick = pit_count;
    vision_link_send_map_request(request_reason);
}

void vision_link_init(void)
{
    memset(&g_full_map_cache, 0, sizeof(g_full_map_cache));
    ring_write_idx = 0U;
    ring_read_idx = 0U;
    rx_state = RX_WAIT_H0;
    rx_type = 0U;
    rx_len = 0U;
    rx_expected = 0U;
    request_pending = 0U;
    request_retry_count = 0U;
    request_timeout_tick = 0U;
    request_last_tick = pit_count;
    vision_link_clear_cells(g_vis.boxes, VISION_LINK_MAX_BOXES);
    vision_link_clear_cells(g_vis.goals, VISION_LINK_MAX_GOALS);
    vision_link_clear_cells(g_vis.bombs, VISION_LINK_MAX_BOMBS);

    uart_init(VISION_LINK_UART_INDEX, VISION_LINK_BAUD, VISION_LINK_UART_TX_PIN, VISION_LINK_UART_RX_PIN);
    uart_rx_interrupt(VISION_LINK_UART_INDEX, 1);
    vision_link_request_full_map();
}

static uint8 vision_link_ring_read(uint8 *dat)
{
    if(ring_read_idx == ring_write_idx)
    {
        return 0U;
    }

    *dat = ring_buf[ring_read_idx];
    ring_read_idx = (uint16)((ring_read_idx + 1U) & (VISION_LINK_RX_BUF_SIZE - 1U));
    return 1U;
}

void vision_link_poll(void)
{
    uint8 dat;
    uint16 limit = VISION_LINK_RX_PARSE_LIMIT;
    uint32 request_period;
#if VISION_LINK_USE_UART_POLL
    uint8 poll_limit = 64U;

    while(poll_limit-- && uart_query_byte(VISION_LINK_UART_INDEX, &dat))
    {
        g_vis.poll_bytes++;
        vision_link_rx_byte(dat);
    }
#endif

    while(limit-- && vision_link_ring_read(&dat))
    {
        g_vis.parsed_bytes++;
        vision_link_parse_byte(dat);
    }

    request_period = VISION_LINK_REQ_FAST_PERIOD;
    if(g_vis.map_packets == 0U &&
       (uint32)(pit_count - request_last_tick) >= request_period)
    {
        vision_link_request_full_map();
    }
}

static void vision_link_parse_byte(uint8 dat)
{
    switch(rx_state)
    {
        case RX_WAIT_H0:
            if(dat == VISION_LINK_HEADER0)
            {
                rx_state = RX_WAIT_H1;
            }
            break;

        case RX_WAIT_H1:
            if(dat == VISION_LINK_HEADER1)
            {
                rx_state = RX_WAIT_TYPE;
            }
            else if(dat != VISION_LINK_HEADER0)
            {
                rx_state = RX_WAIT_H0;
            }
            break;

        case RX_WAIT_TYPE:
            rx_type = dat;
            rx_len = 0;
            if(rx_type == VISION_LINK_TYPE_FULL_MAP)
            {
                rx_expected = VISION_LINK_FULL_V1_PAYLOAD_LEN;
                rx_state = RX_PAYLOAD;
            }
            else if(rx_type == VISION_LINK_TYPE_FULL_MAP_V2)
            {
                rx_expected = VISION_LINK_FULL_PAYLOAD_LEN;
                rx_state = RX_PAYLOAD;
            }
            else if(rx_type == VISION_LINK_TYPE_POS_UPDATE)
            {
                rx_expected = VISION_LINK_POS_PAYLOAD_LEN;
                rx_state = RX_PAYLOAD;
            }
            else
            {
                g_vis.format_errors++;
                rx_state = RX_WAIT_H0;
            }
            break;

        case RX_PAYLOAD:
            if(rx_len >= VISION_LINK_PAYLOAD_MAX)
            {
                g_vis.format_errors++;
                rx_state = RX_WAIT_H0;
                break;
            }

            rx_payload[rx_len++] = dat;

            if(rx_expected != 0U && rx_len >= rx_expected)
            {
                vision_link_finish_packet();
                rx_state = RX_WAIT_H0;
            }
            break;

        default:
            rx_state = RX_WAIT_H0;
            break;
    }
}

void vision_link_rx_byte(uint8 dat)
{
    uint16 next;

    g_vis.rx_bytes++;
    if(g_vis.raw_count < sizeof(g_vis.raw_bytes))
    {
        g_vis.raw_bytes[g_vis.raw_count++] = dat;
    }

    next = (uint16)((ring_write_idx + 1U) & (VISION_LINK_RX_BUF_SIZE - 1U));
    if(next == ring_read_idx)
    {
        g_vis.ring_drops++;
        return;
    }

    ring_buf[ring_write_idx] = dat;
    ring_write_idx = next;
}

void vision_link_get_snapshot(vision_link_snapshot_t *out)
{
    if(out == 0)
    {
        return;
    }

    *out = g_vis;
}

uint8 vision_link_get_pose_grid(float *x_grid, float *y_grid, float *theta_deg)
{
    if(!vision_link_is_online() || !g_vis.pose_valid)
    {
        return 0U;
    }

    if(x_grid != 0)
    {
        *x_grid = (float)g_vis.car_x_mm * 0.1f;
    }
    if(y_grid != 0)
    {
        *y_grid = (float)g_vis.car_y_mm * 0.1f;
    }
    if(theta_deg != 0)
    {
        *theta_deg = (float)g_vis.car_theta_x10 * 0.1f;
    }

    return 1U;
}

uint8 vision_link_get_map(vision_link_map_t *out)
{
    if(out == 0 || !g_full_map_cache.valid)
    {
        return 0U;
    }
    *out = g_full_map_cache;
    return 1U;
}

uint8 vision_link_is_online(void)
{
    if(g_vis.last_packet_tick == 0U)
    {
        return 0;
    }

    return ((pit_count - g_vis.last_packet_tick) <= VISION_LINK_ONLINE_TIMEOUT) ? 1U : 0U;
}

void vision_link_show_ips200(void)
{
    static uint8 screen_cleared = 0;
    vision_link_snapshot_t s;
    uint32 age_ms;
    int32 car_x10;
    int32 car_y10;

    vision_link_get_snapshot(&s);
    age_ms = (s.last_packet_tick == 0U) ? 99999UL : ((pit_count - s.last_packet_tick) * 5UL);
    car_x10 = (int32)s.car_x_mm;
    car_y10 = (int32)s.car_y_mm;

    ips200_set_font(IPS200_6X8_FONT);
    if(!screen_cleared)
    {
        ips200_clear();
        screen_cleared = 1;
    }

    ips200_show_string(0, 0, "VISION UART4");
    ips200_show_string(96, 0, vision_link_is_online() ? "ONLINE " : "WAIT   ");
    ips200_show_string(156, 0, "BAUD:");
    ips200_show_uint(186, 0, VISION_LINK_BAUD, 6);

    ips200_show_string(0, 12, "RX:");
    ips200_show_uint(24, 12, s.rx_bytes, 8);
    ips200_show_string(96, 12, "AGE:");
    ips200_show_uint(126, 12, age_ms, 5);
    ips200_show_string(168, 12, "ms");

    ips200_show_string(0, 24, "POS:");
    ips200_show_uint(30, 24, s.pos_packets, 6);
    ips200_show_string(90, 24, "MAP:");
    ips200_show_uint(120, 24, s.map_packets, 5);
    ips200_show_string(168, 24, "REQ:");
    ips200_show_uint(198, 24, s.map_request_count, 3);

    ips200_show_string(0, 36, "ERR C/F:");
    ips200_show_uint(54, 36, s.checksum_errors, 4);
    ips200_show_string(84, 36, "/");
    ips200_show_uint(90, 36, s.format_errors, 4);
    ips200_show_string(132, 36, "TYPE:");
    if(s.last_type == VISION_LINK_TYPE_POS_UPDATE)
    {
        ips200_show_string(168, 36, "POS");
    }
    else if(s.last_type == VISION_LINK_TYPE_FULL_MAP ||
            s.last_type == VISION_LINK_TYPE_FULL_MAP_V2)
    {
        ips200_show_string(168, 36, "MAP");
    }
    else
    {
        ips200_show_string(168, 36, "---");
    }

    ips200_show_string(0, 52, "CAR cell x10:");
    ips200_show_int(84, 52, car_x10, 4);
    ips200_show_string(126, 52, "Y10:");
    ips200_show_int(156, 52, car_y10, 4);

    ips200_show_string(0, 64, "MM X:");
    ips200_show_int(36, 64, s.car_x_mm, 5);
    ips200_show_string(90, 64, "Y:");
    ips200_show_int(108, 64, s.car_y_mm, 5);

    ips200_show_string(0, 76, "TH x10:");
    ips200_show_int(48, 76, s.car_theta_x10, 5);
    ips200_show_string(102, 76, "deg:");
    ips200_show_float(132, 76, ((float)s.car_theta_x10) / 10.0f, 3, 1);

    ips200_show_string(0, 92, "BOX:");
    ips200_show_uint(30, 92, s.box_count, 2);
    ips200_show_string(60, 92, "GOAL:");
    ips200_show_uint(96, 92, s.goal_count, 2);
    ips200_show_string(126, 92, "BOMB:");
    ips200_show_uint(162, 92, s.bomb_count, 2);
    ips200_show_string(192, 92, "W:");
    ips200_show_uint(210, 92, s.wall_count, 3);

    ips200_show_string(0, 108, "FRAME:");
    ips200_show_uint(42, 108, s.frame_id, 3);
    ips200_show_string(84, 108, "PIT:");
    ips200_show_uint(114, 108, pit_count, 7);

    ips200_show_string(0, 124, "OpenART TX -> RT1064 RX: C17  TX:C16 115200");
}
