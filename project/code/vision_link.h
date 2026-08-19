#ifndef __VISION_LINK_H
#define __VISION_LINK_H

#include "zf_common_typedef.h"

#define VISION_LINK_SCREEN_ENABLE      (1U)
#define VISION_LINK_BAUD               (115200U)
#define VISION_LINK_GRID_SIZE_MM       (200)

#define VISION_LINK_TYPE_FULL_MAP      (0x01U)
#define VISION_LINK_TYPE_FULL_MAP_V2   (0x02U) /* five box/goal slots */
#define VISION_LINK_TYPE_POS_UPDATE    (0x12U) /* v2: bit7 of payload[0] is car_valid */
#define VISION_LINK_TYPE_MAP_REQUEST   (0x03U)

#define VISION_LINK_GRID_W             (16U)
#define VISION_LINK_GRID_H             (12U)
#define VISION_LINK_WALL_BYTES         (24U)
#define VISION_LINK_MAX_BOXES          (5U)
#define VISION_LINK_MAX_GOALS          (5U)
#define VISION_LINK_MAX_BOMBS          (4U)

typedef struct
{
    int8 gx;
    int8 gy;
} vision_link_cell_t;

typedef struct
{
    uint8 valid;
    uint32 map_version;
    uint8 width;
    uint8 height;
    uint8 wall_bits[VISION_LINK_WALL_BYTES];
    uint8 box_count;
    uint8 goal_count;
    uint8 bomb_count;
    vision_link_cell_t boxes[VISION_LINK_MAX_BOXES];
    vision_link_cell_t goals[VISION_LINK_MAX_GOALS];
    vision_link_cell_t bombs[VISION_LINK_MAX_BOMBS];
} vision_link_map_t;

typedef struct
{
    uint32 rx_bytes;
    uint32 poll_bytes;
    uint32 parsed_bytes;
    uint32 ring_drops;
    uint32 pos_packets;
    uint32 map_packets;
    uint32 checksum_errors;
    uint32 format_errors;
    uint32 map_request_count;
    uint32 last_packet_tick;
    uint8 last_type;
    uint8 frame_id;
    uint8 pose_valid;
    uint8 raw_count;
    uint8 raw_bytes[8];
    int16 car_x_mm;
    int16 car_y_mm;
    int16 car_theta_x10;
    uint8 box_count;
    uint8 goal_count;
    uint8 bomb_count;
    uint8 wall_count;
    uint32 map_version;
    uint8 map_valid;
    uint8 map_width;
    uint8 map_height;
    uint8 wall_bits[VISION_LINK_WALL_BYTES];
    vision_link_cell_t boxes[VISION_LINK_MAX_BOXES];
    vision_link_cell_t goals[VISION_LINK_MAX_GOALS];
    vision_link_cell_t bombs[VISION_LINK_MAX_BOMBS];
} vision_link_snapshot_t;

void vision_link_init(void);
void vision_link_poll(void);
void vision_link_rx_byte(uint8 dat);
void vision_link_request_full_map(void);
void vision_link_get_snapshot(vision_link_snapshot_t *out);
uint8 vision_link_get_pose_grid(float *x_grid, float *y_grid, float *theta_deg);
uint8 vision_link_get_map(vision_link_map_t *out);
uint8 vision_link_is_online(void);
void vision_link_show_ips200(void);

#endif
