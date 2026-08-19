#ifndef __STATUS_BUZZER_H
#define __STATUS_BUZZER_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "app_config.h"

/* All timing is centralized here for track-side adjustment. */
#define STATUS_BUZZER_ENABLE                 (APP_BUZZER_ENABLE)
#define STATUS_BUZZER_PIN                    (APP_BUZZER_PIN)
#define STATUS_BUZZER_ACTIVE_LEVEL           (1U)
#define STATUS_BUZZER_IDLE_LEVEL             (0U)
#define STATUS_BUZZER_TICK_MS                (5U)
#define STATUS_BUZZER_QUEUE_SIZE             (16U)

#define STATUS_BUZZER_BOOT_MS                (APP_BUZZER_BOOT_MS)
#define STATUS_BUZZER_COMMAND_MS             (APP_BUZZER_COMMAND_MS)
#define STATUS_BUZZER_PREPARE_START_MS       (APP_BUZZER_PREPARE_START_MS)
#define STATUS_BUZZER_MAP_REQUEST_MS         (APP_BUZZER_MAP_REQUEST_MS)
#define STATUS_BUZZER_SOLVE_DONE_MS          (APP_BUZZER_SOLVE_DONE_MS)
#define STATUS_BUZZER_NODE_REACHED_MS        (APP_BUZZER_NODE_REACHED_MS)
#define STATUS_BUZZER_LOCKED_MS              (APP_BUZZER_LOCKED_MS)
#define STATUS_BUZZER_ALIGN_PULSE_MS         (APP_BUZZER_ALIGN_PULSE_MS)
#define STATUS_BUZZER_ALIGN_GAP_MS           (APP_BUZZER_ALIGN_GAP_MS)

typedef enum {
    STATUS_BUZZER_EVENT_NONE = 0,
    STATUS_BUZZER_EVENT_BOOT,
    STATUS_BUZZER_EVENT_COMMAND,
    STATUS_BUZZER_EVENT_PREPARE_START,
    STATUS_BUZZER_EVENT_MAP_REQUEST,
    STATUS_BUZZER_EVENT_SOLVE_DONE,
    STATUS_BUZZER_EVENT_NODE_REACHED,
    STATUS_BUZZER_EVENT_LOCKED,
    STATUS_BUZZER_EVENT_STRONG_ALIGN
} status_buzzer_event_t;

void status_buzzer_init(void);
uint8 status_buzzer_request(status_buzzer_event_t event);
void status_buzzer_tick_5ms(void);
uint32 status_buzzer_drop_count(void);

#endif
