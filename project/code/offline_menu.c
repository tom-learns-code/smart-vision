#include <stdio.h>
#include "offline_menu.h"
#include "app_config.h"
#include "motion_control.h"
#include "status_buzzer.h"

#define OFFLINE_TICK_MS                    (5U)
#define OFFLINE_MS_TO_TICKS(ms)            (((ms) + OFFLINE_TICK_MS - 1U) / OFFLINE_TICK_MS)
#define OFFLINE_BUTTON_COUNT               (3U)
#define OFFLINE_BUTTON_PREV                (0U)
#define OFFLINE_BUTTON_NEXT                (1U)
#define OFFLINE_BUTTON_OK                  (2U)

#define OFFLINE_FIELD_PRESET               (0U)
#define OFFLINE_FIELD_ROUND_BASE           (1U)
#define OFFLINE_FIELD_PER_ROUND            (4U)
#define OFFLINE_FIELD_RUN_OFFSET           (0U)
#define OFFLINE_FIELD_STRATEGY_OFFSET      (1U)
#define OFFLINE_FIELD_SPEED_OFFSET         (2U)
#define OFFLINE_FIELD_ALGORITHM_OFFSET     (3U)
#define OFFLINE_FIELD_START                (OFFLINE_FIELD_ROUND_BASE + \
                                             APP_RACE_ROUND_COUNT * \
                                             OFFLINE_FIELD_PER_ROUND)
#define OFFLINE_FIELD_COUNT                (OFFLINE_FIELD_START + 1U)

typedef struct
{
    gpio_pin_enum pin;
    uint8 raw_pressed;
    uint8 stable_pressed;
    uint8 ready;
    uint8 long_sent;
    uint32 raw_change_tick;
    uint32 press_tick;
} offline_button_t;

static offline_button_t g_offline_buttons[OFFLINE_BUTTON_COUNT];
static uint32 g_offline_last_draw_tick = 0U;
static uint32 g_offline_last_event = 0xFFFFFFFFUL;
static uint8 g_offline_selected_field = OFFLINE_FIELD_PRESET;
static uint8 g_offline_force_draw = 1U;
static uint8 g_race_button_menu_navigation = 1U;
static char g_offline_last_action[32] = "BOOT";

static const char *offline_short_algorithm(uint8 algorithm)
{
    if(algorithm == APP_RACE_ALGO_IMAGE_ONLY) return "IMG";
    if(algorithm == APP_RACE_ALGO_BOMB_IMAGE) return "BOMB";
    return "BOX";
}

static const char *offline_short_strategy(uint8 strategy)
{
    if(strategy == APP_RACE_STRATEGY_NORMAL) return "NORMAL";
    if(strategy == APP_RACE_STRATEGY_SPRINT) return "SPRINT";
    return "SAFE";
}

/* 用空格补满整行，覆盖上一次较长内容，避免全屏清除产生黑帧闪烁。 */
static void offline_show_line(uint16 y, const char *text)
{
    char padded[31];
    uint8 i;

    for(i = 0U; i < 30U; i++)
    {
        padded[i] = (text != 0 && text[i] != '\0') ? text[i] : ' ';
        if(text == 0 || text[i] == '\0')
        {
            i++;
            while(i < 30U) padded[i++] = ' ';
            break;
        }
    }
    padded[30] = '\0';
    ips200_show_string(0, y, padded);
}

static const char *offline_short_state(match_state_t state)
{
    switch(state)
    {
        case MATCH_STATE_IDLE: return "IDLE";
        case MATCH_STATE_ARMED: return "ARMED";
        case MATCH_STATE_BASE_POSE_WAIT: return "BASE_WAIT";
        case MATCH_STATE_BASE_ALIGN_RUNNING: return "BASE_ALIGN";
        case MATCH_STATE_EXIT_PREP: return "EXIT_PREP";
        case MATCH_STATE_EXIT_RUNNING: return "EXIT_RUN";
        case MATCH_STATE_SKIP_RETURN: return "SKIP_BACK";
        case MATCH_STATE_MAP_WAIT: return "MAP_WAIT";
        case MATCH_STATE_POST_MAP_POSE_WAIT: return "POSE_WAIT";
        case MATCH_STATE_SOLVING: return "SOLVING";
        case MATCH_STATE_MISSION_ARMING: return "MISSION_ARM";
        case MATCH_STATE_MISSION_RETRY_WAIT: return "RETRY_WAIT";
        case MATCH_STATE_MISSION_RUNNING: return "RUNNING";
        case MATCH_STATE_FINISH_SCAN: return "FINISH_SCAN";
        case MATCH_STATE_RETURN_PREP: return "RETURN_PREP";
        case MATCH_STATE_RETURN_DIRECT: return "RETURN_DIR";
        case MATCH_STATE_RETURN_PATH: return "RETURN_PATH";
        case MATCH_STATE_ROUND_DONE: return "ROUND_DONE";
        case MATCH_STATE_BETWEEN_ROUNDS: return "NEXT_WAIT";
        case MATCH_STATE_WAIT_OPERATOR: return "WAIT_KEY";
        case MATCH_STATE_COMPLETE: return "COMPLETE";
        case MATCH_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static uint8 offline_field_round(uint8 field)
{
    if(field < OFFLINE_FIELD_ROUND_BASE || field >= OFFLINE_FIELD_START)
        return 0xFFU;
    return (uint8)((field - OFFLINE_FIELD_ROUND_BASE) /
                   OFFLINE_FIELD_PER_ROUND);
}

static uint8 offline_field_offset(uint8 field)
{
    return (uint8)((field - OFFLINE_FIELD_ROUND_BASE) %
                   OFFLINE_FIELD_PER_ROUND);
}

static void offline_selected_text(
    const match_status_t *status, char *name, uint32 name_size,
    char *value, uint32 value_size)
{
    uint8 round_index;
    uint8 offset;
    const app_race_round_config_t *round;

    if(g_offline_selected_field == OFFLINE_FIELD_PRESET)
    {
        snprintf(name, name_size, "PRESET");
        snprintf(value, value_size, "%s",
                 match_preset_name(status->preset_index));
        return;
    }
    if(g_offline_selected_field == OFFLINE_FIELD_START)
    {
        snprintf(name, name_size, "ARM / START");
        snprintf(value, value_size, "%s",
                 status->state == MATCH_STATE_ARMED ? "START NOW" :
                 (status->running ? "RUNNING" : "ARM MATCH"));
        return;
    }

    round_index = offline_field_round(g_offline_selected_field);
    offset = offline_field_offset(g_offline_selected_field);
    round = &status->round_config[round_index];
    if(offset == OFFLINE_FIELD_RUN_OFFSET)
    {
        snprintf(name, name_size, "R%u RUN", (unsigned)(round_index + 1U));
        snprintf(value, value_size, "%s", round->run ? "RUN" : "SKIP");
    }
    else if(offset == OFFLINE_FIELD_STRATEGY_OFFSET)
    {
        snprintf(name, name_size, "R%u STRATEGY", (unsigned)(round_index + 1U));
        snprintf(value, value_size, "%s",
                 offline_short_strategy((uint8)round->strategy));
    }
    else if(offset == OFFLINE_FIELD_SPEED_OFFSET)
    {
        snprintf(name, name_size, "R%u SPEED", (unsigned)(round_index + 1U));
        snprintf(value, value_size, "%u", (unsigned)round->speed);
    }
    else
    {
        snprintf(name, name_size, "R%u ALGORITHM", (unsigned)(round_index + 1U));
        snprintf(value, value_size, "%s",
                 offline_short_algorithm((uint8)round->algorithm));
    }
}

static void offline_menu_draw(void)
{
    match_status_t status;
    uint8 i;
    uint8 selected_round;
    char line[32];
    char selected_name[24];
    char selected_value[24];

    match_manager_get_status(&status);
    selected_round = offline_field_round(g_offline_selected_field);
    offline_selected_text(&status, selected_name, sizeof(selected_name),
                          selected_value, sizeof(selected_value));

    offline_show_line(0, "OFFLINE 3KEY ID7100969");
    snprintf(line, sizeof(line), "STATE:%s", offline_short_state(status.state));
    offline_show_line(16, line);
    snprintf(line, sizeof(line), "PRESET:%s", match_preset_name(status.preset_index));
    offline_show_line(32, line);

    for(i = 0U; i < APP_RACE_ROUND_COUNT; i++)
    {
        const app_race_round_config_t *round = &status.round_config[i];
        snprintf(line, sizeof(line), "%cR%u %s %s %u %s",
                 selected_round == i ? '>' : ' ', (unsigned)(i + 1U),
                 round->run ? "RUN" : "SKIP",
                 offline_short_strategy((uint8)round->strategy),
                 (unsigned)round->speed,
                 offline_short_algorithm((uint8)round->algorithm));
        offline_show_line((uint16)(48U + i * 16U), line);
    }

    snprintf(line, sizeof(line), "SELECT:%s", selected_name);
    offline_show_line(104, line);
    snprintf(line, sizeof(line), "VALUE:%s", selected_value);
    offline_show_line(120, line);
    snprintf(line, sizeof(line), "ROUND:%u/3 DONE:%u",
             (unsigned)(status.round_index + 1U),
             (unsigned)status.rounds_completed);
    offline_show_line(144, line);
    snprintf(line, sizeof(line), "GATE:%s", match_gate_name(status.gate));
    offline_show_line(160, line);
    snprintf(line, sizeof(line), "FAULT:%s", match_fault_name(status.fault));
    offline_show_line(176, line);
    snprintf(line, sizeof(line), "LAST:%s", g_offline_last_action);
    offline_show_line(192, line);

    offline_show_line(224, "C12 UP    C13 DOWN");
    offline_show_line(240, "C14 SET / HOLD GO");
    offline_show_line(256, "HOLD C12 CANCEL");
    offline_show_line(272, "HOLD C13 EMERGENCY");
}

static void offline_set_last(const char *text)
{
    snprintf(g_offline_last_action, sizeof(g_offline_last_action), "%s", text);
    g_offline_force_draw = 1U;
}

static void offline_command_feedback(uint8 accepted, const char *ok_text)
{
    if(accepted)
    {
        status_buzzer_request(STATUS_BUZZER_EVENT_COMMAND);
        offline_set_last(ok_text);
    }
    else
    {
        offline_set_last("REJECTED / BUSY");
    }
}

static uint8 offline_arm_or_start(void)
{
    match_status_t status;
    match_manager_get_status(&status);
    if(status.state == MATCH_STATE_ARMED)
        return match_manager_start_remote();
    return match_manager_arm();
}

static uint8 offline_apply_selected(void)
{
    uint8 round_index;
    uint8 offset;

    if(g_offline_selected_field == OFFLINE_FIELD_PRESET)
        return match_manager_cycle_preset();
    if(g_offline_selected_field == OFFLINE_FIELD_START)
        return offline_arm_or_start();

    round_index = offline_field_round(g_offline_selected_field);
    offset = offline_field_offset(g_offline_selected_field);
    if(!match_manager_select_config_round(round_index)) return 0U;
    if(offset == OFFLINE_FIELD_RUN_OFFSET)
        return match_manager_toggle_round_run();
    if(offset == OFFLINE_FIELD_STRATEGY_OFFSET)
        return match_manager_cycle_round_strategy();
    if(offset == OFFLINE_FIELD_SPEED_OFFSET)
        return match_manager_cycle_round_speed();
    return match_manager_cycle_round_algorithm();
}

static void offline_handle_short(uint8 button_index)
{
    if(!g_race_button_menu_navigation)
        return;

    if(button_index == OFFLINE_BUTTON_PREV)
    {
        g_offline_selected_field = g_offline_selected_field == 0U ?
            (uint8)(OFFLINE_FIELD_COUNT - 1U) :
            (uint8)(g_offline_selected_field - 1U);
        status_buzzer_request(STATUS_BUZZER_EVENT_COMMAND);
        offline_set_last("C12 UP");
    }
    else if(button_index == OFFLINE_BUTTON_NEXT)
    {
        g_offline_selected_field = (uint8)(
            (g_offline_selected_field + 1U) % OFFLINE_FIELD_COUNT);
        status_buzzer_request(STATUS_BUZZER_EVENT_COMMAND);
        offline_set_last("C13 DOWN");
    }
    else
    {
        offline_command_feedback(offline_apply_selected(), "C14 SET OK");
    }
}

static void offline_handle_long(uint8 button_index)
{
    if(!g_race_button_menu_navigation)
    {
        if(button_index == OFFLINE_BUTTON_NEXT)
        {
            match_manager_emergency_stop();
            offline_set_last("C13 EMERGENCY");
        }
        else if(button_index == OFFLINE_BUTTON_OK)
        {
            offline_command_feedback(offline_arm_or_start(), "C14 ARM / GO");
        }
        return;
    }

    if(button_index == OFFLINE_BUTTON_PREV)
    {
        match_manager_cancel();
        status_buzzer_request(STATUS_BUZZER_EVENT_COMMAND);
        offline_set_last("C12 CANCEL");
    }
    else if(button_index == OFFLINE_BUTTON_NEXT)
    {
        match_manager_emergency_stop();
        offline_set_last("C13 EMERGENCY");
    }
    else
    {
        offline_command_feedback(offline_arm_or_start(), "C14 ARM / GO");
    }
}

static uint8 offline_button_pressed(gpio_pin_enum pin)
{
    return (uint8)(gpio_get_level(pin) == APP_BUTTON_ACTIVE_LEVEL);
}

static void offline_buttons_init(void)
{
    static const gpio_pin_enum pins[OFFLINE_BUTTON_COUNT] =
    {
        APP_BUTTON_PREV_PIN,
        APP_BUTTON_NEXT_PIN,
        APP_BUTTON_OK_PIN
    };
    uint8 i;

    for(i = 0U; i < OFFLINE_BUTTON_COUNT; i++)
    {
        gpio_init(pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        g_offline_buttons[i].pin = pins[i];
        g_offline_buttons[i].raw_pressed = offline_button_pressed(pins[i]);
        g_offline_buttons[i].stable_pressed =
            g_offline_buttons[i].raw_pressed;
        g_offline_buttons[i].ready =
            g_offline_buttons[i].raw_pressed ? 0U : 1U;
        g_offline_buttons[i].long_sent = 0U;
        g_offline_buttons[i].raw_change_tick = pit_count;
        g_offline_buttons[i].press_tick = pit_count;
    }
}

static void offline_buttons_poll(void)
{
    uint8 i;
    uint32 now = pit_count;

    for(i = 0U; i < OFFLINE_BUTTON_COUNT; i++)
    {
        offline_button_t *button = &g_offline_buttons[i];
        uint8 pressed = offline_button_pressed(button->pin);

        if(pressed != button->raw_pressed)
        {
            button->raw_pressed = pressed;
            button->raw_change_tick = now;
        }

        if(button->raw_pressed != button->stable_pressed &&
           (uint32)(now - button->raw_change_tick) >=
               OFFLINE_MS_TO_TICKS(APP_BUTTON_DEBOUNCE_MS))
        {
            button->stable_pressed = button->raw_pressed;
            if(button->stable_pressed)
            {
                button->press_tick = now;
                button->long_sent = 0U;
            }
            else if(!button->ready)
            {
                button->ready = 1U;
            }
            else if(!button->long_sent &&
                    (uint32)(now - button->press_tick) <=
                        OFFLINE_MS_TO_TICKS(APP_BUTTON_SHORT_MAX_MS))
            {
                offline_handle_short(i);
            }
        }

        if(button->ready && button->stable_pressed && !button->long_sent &&
           (uint32)(now - button->press_tick) >=
               OFFLINE_MS_TO_TICKS(APP_BUTTON_LONG_MIN_MS))
        {
            button->long_sent = 1U;
            offline_handle_long(i);
        }
    }
}

void race_button_controls_init(uint8 menu_navigation_enable)
{
    g_race_button_menu_navigation = menu_navigation_enable ? 1U : 0U;
#if APP_BUTTON_HW_ENABLE
    offline_buttons_init();
#endif
}

void race_button_controls_poll(void)
{
#if APP_BUTTON_HW_ENABLE
    offline_buttons_poll();
#endif
}

void offline_menu_init(void)
{
    g_offline_last_draw_tick = pit_count;
    g_offline_last_event = 0xFFFFFFFFUL;
    g_offline_selected_field = OFFLINE_FIELD_PRESET;
    g_offline_force_draw = 1U;
    offline_set_last("READY");
    race_button_controls_init(1U);
    ips200_clear();
    offline_menu_draw();
}

void offline_menu_poll(void)
{
    match_status_t status;
    race_button_controls_poll();
    match_manager_get_status(&status);
    if(g_offline_force_draw || status.event_counter != g_offline_last_event ||
       (uint32)(pit_count - g_offline_last_draw_tick) >=
           OFFLINE_MS_TO_TICKS(APP_OFFLINE_MENU_REFRESH_MS))
    {
        g_offline_force_draw = 0U;
        g_offline_last_event = status.event_counter;
        g_offline_last_draw_tick = pit_count;
        offline_menu_draw();
    }
}

uint8 offline_menu_handle_key(match_key_event_t event)
{
    uint8 accepted = match_manager_handle_key(event);
    g_offline_last_event = 0xFFFFFFFFUL;
    g_offline_force_draw = 1U;
    return accepted;
}
