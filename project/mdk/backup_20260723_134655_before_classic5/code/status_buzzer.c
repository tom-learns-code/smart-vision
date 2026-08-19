#include "zf_common_headfile.h"
#include "status_buzzer.h"

#define STATUS_BUZZER_QUEUE_MASK (STATUS_BUZZER_QUEUE_SIZE - 1U)

static volatile uint8 buzzer_queue[STATUS_BUZZER_QUEUE_SIZE];
static volatile uint8 buzzer_head;
static volatile uint8 buzzer_tail;
static volatile uint8 buzzer_urgent_lock;
static volatile uint8 buzzer_current_event;
static volatile uint8 buzzer_pattern_phase;
static volatile uint16 buzzer_ticks_remaining;
static volatile uint32 buzzer_drops;

static uint16 status_buzzer_ms_to_ticks(uint16 duration_ms)
{
    return (uint16)((duration_ms + STATUS_BUZZER_TICK_MS - 1U) /
                    STATUS_BUZZER_TICK_MS);
}

static uint16 status_buzzer_event_ticks(uint8 event)
{
    switch((status_buzzer_event_t)event)
    {
        case STATUS_BUZZER_EVENT_BOOT:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_BOOT_MS);
        case STATUS_BUZZER_EVENT_COMMAND:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_COMMAND_MS);
        case STATUS_BUZZER_EVENT_PREPARE_START:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_PREPARE_START_MS);
        case STATUS_BUZZER_EVENT_MAP_REQUEST:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_MAP_REQUEST_MS);
        case STATUS_BUZZER_EVENT_SOLVE_DONE:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_SOLVE_DONE_MS);
        case STATUS_BUZZER_EVENT_NODE_REACHED:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_NODE_REACHED_MS);
        case STATUS_BUZZER_EVENT_LOCKED:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_LOCKED_MS);
        case STATUS_BUZZER_EVENT_STRONG_ALIGN:
            return status_buzzer_ms_to_ticks(STATUS_BUZZER_ALIGN_PULSE_MS);
        default:
            return 0U;
    }
}

static void status_buzzer_start_isr(uint8 event)
{
    buzzer_current_event = event;
    buzzer_pattern_phase = 0U;
    buzzer_ticks_remaining = status_buzzer_event_ticks(event);
    if(buzzer_ticks_remaining == 0U)
    {
        buzzer_current_event = STATUS_BUZZER_EVENT_NONE;
        gpio_set_level(STATUS_BUZZER_PIN, STATUS_BUZZER_IDLE_LEVEL);
        return;
    }
    gpio_set_level(STATUS_BUZZER_PIN, STATUS_BUZZER_ACTIVE_LEVEL);
}

void status_buzzer_init(void)
{
    buzzer_head = 0U;
    buzzer_tail = 0U;
    buzzer_urgent_lock = 0U;
    buzzer_current_event = STATUS_BUZZER_EVENT_NONE;
    buzzer_pattern_phase = 0U;
    buzzer_ticks_remaining = 0U;
    buzzer_drops = 0U;
    gpio_init(STATUS_BUZZER_PIN, GPO, STATUS_BUZZER_IDLE_LEVEL,
              GPO_PUSH_PULL);
}

uint8 status_buzzer_request(status_buzzer_event_t event)
{
#if STATUS_BUZZER_ENABLE
    uint32 primask;
    uint8 next;

    if(event <= STATUS_BUZZER_EVENT_NONE ||
       event > STATUS_BUZZER_EVENT_STRONG_ALIGN)
        return 0U;

    primask = interrupt_global_disable();
    if(event == STATUS_BUZZER_EVENT_LOCKED)
    {
        buzzer_urgent_lock = 1U;
        interrupt_global_enable(primask);
        return 1U;
    }

    next = (uint8)((buzzer_head + 1U) & STATUS_BUZZER_QUEUE_MASK);
    if(next == buzzer_tail)
    {
        buzzer_drops++;
        interrupt_global_enable(primask);
        return 0U;
    }
    buzzer_queue[buzzer_head] = (uint8)event;
    buzzer_head = next;
    interrupt_global_enable(primask);
    return 1U;
#else
    (void)event;
    return 0U;
#endif
}

void status_buzzer_tick_5ms(void)
{
#if STATUS_BUZZER_ENABLE
    if(buzzer_urgent_lock)
    {
        buzzer_urgent_lock = 0U;
        buzzer_tail = buzzer_head;
        status_buzzer_start_isr(STATUS_BUZZER_EVENT_LOCKED);
        return;
    }

    if(buzzer_ticks_remaining > 0U)
    {
        buzzer_ticks_remaining--;
        if(buzzer_ticks_remaining > 0U) return;

        if(buzzer_current_event == STATUS_BUZZER_EVENT_STRONG_ALIGN &&
           buzzer_pattern_phase == 0U)
        {
            gpio_set_level(STATUS_BUZZER_PIN, STATUS_BUZZER_IDLE_LEVEL);
            buzzer_pattern_phase = 1U;
            buzzer_ticks_remaining =
                status_buzzer_ms_to_ticks(STATUS_BUZZER_ALIGN_GAP_MS);
            return;
        }
        if(buzzer_current_event == STATUS_BUZZER_EVENT_STRONG_ALIGN &&
           buzzer_pattern_phase == 1U)
        {
            gpio_set_level(STATUS_BUZZER_PIN, STATUS_BUZZER_ACTIVE_LEVEL);
            buzzer_pattern_phase = 2U;
            buzzer_ticks_remaining =
                status_buzzer_ms_to_ticks(STATUS_BUZZER_ALIGN_PULSE_MS);
            return;
        }

        gpio_set_level(STATUS_BUZZER_PIN, STATUS_BUZZER_IDLE_LEVEL);
        buzzer_current_event = STATUS_BUZZER_EVENT_NONE;
        buzzer_pattern_phase = 0U;
    }

    if(buzzer_current_event == STATUS_BUZZER_EVENT_NONE &&
       buzzer_tail != buzzer_head)
    {
        uint8 event = buzzer_queue[buzzer_tail];
        buzzer_tail = (uint8)((buzzer_tail + 1U) &
                              STATUS_BUZZER_QUEUE_MASK);
        status_buzzer_start_isr(event);
    }
#endif
}

uint32 status_buzzer_drop_count(void)
{
    return buzzer_drops;
}
