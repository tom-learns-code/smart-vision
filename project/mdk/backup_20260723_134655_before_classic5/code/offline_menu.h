#ifndef __OFFLINE_MENU_H
#define __OFFLINE_MENU_H

#include "zf_common_headfile.h"
#include "match_manager.h"

/* C12/C13/C14实体按键与串口、离线屏幕共用同一套比赛状态机。 */
void race_button_controls_init(uint8 menu_navigation_enable);
void race_button_controls_poll(void);
void offline_menu_init(void);
void offline_menu_poll(void);
uint8 offline_menu_handle_key(match_key_event_t event);

#endif
