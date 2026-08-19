#ifndef __PC_CONSOLE_H
#define __PC_CONSOLE_H

#include "zf_common_typedef.h"

void pc_console_init(uint8 app_mode);
void pc_console_poll(void);
void pc_console_uart8_rx_isr(void);
void pc_console_uart8_tx_isr(void);

#endif
