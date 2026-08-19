#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "bomb_platform.h"

uint32_t bomb_platform_now_ms(void)
{
    return (uint32_t)GetTickCount64();
}
