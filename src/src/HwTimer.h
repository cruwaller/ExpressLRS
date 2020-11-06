#pragma once

#include "platform.h"
#include <stdint.h>

#define PRINT_TIMER 0
#define PRINT_HW_ISR 0
#define PRINT_RX_ISR 0
#define PRINT_TMR 0

#define TimerIntervalUSDefault 20000

#define TIMER_SOON 40 //80 // 80us

#if RADIO_SX128x && PLATFORM_STM32 && !defined(ARDUINO)
#define USE_TIMER_KICK  1   // TODO: Need testing!!
#endif

class HwTimer
{
public:
    HwTimer();
    void init();
    void ICACHE_RAM_ATTR start();
    void ICACHE_RAM_ATTR reset(int32_t offset = 0);
    void ICACHE_RAM_ATTR pause();
    void ICACHE_RAM_ATTR stop();
    void ICACHE_RAM_ATTR updateInterval(uint32_t newTimerInterval);
    bool ICACHE_RAM_ATTR isRunning(void)
    {
        return running;
    }

    void ICACHE_RAM_ATTR callback();

    void (*callbackTock)(uint32_t us);

    void ICACHE_RAM_ATTR setTime(uint32_t time = 0);

    void ICACHE_RAM_ATTR triggerSoon(void);

private:
    volatile uint32_t HWtimerInterval;
    volatile bool running = false;
};

extern HwTimer TxTimer;
