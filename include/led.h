#pragma once

#include <TSchedulerDeclarations.hpp>

extern TsScheduler ts;
extern TsTask led_tStartupBlink;
extern TsTask led_tAliveBlink;
extern TsTask led_tBLEDisconnectedBlink;

void led_startupBlink();
void led_startupBlinkOnDisable();
void led_aliveBlink();
void led_bleDisconnectedBlink();
void led_bleDisconnectedBlinkOnDisable();