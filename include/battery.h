#pragma once

#include <TSchedulerDeclarations.hpp>
#include <RunningAverage.h>

extern TsTask bat_tSetup;
extern TsTask bat_tLoop;
extern RunningAverage bat_percentage_ra;
extern bool bat_charging;

void bat_setup();
void bat_loop();
float bat_get_voltage();
float bat_get_percentage_for_voltage(float voltage);