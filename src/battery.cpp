#include <Arduino.h>

#include "battery.h"

#define BAT_HIGH_CHARGE 22             // HIGH for 50mA, LOW for 100mA
#define BAT_CHARGE_STATE 23            // LOW for charging, HIGH not charging
#define VBAT_MV_PER_LBS (0.003515625F) // 3.6 reference and 10 bit resolution
#define BAT_R1 979                     // Originally 1M ohm, resistor tolerances, temperature ect..

TsTask bat_tSetup(TASK_IMMEDIATE, TASK_ONCE, &bat_setup);
TsTask bat_tLoop(100 * TASK_MILLISECOND, TASK_FOREVER, &bat_loop);
RunningAverage bat_percentage_ra(100);
bool bat_charging = false;

void bat_setup()
{
  pinMode(VBAT_ENABLE, OUTPUT);
  pinMode(BAT_CHARGE_STATE, INPUT);
  pinMode(BAT_HIGH_CHARGE, OUTPUT);
  digitalWrite(BAT_HIGH_CHARGE, LOW);

  bat_tLoop.enable();
}

void bat_loop()
{
  static uint16_t iteration = 0;
  iteration++;
  float bat_voltage = bat_get_voltage();
  float bat_percentage = bat_get_percentage_for_voltage(bat_voltage);
  bat_percentage_ra.add(bat_percentage);
  bat_charging = digitalRead(BAT_CHARGE_STATE);

#ifdef _DEBUG_
  if (iteration % 50 == 0)
  {
    float avg = bat_percentage_ra.getAverage();
    printf(PSTR("Bat %s percentage: %f%\r\n"), bat_charging ? "not charging" : "charging", avg);
    printf(PSTR(">bat:%f\n"), avg);
  }
#endif
}

float bat_get_voltage()
{
  digitalWrite(VBAT_ENABLE, LOW);

  uint32_t adcCount = analogRead(PIN_VBAT);
  float adcVoltage = adcCount * VBAT_MV_PER_LBS;
  float vBat = adcVoltage * ((BAT_R1 + 510.0f) / 510.0f);

  digitalWrite(VBAT_ENABLE, HIGH);

  return vBat;
}

float bat_get_percentage_for_voltage(float voltage)
{
  return 123.f - (123.f / pow(1.f + pow(voltage / 3.7f, 80.f), 0.165f));
}