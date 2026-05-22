#include <Arduino.h>

#include "battery.h"

#define BAT_HIGH_CHARGE 22              // HIGH for 50mA, LOW for 100mA
#define BAT_CHARGE_STATE 23             // LOW for charging, HIGH not charging
#define BAT_ANALOG_RESOLUTION 12        // 12-bit ADC resolution
#define VBAT_VOLTS_PER_LBS (0.000878906F) // 3.6V reference, 12-bit resolution (3.6 / 4096)
#define BAT_R1 979                      // Originally 1M ohm, resistor tolerances, temperature etc.
#define BAT_CHARGING_VOLTAGE_OFFSET 0.10F // Approx. terminal voltage offset (V) during CC charging at 100mA

TsTask bat_tSetup(TASK_IMMEDIATE, TASK_ONCE, &bat_setup);
TsTask bat_tLoop(100 * TASK_MILLISECOND, TASK_FOREVER, &bat_loop);
RunningAverage bat_percentage_ra(100);
bool bat_charging = false;

void bat_setup()
{
  analogReadResolution(BAT_ANALOG_RESOLUTION); // Use 12-bit ADC for ~2.6 mV/LSB at the battery terminal
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
  // During CC charging the terminal voltage is elevated above OCV by roughly
  // I * R_internal. Subtract a fixed offset so the lookup table sees a value
  // closer to the true open-circuit voltage, giving a more accurate SOC.
  bool is_charging = !digitalRead(BAT_CHARGE_STATE);
  float effective_voltage = bat_voltage - (is_charging ? BAT_CHARGING_VOLTAGE_OFFSET : 0.0f);
  float bat_percentage = bat_get_percentage_for_voltage(effective_voltage);
  bat_percentage_ra.add(bat_percentage);
  bat_charging = is_charging;

#ifdef _DEBUG_
  if (iteration % 50 == 0)
  {
    float avg = bat_percentage_ra.getAverage();
    printf(PSTR("Bat %s percentage: %f%\r\n"), bat_charging ? "charging" : "not charging", avg);
  }
#endif
}

float bat_get_voltage()
{
  digitalWrite(VBAT_ENABLE, LOW);

  uint32_t adcCount = analogRead(PIN_VBAT);
  float adcVoltage = adcCount * VBAT_VOLTS_PER_LBS;
  float vBat = adcVoltage * ((BAT_R1 + 510.0f) / 510.0f);

  digitalWrite(VBAT_ENABLE, HIGH);

  return vBat;
}

// OCV-SOC lookup table for a 3.7V nominal LiPo (4.2V max, 3.0V cutoff).
static const float bat_ocv_table[] = {
    3.000f, 3.280f, 3.360f, 3.420f, 3.490f, 3.550f,
    3.610f, 3.670f, 3.710f, 3.750f, 3.790f, 3.830f,
    3.870f, 3.910f, 3.950f, 3.980f, 4.020f, 4.080f,
    4.110f, 4.150f, 4.200f
};
static const float bat_soc_table[] = {
     0.0f,  5.0f, 10.0f, 15.0f, 20.0f, 25.0f,
    30.0f, 35.0f, 40.0f, 45.0f, 50.0f, 55.0f,
    60.0f, 65.0f, 70.0f, 75.0f, 80.0f, 85.0f,
    90.0f, 95.0f, 100.0f
};
static const int bat_table_size = (int)(sizeof(bat_ocv_table) / sizeof(bat_ocv_table[0]));

float bat_get_percentage_for_voltage(float voltage)
{
  if (voltage <= bat_ocv_table[0])
    return 0.0f;
  if (voltage >= bat_ocv_table[bat_table_size - 1])
    return 100.0f;

  for (int i = 1; i < bat_table_size; i++)
  {
    if (voltage <= bat_ocv_table[i])
    {
      float v0 = bat_ocv_table[i - 1];
      float v1 = bat_ocv_table[i];
      float p0 = bat_soc_table[i - 1];
      float p1 = bat_soc_table[i];
      return p0 + (voltage - v0) / (v1 - v0) * (p1 - p0);
    }
  }
  return 100.0f;
}
