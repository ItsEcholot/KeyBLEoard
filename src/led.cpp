#include <Arduino.h>

#include "led.h"
#include "usb.h"

TsTask led_tStartupBlink(100 * TASK_MILLISECOND, TASK_FOREVER, &led_startupBlink, nullptr, false, nullptr, &led_startupBlinkOnDisable);
TsTask led_tAliveBlink(1 * TASK_SECOND, 20, &led_aliveBlink);
TsTask led_tBLEDisconnectedBlink(75 * TASK_MILLISECOND, TASK_FOREVER, &led_bleDisconnectedBlink, nullptr, false, nullptr, &usb_caps_reset_led);
TsTask led_tBLESlotSaveDone(TASK_IMMEDIATE, TASK_ONCE, &usb_caps_reset_led);

void led_startupBlink()
{
  static bool led_state = false;
  if (led_tStartupBlink.isFirstIteration())
    pinMode(LED_RED, OUTPUT);

  led_state = !led_state;
  digitalWrite(LED_RED, led_state);
}

void led_startupBlinkOnDisable()
{
  digitalWrite(LED_RED, HIGH);
}

void led_aliveBlink()
{
  static bool led_state = false;
  if (led_tStartupBlink.isFirstIteration())
    pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_BLUE, led_state);
  led_state = !led_state;
  if (led_state)
  {
    led_tAliveBlink.setInterval(10 * TASK_MILLISECOND);
  }
  else
  {
    led_tAliveBlink.setInterval(3 * TASK_SECOND);
  }
}

void led_bleDisconnectedBlink()
{
  static bool led_state = true;
  usb_caps_set_led(led_state);
  led_state = !led_state;
}