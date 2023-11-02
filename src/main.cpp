#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#include "led.h"
#include "battery.h"
#include "usb.h"
#include "bt.h"

TsScheduler ts;

#ifdef _DEBUG_
void main_cpu_load();
TsTask main_tCpu_load(5 * TASK_SECOND, TASK_FOREVER, &main_cpu_load);
#endif

void setup() {
  Serial.begin(115200);
#ifdef _DEBUG_
  //while(!Serial);
#endif

  ts.init();

  led_tStartupBlink.setSelfDestruct();
  led_tAliveBlink.setSelfDestruct();
  ts.addTask(led_tStartupBlink);
  ts.addTask(led_tAliveBlink);
  ts.addTask(led_tBLEDisconnectedBlink);
  ts.addTask(led_tBLESlotSaveDone);
  led_tStartupBlink.enable();

  bat_tSetup.setSelfDestruct();
  ts.addTask(bat_tSetup);
  ts.addTask(bat_tLoop);
  bat_tSetup.enable();

  usb_tSetup.setSelfDestruct();
  ts.addTask(usb_tSetup);
  ts.addTask(usb_tLoop);
  usb_tSetup.enable();

  bt_tSetup.setSelfDestruct();
  ts.addTask(bt_tSetup);
  ts.addTask(bt_tLoop);
  bt_tSetup.enableDelayed(500 * TASK_MILLISECOND);

#ifdef _DEBUG_
  // ts.addTask(main_tCpu_load);
  // main_tCpu_load.enable();
#endif
}

void loop() {
  ts.execute();
}

#ifdef _DEBUG_
void main_cpu_load()
{
  unsigned long cpuTot = ts.getCpuLoadTotal();
  unsigned long cpuCyc = ts.getCpuLoadCycle();
  unsigned long cpuIdl = ts.getCpuLoadIdle();

  // Serial.print("Total CPU time="); Serial.print(cpuTot); Serial.println(" micros");
  // Serial.print("Scheduling Overhead CPU time="); Serial.print(cpuCyc); Serial.println(" micros");
  // Serial.print("Idle Sleep CPU time="); Serial.print(cpuIdl); Serial.println(" micros");
  // Serial.print("Productive work CPU time="); Serial.print(cpuTot - cpuIdl - cpuCyc); Serial.println(" micros");

  float idle = (float)cpuIdl / (float)cpuTot * 100;
  Serial.print("CPU Idle Sleep "); Serial.print(idle); Serial.println(" % of time.");

  float prod = (float)(cpuIdl + cpuCyc) / (float)cpuTot * 100;
  Serial.print("Prod work "); Serial.print(100.00 - prod); Serial.println(" % of time.");


  ts.cpuLoadReset();
}
#endif