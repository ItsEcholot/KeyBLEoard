#pragma once

#include "bluefruit_common.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

union ble_battery_status
{
  struct
  {
    struct
    {
      bool identifier_present : 1;
      bool battery_level_present : 1;
      bool additional_status_present : 1;
      uint8_t reserved : 5;
    } flags;
    struct
    {
      bool battery_present : 1;
      bool wired_external_power_source_connected : 2;
      bool wireless_external_power_source_connected : 2;
      uint8_t battery_charge_state : 2;
      uint8_t battery_charge_level : 2;
      uint16_t charging_type : 3;
      uint8_t charging_fault_reason : 3;
      bool reserved : 1;
    } power_state;
    uint8_t battery_level;
  } fields;
  uint32_t bits;
};

class BLEBasStatus : public BLEService
{
protected:
  BLECharacteristic _battery;
  uint32_t get_packet(uint8_t level, bool charging);

public:
  BLEBasStatus(void);

  virtual err_t begin(void);

  bool write(uint8_t level, bool charging);
  bool notify(uint8_t level, bool charging);
};