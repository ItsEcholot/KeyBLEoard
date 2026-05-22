#include "bluefruit.h"
#include "ble_bas_status.h"

// NOTE: This service is intentionally *not* UUID16_SVC_BATTERY (0x180F).
// Having multiple 0x180F Battery Service instances can confuse some clients
// (notably newer macOS), causing them to miss the standard 0x2A19 Battery Level.
// We keep the standard Battery Level in BLEBas and expose extended status via a
// vendor-specific service.
#define UUID16_SVC_BATTERY_LEVEL_STATUS 0xFFF0
#define UUID16_CHR_BATTERY_LEVEL_STATUS 0xFFF1

BLEBasStatus::BLEBasStatus(void) :
  BLEService(UUID16_SVC_BATTERY_LEVEL_STATUS), _battery(UUID16_CHR_BATTERY_LEVEL_STATUS)
{

}

err_t BLEBasStatus::begin(void)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  _battery.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  _battery.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _battery.setFixedLen(4);
  VERIFY_STATUS( _battery.begin() );

  return ERROR_NONE;
}

bool BLEBasStatus::write(uint8_t level, bool charging)
{
  return _battery.write32(get_packet(level, charging)) > 0;
}

bool BLEBasStatus::notify(uint8_t level, bool charging)
{
  return _battery.notify32(get_packet(level, charging));
}

uint32_t BLEBasStatus::get_packet(uint8_t level, bool charging)
{
  union ble_battery_status packet = {0};
  packet.fields.flags.battery_level_present = true;

  packet.fields.power_state.battery_present = true;
  packet.fields.power_state.wired_external_power_source_connected = charging;
  packet.fields.power_state.wireless_external_power_source_connected = false;
  packet.fields.power_state.battery_charge_state = charging ? 0x01 : 0x02;
  packet.fields.power_state.battery_charge_level = level < 20 ? (level < 10 ? 0x03 : 0x02) : 0x01;
  packet.fields.power_state.charging_type = 0x00;

  packet.fields.battery_level = level;

  return packet.bits;
}
