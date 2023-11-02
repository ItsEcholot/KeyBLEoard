#include <Arduino.h>

#include "battery.h"
#include "bt.h"
#include "ble_bas_status.h"
#include "led.h"
#include "usb.h"

#include <utility/bonding.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#define BT_SLOT_DIR "/slot"
#define BT_SLOT_PATH "/slot/%u"
#define BT_SLOT_FILENAME_LENGTH sizeof(BT_SLOT_PATH)

BLEDis bledis;
BLEBas blebas;
BLEBasStatus blebasstatus;
BLEHidAdafruit blehid;

TsTask bt_tSetup(TASK_IMMEDIATE, TASK_ONCE, &bt_setup);
TsTask bt_tLoop(10 * TASK_SECOND, TASK_FOREVER, &bt_loop);

BLEConnection *curr_connection;
ble_gap_addr_t curr_addr;
bond_keys_t curr_bond_key;
bool manual_disconnect = false;
bool connect_to_slot = false;
ble_gap_addr_t slots[10];
ble_gap_addr_t curr_slot;

void bt_setup()
{
  Bluefruit.begin(1, 0);

#ifdef _DEBUG_
  printf(PSTR("Printing BLE bonding table"));
  bond_print_list(BLE_GAP_ROLE_PERIPH);
#endif

  Bluefruit.setEventCallback(&bt_on_event);
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(8); // This is overkill
  Bluefruit.setName(BT_MODEL);
  Bluefruit.ScanResponse.addName();

  bledis.setManufacturer(BT_MANUFACTURER);
  bledis.setModel(BT_MODEL);
  bledis.begin();
  blehid.begin();

  blebas.begin();
  blebas.write(100);

  blebasstatus.begin();
  blebasstatus.write(100, false);

  bt_load_slots();

  bt_tSetup.yield(&bt_start_adv);
}

void bt_start_adv()
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addService(blebas);
  Bluefruit.Advertising.addService(blebasstatus);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms, Apple wants fast mode = 20 ms, slow mode = 152.5 ms EXACTLY
                                              // https://developer.apple.com/library/archive/qa/qa1931/_index.html
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.start(0);
  printf(PSTR("Started BLE advertisements\r\n"));

  led_tBLEDisconnectedBlink.enable();

  bt_tLoop.enable();
}

void bt_loop()
{
  static uint8_t last_bat_percentage = 0;
  static bool last_bat_charging = false;
  uint8_t battery_percentage = (uint8_t)round(bat_percentage_ra.getAverage());

  if (battery_percentage != last_bat_percentage)
  {
    last_bat_percentage = battery_percentage;

    blebas.write(battery_percentage);
    blebas.notify(battery_percentage);

    blebasstatus.write(battery_percentage, bat_charging);
    blebasstatus.notify(battery_percentage, bat_charging);
  }

  if (last_bat_charging != bat_charging)
  {
    last_bat_charging = bat_charging;

    blebasstatus.write(battery_percentage, bat_charging);
    blebasstatus.notify(battery_percentage, bat_charging);
  }
}

bool bt_is_connected()
{
  uint8_t null_address[6] = {0, 0, 0, 0, 0, 0};
  return memcmp(curr_bond_key.peer_id.id_addr_info.addr, null_address, sizeof(curr_addr.addr)) != 0;
}

void bt_on_event(ble_evt_t *evt)
{
#ifdef _DEBUG_
  printf(PSTR("BLE Event %i\n"), evt->header.evt_id);
#endif
  switch (evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
  {
    curr_connection = Bluefruit.Connection(evt->evt.common_evt.conn_handle);

    memset(&curr_bond_key, 0, sizeof(bond_keys_t));
  }
  break;
  case BLE_GAP_EVT_CONN_SEC_UPDATE:
  {
    curr_connection = Bluefruit.Connection(evt->evt.common_evt.conn_handle);
    if (curr_connection->loadBondKey(&curr_bond_key))
    {
      if (manual_disconnect)
      {
        for (uint8_t i = 0; i < 10; i++)
        {
          if (memcmp(slots[i].addr, curr_bond_key.peer_id.id_addr_info.addr, sizeof(curr_bond_key.peer_id.id_addr_info.addr)) == 0)
          {
            curr_connection->disconnect();
            return;
          }
        }
      }
      curr_addr = curr_bond_key.peer_id.id_addr_info;
      printf(PSTR("BLE Connected %2.2x\r\n"), curr_addr.addr[5]);
      manual_disconnect = false;
#ifdef _DEBUG_
      printf(PSTR("Loaded bond keys successfully\n"));
      printf(PSTR("Printing BLE bonding table"));
      bond_print_list(BLE_GAP_ROLE_PERIPH);
#endif
    }
  }
  break;
  case BLE_GATTS_EVT_WRITE:
  {
    led_tBLEDisconnectedBlink.disable();
  }
  break;
  case BLE_GAP_EVT_DISCONNECTED:
  {
    uint8_t reason = evt->evt.gap_evt.params.disconnected.reason;
    printf(PSTR("BLE Disconnected reason: %2.2x\r\n"), reason);
    if (reason == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)
    {
      bt_manual_disconnect();
    }
    else if (reason == BLE_HCI_CONN_TERMINATED_DUE_TO_MIC_FAILURE)
    {
      // Remove the current bond key, it is obviously no longer valid.
      bond_remove_key(BLE_GAP_ROLE_PERIPH, &curr_bond_key.peer_id.id_addr_info);
    }

    memset(&curr_bond_key, 0, sizeof(bond_keys_t));
    Bluefruit.Advertising.start(0);
    if (connect_to_slot)
      bt_set_adv_data_for_curr_slot();
    led_tBLEDisconnectedBlink.enable();
  }
  break;
  }
}

void bt_on_key(uint8_t mod, uint8_t *keys)
{
  blehid.keyboardReport(mod, keys);
}

void bt_manual_disconnect()
{
  manual_disconnect = true;
  connect_to_slot = false;
  if (bt_is_connected())
  {
    curr_connection->disconnect();
  }
}

void bt_load_slots()
{
  for (uint8_t i = 1; i <= 10; i++)
  {
    char filename[BT_SLOT_FILENAME_LENGTH];
    sprintf(filename, BT_SLOT_PATH, i);

    if (!InternalFS.exists(BT_SLOT_DIR) || !InternalFS.exists(filename))
      continue;

    Adafruit_LittleFS_Namespace::File file(InternalFS);
    if (!file.open(filename, Adafruit_LittleFS_Namespace::FILE_O_READ))
      printf(PSTR("Couldn't open slot"));

    file.read(&slots[i - 1], sizeof(curr_slot));
    file.close();
  }
}

void bt_learn_device(uint8_t slot)
{
  printf(PSTR("Learning into slot %u\r\n"), slot);
  usb_caps_set_led(true);

  memcpy(&curr_addr, &slots[slot - 1], sizeof(curr_addr));

  char filename[BT_SLOT_FILENAME_LENGTH];
  sprintf(filename, BT_SLOT_PATH, slot);

  if (!InternalFS.exists(BT_SLOT_DIR))
    InternalFS.mkdir(BT_SLOT_DIR);
  if (InternalFS.exists(filename))
    InternalFS.remove(filename);

  Adafruit_LittleFS_Namespace::File file(filename, Adafruit_LittleFS_Namespace::FILE_O_WRITE, InternalFS);
  if (!(file))
    printf(PSTR("Failed to create File obj for learning device\r\n"));

  file.write((uint8_t const *)&curr_addr, sizeof(curr_addr));
  file.close();
  memcpy(&curr_addr, &curr_slot, sizeof(curr_addr));
  connect_to_slot = true;
  manual_disconnect = false;
  led_tBLESlotSaveDone.enableDelayed(1 * TASK_SECOND);
}

void bt_select_slot(uint8_t slot)
{
  manual_disconnect = false;

  char filename[BT_SLOT_FILENAME_LENGTH];
  sprintf(filename, BT_SLOT_PATH, slot);

  if (!InternalFS.exists(BT_SLOT_DIR) || !InternalFS.exists(filename))
    return;

  Adafruit_LittleFS_Namespace::File file(InternalFS);
  if (!file.open(filename, Adafruit_LittleFS_Namespace::FILE_O_READ))
    printf(PSTR("Couldn't open slot"));

  file.read(&curr_slot, sizeof(curr_slot));
  file.close();
  printf(PSTR("Connecting to slot %u: %2.2x\r\n"), slot, curr_slot.addr[5]);
  connect_to_slot = true;

  if (bt_is_connected())
    curr_connection->disconnect();
}

void bt_set_adv_data_for_curr_slot()
{
  uint8_t handle = 0;
  sd_ble_gap_adv_stop(handle);

  bond_keys_t slot_bond_key;
  bool res = bond_load_keys(BLE_GAP_ROLE_PERIPH, &curr_slot, &slot_bond_key);

#ifdef _DEBUG_
  printf("Loaded key %s, %2.2x irk %2.2x", res ? "true" : "false\r\n", slot_bond_key.peer_id.id_addr_info.addr[5], slot_bond_key.peer_id.id_info.irk[15]);
#endif

  const ble_gap_id_key_t *p_key1 = &slot_bond_key.peer_id;
  const ble_gap_id_key_t *const *pp_id_keys = {&p_key1};
  uint32_t result = sd_ble_gap_device_identities_set(pp_id_keys, NULL, 1);
#ifdef _DEBUG_
  printf(PSTR("device identities set 0x%08x\r\n"), result);
#endif

  curr_slot.addr_id_peer = 1;
  const ble_gap_addr_t *const whitelist_data = &curr_slot;
  const ble_gap_addr_t *const *whitelist = &whitelist_data;
  result = sd_ble_gap_whitelist_set(whitelist, 1);
#ifdef _DEBUG_
  printf(PSTR("whitelist set 0x%08x\r\n"), result);
#endif

  printf(PSTR("Set ADV data for %2.2x type %2.2x, resolved? %2.2x\r\n"), curr_slot.addr[5], curr_slot.addr_type, curr_slot.addr_id_peer);

  ble_gap_adv_params_t adv_params_directed = {
      .properties = {.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED, .anonymous = 0},
      .p_peer_addr = NULL,                    // Undirected to current slot
      .interval = BLE_ADV_INTERVAL_FAST_DFLT, // advertising interval (in units of 0.625 ms)
      .duration = 0,                          //(uint16_t)(BLE_ADV_FAST_TIMEOUT_DFLT * 100), // in 10-ms unit

      .max_adv_evts = 0,               // TODO can be used for fast/slow mode
      .channel_mask = {0, 0, 0, 0, 0}, // 40 channel, set 1 to disable
      .filter_policy = BLE_GAP_ADV_FP_FILTER_BOTH,

      .primary_phy = BLE_GAP_PHY_AUTO,   // 1 Mbps will be used
      .secondary_phy = BLE_GAP_PHY_AUTO, // 1 Mbps will be used
                                         // , .set_id, .scan_req_notification
  };

  // gap_adv long-live is required by SD v6
  static ble_gap_adv_data_t gap_adv = {
      .adv_data = {.p_data = Bluefruit.Advertising.getData(), .len = Bluefruit.Advertising.count()},
      .scan_rsp_data = {.p_data = Bluefruit.ScanResponse.getData(), .len = Bluefruit.ScanResponse.count()}};
  result = sd_ble_gap_adv_set_configure(&handle, &gap_adv, &adv_params_directed);
#ifdef _DEBUG_
  printf(PSTR("adv set 0x%08x\r\n"), result);
#endif
  result = sd_ble_gap_adv_start(handle, CONN_CFG_PERIPHERAL);
#ifdef _DEBUG_
  printf(PSTR("adv start 0x%08x\r\n"), result);
#endif
}