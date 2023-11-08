#pragma once

#include <TSchedulerDeclarations.hpp>
#include <bluefruit.h>

#define BT_MANUFACTURER "Echolot"
#define BT_MODEL "KeyBLEoard"

extern TsTask bt_tSetup;
extern TsTask bt_tLoop;

void bt_setup();
void bt_start_adv();

void bt_loop();
void bt_on_event(ble_evt_t* evt);
void bt_on_key(uint8_t mod, uint8_t *keys);
void bt_on_consumer(uint16_t value);

void bt_manual_disconnect();

void bt_load_slots();
void bt_learn_device(uint8_t slot);
void bt_select_slot(uint8_t slot);
void bt_set_adv_data_for_curr_slot();