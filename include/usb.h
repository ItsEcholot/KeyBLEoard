#pragma once

#define PIN_MAX3421E_CS 6
#define PIN_L_ALT 1
#define PLL_RESET_COUNT 2

#include <TSchedulerDeclarations.hpp>
#include <hidboot.h>

class KbdRptParser : public KeyboardReportParser {
  #ifdef _DEBUG_
    void PrintKey(uint8_t mod, uint8_t key);
  #endif

  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

  protected:
    void OnControlKeysChanged(uint8_t before, uint8_t after);
    void OnKey(uint8_t mod, uint8_t *keys);
    void OnKeyDown(uint8_t mod, uint8_t key);
    void OnKeyUp(uint8_t mod, uint8_t key);
};

extern TsTask usb_tSetup;
extern TsTask usb_tLoop;

// Setup tasks
void usb_setup();
void usb_read_max_revision();
void usb_pll_test();
void usb_setup_hid_keyboard();

// Main loop
void usb_loop();

void usb_caps_set_led(bool on);
void usb_caps_reset_led();