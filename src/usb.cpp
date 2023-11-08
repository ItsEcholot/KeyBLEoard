#include <Arduino.h>

#include "usb.h"
#include "led.h"
#include "bt.h"

#include <usbhub.h>
#include <hidboot.h>
#include <SPI.h>

USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> HidKeyboard(&Usb);
KbdRptParser Parser;

TsTask usb_tSetup(TASK_IMMEDIATE, TASK_ONCE, &usb_setup);
TsTask usb_tLoop(1000 - 30, TASK_FOREVER, &usb_loop); // The USB loop doesn't do anything if run faster than every 975us,
                                                      // 5us to give scheduler some time

uint8_t usb_curr_state;
uint8_t usb_last_state;
bool usb_capslock_state;
volatile uint8_t selecting_slot = 0;
volatile uint16_t selecting_slot_counter = 0;

uint8_t pressed_keycodes[6];

void usb_setup()
{
  pinMode(PIN_MAX3421E_CS, OUTPUT);
  digitalWrite(PIN_MAX3421E_CS, HIGH);
  pinMode(PIN_L_ALT, INPUT);

  Usb.Init();
  usb_tSetup.yield(&usb_read_max_revision);
}

void usb_read_max_revision()
{
  uint8_t rev = Usb.regRd(rREVISION);
  printf(PSTR("USB Host die revision "));
  switch (rev)
  {
  case (0x01): // rev.01
    printf(PSTR("01\r\n"));
    break;
  case (0x12): // rev.02
    printf(PSTR("02\r\n"));
    break;
  case (0x13): // rev.03
    printf(PSTR("03\r\n"));
    break;
  default:
    printf(PSTR("invalid. Value returned: %2.2x\r\n"), rev);
    break;
  }
  usb_tSetup.yield(&usb_pll_test);
}

void usb_pll_test()
{
  if (usb_tSetup.isFirstIteration())
    usb_tSetup.setIterations(PLL_RESET_COUNT);

  if (!usb_tSetup.isFirstIteration() && Usb.regRd(rUSBIRQ) & bmOSCOKIRQ)
  {
    usb_tSetup.yield(&usb_pll_test);
    return;
  }

  if (usb_tSetup.isLastIteration())
  {
    printf(PSTR("PLL stabilized\r\n"));
    usb_tSetup.setIterations(TASK_ONCE);
    usb_tSetup.enable(); // Reset counter
    usb_tSetup.yield(&usb_setup_hid_keyboard);
    return;
  }

  printf(PSTR("Resetting oscillator\r\n"));
  Usb.regWr(rUSBCTL, bmCHIPRES); // reset

  if (Usb.regRd(rUSBIRQ) & bmOSCOKIRQ)
  { // wrong state - should be off
    printf(PSTR("\r\n!!! Current oscillator state unexpected.\r\n"));
  }

  Usb.regWr(rUSBCTL, 0x00); // release from reset
}

void usb_setup_hid_keyboard()
{
  if (usb_tSetup.isLastIteration())
  {
    printf(PSTR("Setting HID Keyboard report parser\r\n"));
    HidKeyboard.SetReportParser(0, &Parser);
    led_tStartupBlink.disable();
    led_tAliveBlink.enable();
    usb_tLoop.enableDelayed(1000 * TASK_MILLISECOND);
  }
  else
  {
    if (Usb.Init() == -1)
    {
      printf(PSTR("\r\n!!! OSCOKIRQ failed to assert\r\n"));
    }
    printf(PSTR("Sending bus reset\r\n"));
    Usb.regWr(rHCTL, bmBUSRST);

    usb_tSetup.delay(102 * TASK_MILLISECOND);
  }
}

void usb_loop()
{
  Usb.Task();

  usb_curr_state = Usb.getUsbTaskState();
  if (usb_curr_state != usb_last_state || usb_curr_state == USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE)
  {
    usb_last_state = usb_curr_state;
    switch (usb_curr_state)
    {
    case (USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE):
#ifdef _DEBUG_
      E_Notify(PSTR("\r\nWaiting for device..."), 0x80);
#endif
      Usb.regWr(rUSBCTL, bmCHIPRES);
      Usb.regWr(rUSBCTL, 0x00);
      while (Usb.regRd(rUSBIRQ) & bmOSCOKIRQ)
      {
        yield();
      }
      Usb.Init();
      delay(50);
      break;
#ifdef _DEBUG_
    case (USB_ATTACHED_SUBSTATE_RESET_DEVICE):
      E_Notify(PSTR("\r\nDevice connected. Resetting..."), 0x80);
      break;
    case (USB_ATTACHED_SUBSTATE_WAIT_SOF):
      E_Notify(PSTR("\r\nReset complete. Waiting for the first SOF..."), 0x80);
      break;
    case (USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE):
      E_Notify(PSTR("\r\nSOF generation started. Enumerating device..."), 0x80);
      break;
    case (USB_STATE_ADDRESSING):
      E_Notify(PSTR("\r\nSetting device address..."), 0x80);
      break;
    case (USB_STATE_RUNNING):
    {
      E_Notify(PSTR("\r\nGetting device descriptor"), 0x80);
      USB_DEVICE_DESCRIPTOR buf;
      uint8_t rcode = Usb.getDevDescr(1, 0, sizeof(USB_DEVICE_DESCRIPTOR), (uint8_t *)&buf);

      if (rcode)
        printf(PSTR("\r\nError reading device descriptor. Error code %2.2x\r\n"), rcode);
      else
      {
        /**/
        printf(PSTR("\r\nDescriptor Length: %2.2x\t"), buf.bLength);
        printf(PSTR("\r\nDescriptor type: %2.2x\t"), buf.bDescriptorType);
        printf(PSTR("\r\nUSB version: %2.2x\t\t"), buf.bcdUSB);
        printf(PSTR("\r\nDevice class: %2.2x\t\t"), buf.bDeviceClass);
        printf(PSTR("\r\nDevice Subclass: %2.2x\t"), buf.bDeviceSubClass);
        printf(PSTR("\r\nDevice Protocol: %2.2x\t"), buf.bDeviceProtocol);
        printf(PSTR("\r\nMax.packet size: %2.2x\t"), buf.bMaxPacketSize0);
        printf(PSTR("\r\nVendor  ID: %2.2x\t\t"), buf.idVendor);
        printf(PSTR("\r\nProduct ID: %2.2x\t\t"), buf.idProduct);
        printf(PSTR("\r\nRevision ID: %2.2x\t\t"), buf.bcdDevice);
        printf(PSTR("\r\nMfg.string index: %2.2x\t"), buf.iManufacturer);
        printf(PSTR("\r\nProd.string index: %2.2x\t"), buf.iProduct);
        printf(PSTR("\r\nSerial number index: %2.2x\t"), buf.iSerialNumber);
        printf(PSTR("\r\nNumber of conf.: %2.2x\t"), buf.bNumConfigurations);
        /**/
        printf(PSTR("\r\n\nAll tests passed.\r\n"));
      }
      break;
    }
    case (USB_STATE_ERROR):
      E_Notify(PSTR("\r\nUSB state machine reached error state"), 0x80);
      break;
#endif

    default:
      break;
    }
  }

  if (selecting_slot > 0)
  {
    if (selecting_slot_counter > 2000)
    {
      bt_learn_device(selecting_slot);
      selecting_slot = 0;
      selecting_slot_counter = 0;
    }

    selecting_slot_counter++;
  }
  else
  {
    selecting_slot_counter = 0;
  }
}

void usb_caps_set_led(bool on)
{
  if (usb_last_state != USB_STATE_RUNNING)
    return;

  union
  {
    KBDLEDS led_status = {0};
    uint8_t bLeds;
  } led_status;
  led_status.led_status.bmCapsLock = on;
  HidKeyboard.SetReport(0, 0, HID_REPORT_TYPE_OUTPUT, 0, 1, &led_status.bLeds);
}

void usb_caps_reset_led()
{
  usb_caps_set_led(usb_capslock_state);
}

void KbdRptParser::Parse(USBHID *hid, bool is_rpt_id __attribute__((unused)), uint8_t len __attribute__((unused)), uint8_t *buf)
{
  // On error - return
  if (buf[2] == 1)
    return;

  // KBDINFO       *pki = (KBDINFO*)buf;

  // Switch GUI and ALT keys for Mac
  MODIFIERKEYS modKeys;
  *((uint8_t *)&modKeys) = buf[0x00];
  uint8_t left_gui_state = modKeys.bmLeftGUI;
  modKeys.bmLeftGUI = modKeys.bmLeftAlt;
  modKeys.bmLeftAlt = left_gui_state;
  modKeys.bmRightGUI = modKeys.bmRightAlt;
  modKeys.bmRightAlt = 0;
  buf[0x00] = *((uint8_t *)&modKeys);

#ifdef _DEBUG_
  // provide event for changed control key state
  if (prevState.bInfo[0x00] != buf[0x00])
  {
    OnControlKeysChanged(prevState.bInfo[0x00], buf[0x00]);
  }
#endif

  for (uint8_t i = 2; i < 8; i++)
  {
    bool down = false;
    bool up = false;

    for (uint8_t j = 2; j < 8; j++)
    {
      if (buf[i] == prevState.bInfo[j] && buf[i] != 1)
        down = true;
      if (buf[j] == prevState.bInfo[i] && prevState.bInfo[i] != 1)
        up = true;
    }
    if (!down)
    {
      HandleLockingKeys(hid, buf[i]);
      OnKeyDown(*buf, buf[i]);
      if (buf[i] == UHS_HID_BOOT_KEY_CAPS_LOCK)
      {
        usb_capslock_state = !usb_capslock_state;
      }
    }
    if (!up)
      OnKeyUp(prevState.bInfo[0], prevState.bInfo[i]);
  }
  for (uint8_t i = 0; i < 8; i++)
    prevState.bInfo[i] = buf[i];

  OnKey(buf[0], prevState.kbdInfo.Keys);
};

void KbdRptParser::OnKey(uint8_t mod, uint8_t *keys)
{
  uint16_t consumer_value = 0;
  bool key_application_pressed = false;
  bool key_escape_pressed = false;
  int8_t key_w_pressed = -1;
  int8_t key_a_pressed = -1;
  int8_t key_s_pressed = -1;
  int8_t key_d_pressed = -1;
  bool key_number_pressed[11] = {0}; // First value true if any number key true
  bool key_fs_pressed[13] = {0}; // First value true if any F-Key true

  for (uint8_t i = 0; i < 6; i++)
  {
    switch (keys[i])
    {
    case HID_KEY_APPLICATION:
      key_application_pressed = true;
      keys[i] = 0;
      break;
    case HID_KEY_ESCAPE:
      key_escape_pressed = true;
      break;
    case HID_KEY_W:
      key_w_pressed = i;
      break;
    case HID_KEY_A:
      key_a_pressed = i;
      break;
    case HID_KEY_S:
      key_s_pressed = i;
      break;
    case HID_KEY_D:
      key_d_pressed = i;
      break;
    }

    if (keys[i] >= HID_KEY_1 && keys[i] <= HID_KEY_0)
    {
      key_number_pressed[0] = true;
      key_number_pressed[keys[i] - HID_KEY_1 + 1] = true;
    }

    if (keys[i] >= HID_KEY_F1 && keys[i] <= HID_KEY_F12)
    {
      key_fs_pressed[0] = true;
      key_fs_pressed[keys[i] - HID_KEY_F1 + 1] = true;
    }
  }

  // Key Mutations ----------------------------------
  if (key_application_pressed)
  {
    // Arrow keys
    if (key_w_pressed != -1)
      keys[key_w_pressed] = HID_KEY_ARROW_UP;
    if (key_a_pressed != -1)
      keys[key_a_pressed] = HID_KEY_ARROW_LEFT;
    if (key_s_pressed != -1)
      keys[key_s_pressed] = HID_KEY_ARROW_DOWN;
    if (key_d_pressed != -1)
      keys[key_d_pressed] = HID_KEY_ARROW_RIGHT;

    // Media keys
    if (key_fs_pressed[0])
    {
      memset(keys, 0, sizeof(keys));
      for (uint8_t i = 1; i < sizeof(key_fs_pressed); i++)
      {
        if (key_fs_pressed[i])
          switch (i)
          {
          case 1:
            consumer_value = 0x070; // Brightness down
            break;
          case 2:
            consumer_value = 0x06F; // Brightness up
            break;
          case 3:
            consumer_value = 0x29F; // Mission-control
            break;
          case 4:
            consumer_value = 0x221; // Search
            break;
          case 5:
            consumer_value = 0x0CF; // Dictation (Kinda not working?)
            break;
          case 6:
            consumer_value = 0x19E; // Lock? (Not working)
            break;
          case 7:
            consumer_value = 0x0B4; // Rewind
            break;
          case 8:
            consumer_value = 0x0CD; // Play / pause
            break;
          case 9:
            consumer_value = 0x0B3; // Fast-forward
            break;
          case 10:
            consumer_value = 0x0E2; // Volume mute
            break;
          case 11:
            consumer_value = 0x0EA; // Volume down
            break;
          case 12:
            consumer_value = 0x0E9; // Volume up
            break;

          default:
            break;
          }
      }
    }

    printf("Media Keys: %" SCNd16 "\r\n", consumer_value);
  }
  // Key Mutations ----------------------------------
  // Shortcuts --------------------------------------

  // Was pressed but released before learning could start
  if (selecting_slot > 0 && !key_number_pressed[0])
    bt_select_slot(selecting_slot);
  if (!key_number_pressed[0])
    selecting_slot = 0;

  if (key_application_pressed)
  {
    if (key_escape_pressed)
      bt_manual_disconnect();
    else if (key_number_pressed[0])
    {
      for (uint8_t i = 1; i < sizeof(key_number_pressed); i++)
        if (key_number_pressed[i])
          selecting_slot = i;
      return;
    }
  }
  // Shortcuts --------------------------------------

  bt_on_consumer(consumer_value);
  bt_on_key(mod, keys);
}

void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
#ifdef _DEBUG_
  Serial.print("DN ");
  PrintKey(mod, key);

  uint8_t c = OemToAscii(mod, key);

  if (c)
  {
    Serial.print("ASCII: ");
    Serial.println((char)key);
  }
#endif
}

void KbdRptParser::OnKeyUp(uint8_t mod, uint8_t key)
{
#ifdef _DEBUG_
  Serial.print("UP ");
  PrintKey(mod, key);
#endif
}

void KbdRptParser::OnControlKeysChanged(uint8_t before, uint8_t after)
{
#ifdef _DEBUG_
  MODIFIERKEYS beforeMod;
  *((uint8_t *)&beforeMod) = before;

  MODIFIERKEYS afterMod;
  *((uint8_t *)&afterMod) = after;
  if (beforeMod.bmLeftCtrl != afterMod.bmLeftCtrl)
  {
    Serial.println("LeftCtrl changed");
  }
  if (beforeMod.bmLeftShift != afterMod.bmLeftShift)
  {
    Serial.println("LeftShift changed");
  }
  if (beforeMod.bmLeftAlt != afterMod.bmLeftAlt)
  {
    Serial.println("LeftAlt changed");
  }
  if (beforeMod.bmLeftGUI != afterMod.bmLeftGUI)
  {
    Serial.println("LeftGUI changed");
  }

  if (beforeMod.bmRightCtrl != afterMod.bmRightCtrl)
  {
    Serial.println("RightCtrl changed");
  }
  if (beforeMod.bmRightShift != afterMod.bmRightShift)
  {
    Serial.println("RightShift changed");
  }
  if (beforeMod.bmRightAlt != afterMod.bmRightAlt)
  {
    Serial.println("RightAlt changed");
  }
  if (beforeMod.bmRightGUI != afterMod.bmRightGUI)
  {
    Serial.println("RightGUI changed");
  }
#endif
}

void KbdRptParser::PrintKey(uint8_t m, uint8_t key)
{
  MODIFIERKEYS mod;
  *((uint8_t *)&mod) = m;
  Serial.print((mod.bmLeftCtrl == 1) ? "C" : " ");
  Serial.print((mod.bmLeftShift == 1) ? "S" : " ");
  Serial.print((mod.bmLeftAlt == 1) ? "A" : " ");
  Serial.print((mod.bmLeftGUI == 1) ? "G" : " ");

  Serial.print(" >");
  PrintHex<uint8_t>(key, 0x80);
  Serial.print("< ");

  Serial.print((mod.bmRightCtrl == 1) ? "C" : " ");
  Serial.print((mod.bmRightShift == 1) ? "S" : " ");
  Serial.print((mod.bmRightAlt == 1) ? "A" : " ");
  Serial.println((mod.bmRightGUI == 1) ? "G" : " ");
};