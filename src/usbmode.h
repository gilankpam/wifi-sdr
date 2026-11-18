// SPDX-License-Identifier: GPL-2.0-only
// USB 2 -> USB 3 mode switch for the RTL8822B (RTL8812BU), as rtw88 does at probe
// (rtw_usb_switch_mode_new; the register sequence as in ref/ac54_sdr/tools/usb3.c). The chip
// enumerates at USB 2 after a cold plug; once switched it re-enumerates as SuperSpeed and stays so
// until it loses power. Register writes only (RAM): nothing is written to efuse or flash.
// If the port or cable has no USB 3 lines the device comes back at USB 2, and the chip records the
// attempt (PAD_CTRL2 mode = 2), so it is not retried until the next power cycle.
#pragma once

#include <cstdint>
#include <string>

namespace wsdr {

enum class Usb3Switch {
  NotNeeded,    // already SuperSpeed
  Unsupported,  // not an 8822B, or the chip does not advertise mode switching
  AlreadyTried, // a switch was attempted since power-up (USB 2 port or cable?)
  Requested,    // written; the device is re-enumerating now (rescan in ~1-3 s)
  Failed,       // could not open / talk to the device (detail says why)
};
const char *usb3_switch_name(Usb3Switch r);

// The adapter at bus/port (port path "1" or "1.2"). Opens it briefly; nothing else may hold it.
Usb3Switch request_usb3(uint16_t pid, int bus, const std::string &port, std::string *detail = nullptr);
// Back to USB 2 without a replug (the vendor driver's 3 -> 2 switch; tests of the USB 2 paths). The
// chip then comes up at USB 2, and request_usb3 can switch it again.
Usb3Switch request_usb2(uint16_t pid, int bus, const std::string &port, std::string *detail = nullptr);
// The USB 3 switch on an already-open usbfs fd (Android UsbDeviceConnection); no bus scan. The fd's device
// disconnects on Requested: close the connection and wait for the new (SuperSpeed) device.
Usb3Switch request_usb3_fd(int sys_fd, std::string *detail = nullptr);

} // namespace wsdr
