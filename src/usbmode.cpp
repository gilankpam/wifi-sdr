// SPDX-License-Identifier: GPL-2.0-only
#include "usbmode.h"

#include <libusb.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

#include "rtlreg.h"
#include "receiver.h"

namespace wsdr {

const char *usb3_switch_name(Usb3Switch r) {
  switch (r) {
  case Usb3Switch::NotNeeded: return "not_needed";
  case Usb3Switch::Unsupported: return "unsupported";
  case Usb3Switch::AlreadyTried: return "already_tried";
  case Usb3Switch::Requested: return "requested";
  case Usb3Switch::Failed: return "failed";
  }
  return "?";
}

namespace {
std::string port_path(libusb_device *d) {
  uint8_t p[8];
  const int n = libusb_get_port_numbers(d, p, sizeof p);
  std::string s;
  for (int i = 0; i < n; i++)
    s += (i ? "." : "") + std::to_string(p[i]);
  return s;
}

Usb3Switch fail(std::string *detail, const std::string &why) {
  if (detail)
    *detail = why;
  return Usb3Switch::Failed;
}

// The switch on an open handle (the caller owns it).
Usb3Switch switch_mode(libusb_device_handle *h, bool usb3, std::string *detail) {
  libusb_set_auto_detach_kernel_driver(h, 1);
  if (const int rc = libusb_claim_interface(h, 0); rc != 0)
    return fail(detail, std::string("claim: ") + libusb_error_name(rc)); // held by someone else: left alone
  // Release on every return (with auto-detach this also reattaches a kernel driver we moved aside).
  struct Release {
    libusb_device_handle *h;
    ~Release() { libusb_release_interface(h, 0); }
  } release{h};
  uint32_t pad = 0; // PAD_CTRL2
  try {
    Reg r(h, false);
    if ((r.r8(0xff) == 0x20) == usb3) // (0x20: running at USB 3)
      return Usb3Switch::NotNeeded;
    pad = r.r32(0xc4);
    if (!(pad & ((3u << 18) | (1u << 20)))) // USB23_SW_MODE / USB3_USB2_TRANSITION
      return Usb3Switch::Unsupported;
    if (usb3 && ((pad >> 18) & 3) == 2)
      return Usb3Switch::AlreadyTried;
    if (usb3)
      r.w8(0x10c3, static_cast<uint8_t>(r.r8(0x10c3) & ~1u));
    pad = (pad & ~(3u << 18)) | ((usb3 ? 2u : 1u) << 18) | (1u << 16); // mode U3 / U2, RSM_EN
    r.w32(0xc4, pad);
    r.w8(0xc5, 4);
    r.w16(0x04, static_cast<uint16_t>(r.r16(0x04) | (1u << 9))); // SYS_PW_CTRL: APFM_OFFMAC
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    pad = r.r32(0xc4);
  } catch (const std::exception &e) {
    return fail(detail, e.what());
  }
  try {
    Reg r(h, false);
    r.w32(0xc4, pad | (1u << 17)); // NO_PDN_CHIPOFF: the device disconnects (this write may not complete)
  } catch (...) {
  }
  return Usb3Switch::Requested;
}

// The switch both ways: halmac's set_usb_mode_88xx (PAD_CTRL2 USB23_SW_MODE 2 = USB 3, 1 = USB 2), which
// rtw88 uses for USB 2 -> 3 (plus its 0x10C3 write) and the vendor driver also for 3 -> 2.
Usb3Switch request_mode(uint16_t pid, int bus, const std::string &port, bool usb3, std::string *detail) {
  if (pid != kPid8812bu)
    return Usb3Switch::Unsupported; // the 8822C is not switched (our 8812CU's efuse forbids it)
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) != 0)
    return fail(detail, "libusb_init failed");
  std::unique_ptr<libusb_context, void (*)(libusb_context *)> ctx_guard(ctx, libusb_exit);
  libusb_device **list = nullptr;
  const ssize_t n = libusb_get_device_list(ctx, &list);
  libusb_device *dev = nullptr;
  for (ssize_t i = 0; i < n; i++) {
    libusb_device_descriptor dd{};
    if (libusb_get_device_descriptor(list[i], &dd) == 0 && realtek_pid(dd.idVendor, dd.idProduct) == pid &&
        libusb_get_bus_number(list[i]) == bus && port_path(list[i]) == port) {
      dev = libusb_ref_device(list[i]);
      break;
    }
  }
  libusb_free_device_list(list, 1);
  if (!dev)
    return fail(detail, "adapter not found");
  std::unique_ptr<libusb_device, void (*)(libusb_device *)> dev_guard(dev, libusb_unref_device);
  if ((libusb_get_device_speed(dev) >= LIBUSB_SPEED_SUPER) == usb3)
    return Usb3Switch::NotNeeded;
  libusb_device_handle *h = nullptr;
  if (const int rc = libusb_open(dev, &h); rc != 0)
    return fail(detail, std::string("open: ") + libusb_error_name(rc));
  std::unique_ptr<libusb_device_handle, void (*)(libusb_device_handle *)> h_guard(h, libusb_close);
  return switch_mode(h, usb3, detail);
}

} // namespace

Usb3Switch request_usb3(uint16_t pid, int bus, const std::string &port, std::string *detail) {
  return request_mode(pid, bus, port, true, detail);
}

Usb3Switch request_usb3_fd(int sys_fd, std::string *detail) {
#ifdef __ANDROID__
  libusb_set_option(nullptr, LIBUSB_OPTION_NO_DEVICE_DISCOVERY); // (process-wide: as in Session::open)
#endif
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) != 0)
    return fail(detail, "libusb_init failed");
  std::unique_ptr<libusb_context, void (*)(libusb_context *)> ctx_guard(ctx, libusb_exit);
  libusb_device_handle *h = nullptr;
  if (const int rc = libusb_wrap_sys_device(ctx, static_cast<intptr_t>(sys_fd), &h); rc != 0)
    return fail(detail, std::string("wrap: ") + libusb_error_name(rc));
  std::unique_ptr<libusb_device_handle, void (*)(libusb_device_handle *)> h_guard(h, libusb_close);
  libusb_device_descriptor dd{};
  if (libusb_get_device_descriptor(libusb_get_device(h), &dd) != 0 || realtek_pid(dd.idVendor, dd.idProduct) != kPid8812bu)
    return Usb3Switch::Unsupported;
  if (libusb_get_device_speed(libusb_get_device(h)) >= LIBUSB_SPEED_SUPER)
    return Usb3Switch::NotNeeded;
  return switch_mode(h, true, detail);
}

Usb3Switch request_usb2(uint16_t pid, int bus, const std::string &port, std::string *detail) {
  return request_mode(pid, bus, port, false, detail);
}

} // namespace wsdr
