// SPDX-License-Identifier: GPL-2.0-only
#include "session.h"
#include "receiver.h" // model_pid

#include <stdexcept>

#include "UsbOpen.h"
#include "WiFiDriver.h"

namespace wsdr {



static ChannelWidth_t width_of(int bw) {
  switch (bw) {
  case 40: return CHANNEL_WIDTH_40;
  case 80: return CHANNEL_WIDTH_80;
  default: return CHANNEL_WIDTH_20;
  }
}

Session::Session(Logger_t logger) : _log(std::move(logger)), _s(_log) {}

Session::~Session() {
  _reg.reset();
  _s.close();
}

void Session::open(const OpenOpts &o) {
  if (!o.pid)
    throw std::invalid_argument("pid required");
  libusb_context *ctx = nullptr;
  if (o.sys_fd >= 0) // no bus scan (Android: the app may only touch the device it was granted)
    libusb_set_option(nullptr, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
  if (libusb_init(&ctx) < 0)
    throw std::runtime_error("libusb_init failed");
  _s.adopt_context(ctx);
  libusb_device_handle *h = nullptr;
  if (o.sys_fd >= 0) {
    const int rc = libusb_wrap_sys_device(ctx, static_cast<intptr_t>(o.sys_fd), &h);
    if (rc != 0)
      throw OpenError(OpenError::Other, std::string("libusb_wrap_sys_device: ") + libusb_error_name(rc));
    libusb_device_descriptor d{};
    if (libusb_get_device_descriptor(libusb_get_device(h), &d) != 0 ||
        (o.any_board ? model_pid(d.idVendor, d.idProduct) != o.pid : d.idProduct != o.pid)) {
      libusb_close(h);
      throw OpenError(OpenError::NotFound, "the fd is not the requested adapter");
    }
  } else {
    libusb_device **list = nullptr;
    const ssize_t n = libusb_get_device_list(ctx, &list);
    int rc = LIBUSB_ERROR_NOT_FOUND;
    for (ssize_t i = 0; i < n && rc == LIBUSB_ERROR_NOT_FOUND; i++) {
      libusb_device_descriptor d{};
      if (libusb_get_device_descriptor(list[i], &d) != 0 ||
          (o.any_board ? model_pid(d.idVendor, d.idProduct) != o.pid : d.idVendor != o.vid || d.idProduct != o.pid))
        continue;
      if (o.bus >= 0 && libusb_get_bus_number(list[i]) != o.bus)
        continue;
      if (!o.port.empty()) {
        uint8_t ports[8];
        const int np = libusb_get_port_numbers(list[i], ports, sizeof ports);
        std::string p;
        for (int k = 0; k < np; k++)
          p += (k ? "." : "") + std::to_string(ports[k]);
        if (p != o.port)
          continue;
      }
      rc = libusb_open(list[i], &h);
    }
    if (list)
      libusb_free_device_list(list, 1);
    if (rc == LIBUSB_ERROR_NOT_FOUND)
      throw OpenError(OpenError::NotFound, "adapter not found");
    if (rc == LIBUSB_ERROR_ACCESS)
      throw OpenError(OpenError::Permission, "permission denied opening the adapter (udev rule missing?)");
    if (rc != 0)
      throw OpenError(OpenError::Other, std::string("libusb_open: ") + libusb_error_name(rc));
  }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), _log, o.reset,
                                           lock) != 0) {
    _s.adopt_handle(h);
    throw OpenError(OpenError::Busy, "claim failed (adapter busy?)");
  }
  _s.adopt_handle(h);
  _s.adopt_lock(lock);
  _ss = libusb_get_device_speed(libusb_get_device(h)) >= LIBUSB_SPEED_SUPER;

  devourer::DeviceConfig cfg;
  cfg.tuning.thermal_track = o.thermal_track;
  // Jaguar3 (8822C/E): devourer starts a coex/phydm runtime thread after bring-up that keeps a bulk-IN
  // read posted (it steals C2H packets from our reader) and rewrites DIG/IGI every ~2 s. All wifi-sdr
  // users own bulk-IN and want fixed gain. The switch (tuning.j3_runtime_thread, OpenIPC/devourer#498)
  // is required: the pinned devourer (external/devourer) has it, and a build without it must fail.
  cfg.tuning.j3_runtime_thread = false;
  WiFiDriver driver(_log);
  auto dev = driver.CreateRadio(h, ctx, lock, cfg);
  if (!dev)
    throw std::runtime_error("CreateRadio failed");
  _s.adopt_device(std::move(dev));
  _chip = radio()->GetAdapterCaps().chip_name;
  radio()->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(o.channel),
                                     .ChannelOffset = static_cast<uint8_t>(o.bw > 20 ? 1 : 0),
                                     .ChannelWidth = width_of(o.bw)});
  _reg = std::make_unique<Reg>(h, _ss);
}

void Session::tune(int channel, int bw) {
  radio()->SetMonitorChannel(SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                             .ChannelOffset = static_cast<uint8_t>(bw > 20 ? 1 : 0),
                                             .ChannelWidth = width_of(bw)});
  // The 8822B RF synth re-latches its channel state only on a change of RF18's channel byte (devourer's
  // set_channel_bw, the 5/10 MHz comment). A width change on the same central channel (40 MHz at primary
  // 152 -> 20 MHz at 154: both 154) rewrites only the BW bits, and RX stays deaf until the next channel
  // change. MEASURED on an 8812BU: path-A RMS 2.9 after the 40 -> 20 switch, 158 with this edge. So every
  // retune writes a different channel byte first, then the real value, on both paths (3-wire LSSI writes
  // as devourer's rf_write; RF18 read back through the direct window).
  if (_chip == "RTL8822B") {
    for (int path = 0; path < 2; path++) {
      const uint16_t lssi = path ? 0x0E90 : 0x0C90;
      const uint32_t rf18 = _reg->r32(static_cast<uint16_t>((path ? 0x2C00 : 0x2800) + (0x18 << 2))) & 0xFFFFFu;
      const uint32_t alt = (rf18 & ~0xFFu) | ((rf18 & 0xFF) == 1 ? 2u : 1u);
      _reg->w32(lssi, (0x18u << 20) | alt);
      _reg->w32(lssi, (0x18u << 20) | rf18);
    }
  }
}

} // namespace wsdr
