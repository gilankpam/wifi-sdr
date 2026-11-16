// SPDX-License-Identifier: GPL-2.0-only
#include "session.h"

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
  if (libusb_init(&ctx) < 0)
    throw std::runtime_error("libusb_init failed");
  _s.adopt_context(ctx);
  libusb_device_handle *h = nullptr;
  {
    libusb_device **list = nullptr;
    const ssize_t n = libusb_get_device_list(ctx, &list);
    int rc = LIBUSB_ERROR_NOT_FOUND;
    for (ssize_t i = 0; i < n && rc == LIBUSB_ERROR_NOT_FOUND; i++) {
      libusb_device_descriptor d{};
      if (libusb_get_device_descriptor(list[i], &d) != 0 || !usb_id_matches(o.vid, o.pid, d.idVendor, d.idProduct))
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
}

} // namespace wsdr
