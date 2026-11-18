// SPDX-License-Identifier: GPL-2.0-only
// Device session: open + claim the adapter, bring it up with devourer (power, firmware, MAC/BB/RF
// tables, calibration, channel), then hand out raw register access for the SDR work.
// Teardown order is devourer's DeviceSession contract: radio -> interface -> handle -> context.
#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "DeviceSession.h"
#include "IRadio.h"
#include "logger.h"
#include "rtlreg.h"

namespace wsdr {

constexpr uint16_t kRealtekVid = 0x0bda;
constexpr uint16_t kPid8812bu = 0xb812;
constexpr uint16_t kPid8812cu = 0xc812;
constexpr uint16_t kPid8812eu = 0xa81a; // RTL8812EU (RTL8822E): runs the 8822C code unchanged (notes/cu-custom-code.md)
// The Jaguar3 adapters (8822C, 8822E): the 8822C LA dialect, loader, streamer and gain map throughout.
inline bool jaguar3_pid(uint16_t pid) { return pid == kPid8812cu || pid == kPid8812eu; }

// OEM-branded dongles: a supported Realtek chip under the brand's own USB id. Everything past the USB
// descriptor (the --pid dispatch, Soapy's "pid", the studio) sees the Realtek pid they stand for.
// A new entry also needs its udev line in tools/70-wsdr.rules, and in wsdr-studio's README and
// web/src/util.ts UDEV_RULES (a studio test checks that copy against the rules file).
struct UsbAlias {
  uint16_t vid, pid, rtl_pid;
};
inline constexpr UsbAlias kUsbAliases[] = {
    {0x2357, 0x0138, kPid8812bu}, // TP-Link Archer T3U Plus
    {0x2357, 0x0115, kPid8812bu}, // TP-Link Archer T4U v3 (tested in gilankpam/kepyur)
    {0x2357, 0x012d, kPid8812bu}, // TP-Link Archer T3U (the same chip; not yet tested)
};

// The Realtek pid a USB descriptor stands for: its own pid under 0bda, an alias's, else 0.
inline uint16_t realtek_pid(uint16_t dvid, uint16_t dpid) {
  if (dvid == kRealtekVid)
    return dpid;
  for (const UsbAlias &a : kUsbAliases)
    if (a.vid == dvid && a.pid == dpid)
      return a.rtl_pid;
  return 0;
}

// Does a USB descriptor (dvid, dpid) match the requested id (vid, pid)? A 0bda request also matches an alias.
inline bool usb_id_matches(uint16_t vid, uint16_t pid, uint16_t dvid, uint16_t dpid) {
  return vid == kRealtekVid ? pid != 0 && realtek_pid(dvid, dpid) == pid : dvid == vid && dpid == pid;
}

struct OpenOpts {
  uint16_t vid = kRealtekVid;
  uint16_t pid = 0;   // required
  int bus = -1;       // optional: only the adapter on this USB bus ...
  std::string port;   // ... and port path ("1", "1.2"); empty = any
  int channel = 36;   // primary 20 MHz channel
  int bw = 20;        // 20 / 40 / 80
  int offset = 1;     // devourer ChannelOffset at 40/80 MHz (where the centre sits from the primary; 1: above)
  bool reset = true;  // libusb_reset_device before claim (devourer default)
  bool thermal_track = true; // devourer's ~2 s RF thermal tick; off for timing-critical probes
};

// Session::open failures the caller can act on (e.g. "install the udev rule" for Permission).
struct OpenError : std::runtime_error {
  enum Kind { NotFound, Permission, Busy, Other } kind;
  OpenError(Kind k, const std::string &what) : std::runtime_error(what), kind(k) {}
};

class Session {
public:
  explicit Session(Logger_t logger);
  ~Session();
  // Open, claim, CreateRadio, InitWrite(channel). Throws OpenError (not found / permission / busy) or
  // std::runtime_error (bring-up failure).
  void open(const OpenOpts &o);
  IRadio *radio() const { return _s.device(); }
  libusb_device_handle *handle() const { return _s.handle(); }
  libusb_context *context() const { return _s.context(); }
  Reg &reg() { return *_reg; }
  bool superspeed() const { return _ss; }
  std::string chip() const { return _chip; }
  // Retune via devourer (full SetMonitorChannel path). channel: the primary; offset as OpenOpts::offset.
  void tune(int channel, int bw = 20, int offset = 1);

private:
  Logger_t _log;
  devourer::DeviceSession _s;
  std::unique_ptr<Reg> _reg;
  bool _ss = false;
  std::string _chip;
};

} // namespace wsdr
