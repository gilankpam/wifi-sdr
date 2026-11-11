// SPDX-License-Identifier: GPL-2.0-only
// Receiver: one adapter (RTL8812BU / RTL8812CU) as a tunable receiver, for applications.
//
// Owns the Session and a worker thread that applies settings between stream runs (the chip is never
// touched while it streams), tunes (5 MHz synthesizer grid; the residual is reported for a digital
// shift), runs the chip's streamer and hands every completed 64 KiB LA bank to a sink.
//
// The sink runs on the stream thread, which is also the libusb event thread: it must only copy (a
// stall there becomes a chip-side deadline fault). Unpacking (la_i/la_q, path_b_i/path_b_q), DC removal and
// the residual shift belong on the consumer's thread; BankMeta carries what they need.
//
// Faults restart the stream; an unplugged or dead adapter moves the state to Lost and the worker ends.
// Logic follows src/soapy/SoapyWsdr.cpp (measured behaviour: FINDINGS §4).
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "logger.h"
#include "session.h"

namespace wsdr {

constexpr uint16_t kPid8812bu = 0xb812;
constexpr uint16_t kPid8812cu = 0xc812;
constexpr uint16_t kPid8812eu = 0xa81a; // RTL8812EU (RTL8822E): runs the 8822C code unchanged (notes/cu-custom-code.md)
// The Jaguar3 adapters (8822C, 8822E): the 8822C LA dialect, loader, streamer and gain map throughout.
inline bool jaguar3_pid(uint16_t pid) { return pid == kPid8812cu || pid == kPid8812eu; }
// The adapter (kPid8812bu / kPid8812cu / kPid8812eu) a USB id is, 0 if none of ours: Realtek's own ids,
// and boards that carry one of the chips under their maker's id (devourer tells the chip by register).
inline uint16_t model_pid(uint16_t vid, uint16_t pid) {
  if (vid == 0x0bda)
    return pid == kPid8812bu || jaguar3_pid(pid) ? pid : 0;
  if (vid == 0x2357 && (pid == 0x0115 || pid == 0x012d || pid == 0x0138)) // TP-Link Archer T4U v3, T3U, T3U Plus (RTL8812BU)
    return kPid8812bu;
  return 0;
}

struct RxSettings {
  double freq = 5180e6; // requested centre
  int rate_sel = 5;     // fs = 80 MHz >> rate_sel: 5 = 2.5, 4 = 5, 3 = 10, 2 = 20, 1 = 40 MS/s; 0 = 80 MS/s
                        // (8812BU, gapped only: 80 MHz width + vendor BW80 RX DFIR, ~77 MHz at -6 dB)
  bool wide = true;     // at 40 MS/s: 40 MHz channel width (8812BU: + vendor BW40 RX DFIR; 8822C: devourer's
                        // BW40 set opens its filter, flat to +-17 MHz, measured)
  bool agc = false;     // true: chip AGC; false: fixed gain (OFDM CCA off, IGI pinned)
  double gain_db = 30;
  bool allow_gapped = false; // rate_sel below min_rate_sel streams gapped (see BankMeta::gapped)
  bool soft_deadline = false; // 8812BU: deliver banks that ship late instead of restarting (StreamCfg::soft_deadline)
};

struct RxCaps {
  uint16_t pid = 0;
  bool cu = false;          // Jaguar3 (RTL8822C / RTL8822E: path B elsewhere in the LA word), USB 2 rates only
  bool superspeed = false;
  std::string chip;
  int min_rate_sel = 5;     // fastest rate streamed continuously
  int gapped_min_rate_sel = 6; // fastest rate in gapped mode (8812BU: 0; 8812CU: 1); 6 = none
  int paths = 1;            // 2: path A and path B (both chips; the 8812BU via kDmaBothPaths8822b)
  double gain_max = 0, gain_step = 1;
};

enum class RxState { Closed, Opening, Starting, Streaming, Lost, Error };
const char *rx_state_name(RxState s);

struct BankMeta {
  uint64_t seq = 0;      // banks delivered since start()
  uint32_t gen = 0;      // stream run (changes on every restart: reset DSP state when it does)
  uint32_t cfg = 0;      // settings generation (changes only when the caller changed settings)
  double fs = 0;         // sample rate
  double centre_hz = 0;  // hardware centre frequency
  double shift_hz = 0;   // requested freq - hardware centre; Nco::set(shift_hz, fs) brings the requested
                         // frequency to 0 Hz (as the Soapy module does)
  int paths = 1;
  bool cu = false;
  bool gapped = false;   // each bank is a complete capture, but banks are not contiguous (a ~1.5 ms gap
                         // between them): FFT within a bank only, no continuous recording
};

using BankSink = std::function<void(const uint8_t *bank, size_t bytes, const BankMeta &)>;

struct RxStatus {
  RxState state = RxState::Closed;
  uint64_t banks = 0, restarts = 0;
  uint64_t late_banks = 0; // delivered past the deadline (RxSettings::soft_deadline)
  uint64_t damaged_banks = 0; // delivered with lost chunks zeroed (RxSettings::soft_deadline)
  int channel = 0, width = 20;
  double centre_hz = 0, fs = 0;
  std::string error;     // last fault (cleared on a clean run start)
  std::string last_fault; // why the stream last restarted; kept until the next open (error is cleared
                          // ~100 ms later by the first clean bank, before a poll can read it)
  uint32_t gen = 0, cfg = 0; // run / settings generation of the current stream (as in BankMeta)
  bool running = false;  // the worker is alive (false after Lost/Error ended it, or stop())
  bool settling = false; // a settings change is waiting to be applied (the run above is about to end)
  bool gapped = false;   // the current run is gapped (BankMeta::gapped)
  unsigned start_retries = 0; // stream starts repeated because the first packet never came (issue #2)
};

class Receiver {
public:
  explicit Receiver(Logger_t log);
  ~Receiver();
  Receiver(const Receiver &) = delete;
  Receiver &operator=(const Receiver &) = delete;

  // Blocking (~5 s bring-up). Throws OpenError (not found / permission / busy) or runtime_error.
  // bus/port pick one of several identical adapters (port path as "1" or "1.2"; -1/"" = any).
  // pid: the adapter (kPid8812bu, ...); a board under another USB id that is one (model_pid) opens too.
  // sys_fd >= 0: open the adapter behind this usbfs fd (Android) instead of scanning (OpenOpts::sys_fd).
  void open(uint16_t pid, const RxSettings &initial = {}, int bus = -1, const std::string &port = {},
            int sys_fd = -1);
  void close(); // stop + release the adapter
  bool is_open() const { return _open; }
  RxCaps caps() const { return _caps; }

  void start(BankSink sink); // worker thread; no-op if running (after Lost/Error: a new worker)
  void stop();               // joins the worker
  // Takes effect between stream runs. A change that leaves the hardware as it is (e.g. a frequency
  // inside the same 5 MHz synthesizer step) is recorded without restarting the stream.
  void set(const RxSettings &s);
  RxSettings settings() const;
  RxStatus status() const;

  // Pure helpers (also used by tests and UIs).
  static double rate_of(int rate_sel) { return 80e6 / (1 << rate_sel); }
  static int width_of(const RxSettings &s, bool cu) {
    return s.rate_sel == 0 && !cu ? 80 : s.rate_sel == 1 && s.wide ? 40 : 20;
  }
  int lowest_rate_sel(const RxSettings &s) const {
    return s.allow_gapped ? std::min(_caps.min_rate_sel, _caps.gapped_min_rate_sel) : _caps.min_rate_sel;
  }
  static int channel_for(double freq, int width);   // primary channel (5 MHz grid)
  static double centre_of(int channel, int width);  // hardware centre for that primary channel
  static int igi_for(double db, bool cu);
  static double gain_step(bool cu) { return cu ? 1.0 : 0.95; }
  static double gain_max(bool cu) { return (0x50 - 0x1C) * gain_step(cu); }

private:
  void worker();
  void set_state(RxState s, const std::string &err = {});

  Logger_t _log;
  std::unique_ptr<Session> _s;
  RxCaps _caps;
  mutable std::mutex _m; // settings, status strings
  RxSettings _set;
  uint32_t _gen = 0, _cfg = 0;
  std::chrono::steady_clock::time_point _last_change{};
  RxStatus _st;
  int _hw_channel = 0, _hw_width = 20;
  BankSink _sink;
  std::atomic<bool> _active{false}, _restart{false}, _open{false};
  std::thread _worker;
};

} // namespace wsdr
