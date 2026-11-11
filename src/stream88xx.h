// SPDX-License-Identifier: GPL-2.0-only
// Continuous LA streaming from the 8822B: stream8822b.S on the embedded CPU ships each completed
// 64 KiB ring bank (8192 samples) as four C2H data packets + one trailer; this side reassembles the
// banks in order and checks them. Rates are 80 MHz >> rate_sel; over USB 2 (~44 MB/s measured C2H)
// rate_sel 5 (2.5 MS/s, 20 MB/s) is the comfortable setting and 4 (5 MS/s, 40 MB/s) the ceiling.
//
// Integrity checks per bank:
//   framing   every packet is the expected C2H descriptor + length, in order; trailer seq matches
//   timing    the CPU's own stamps: shipping must end before the ADC re-enters the bank
//   staleness live data: the descriptors and the last-sample sentinel the CPU wrote last revolution
//             must have been overwritten by the ADC (if not, that part of the bank is old data);
//             synthetic data: exact pattern match outside those windows
#pragma once

#include <libusb.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "la88xx.h"
#include "session.h"

namespace wsdr {

struct StreamCfg {
  LaRingCfg la;            // la.rate_sel picks the rate (80 MHz >> rate_sel; LaRingCfg default 2)
  bool synthetic = false;  // host-filled banks, LA left stopped: transport/framing self-test
  int depth = 32;          // bulk-IN transfers kept queued
  int guard_us = 20;       // ship a bank this long after the ADC is due to finish it (< 0: test only)
  bool gapped = false;     // capture one bank, stop, ship, re-arm (banks not contiguous)
  unsigned pkt_timeout_ms = 200;
  // A bank that ships past period - guard is normally a fault (the ADC may already be overwriting its
  // first samples). soft_deadline: deliver it anyway (BankInfo::late), keep streaming, and fault only after
  // kMaxLateRun such banks in a row. Likewise a bank whose data packets the chip cut short (BankInfo::damaged). For consumers that shrug off a few damaged samples (video) on hosts
  // whose USB stack stalls now and then (Windows: DPC latency, measured 2026-11-11).
  bool soft_deadline = false;
};

struct BankInfo {
  uint32_t seq = 0;
  int bank = 0;            // 0 / 1 (seq & 1)
  double late_us = 0;      // shipping start - bank due time
  double ship_us = 0;      // data packets, start to end
  double end_us = 0;       // end - due time; must stay below period - guard
  bool stale = false;      // live: a stale descriptor survived (bank not rewritten by the ADC)
  bool pattern_bad = false; // synthetic: payload differs from the fill pattern
  bool late = false;       // soft_deadline: shipped past the deadline (first samples may be newer data)
  bool damaged = false;    // soft_deadline: data packets lost, the trailer closed the bank (missing chunks zeroed)
};

struct StreamStats {
  uint64_t banks = 0;
  unsigned strays = 0;     // non-C2H packets skipped (MAC RX frames)
  unsigned stale = 0;
  unsigned pattern_bad = 0;
  unsigned start_retries = 0; // starts repeated because the first packet never came (issue #2)
  uint64_t late = 0;       // banks delivered past the deadline (soft_deadline)
  uint64_t damaged = 0;    // banks delivered with chunks missing (soft_deadline)
  double late_max_us = 0, ship_max_us = 0, end_max_us = 0;
  double wall_s = 0;
  std::string error;       // empty unless the stream stopped on a fault
};

class Stream8822b {
public:
  static constexpr size_t kBankBytes = 0x10000;
  static constexpr size_t kBankSamples = kBankBytes / 8;
  static constexpr size_t kPrefix = 128;
  static constexpr size_t kChunk = 16352;
  static constexpr size_t kTrailerPayload = kPrefix + 16;

  Stream8822b(Session &s, const StreamCfg &c) : _s(s), _c(c) {}
  double fs() const { return 80e6 / (1u << _c.la.rate_sel); }
  double period_us() const { return kBankSamples / fs() * 1e6; }
  // Streams until on_bank returns false, stop is set, or a fault. bank points at kBankBytes bytes
  // (kBankSamples little-endian u64 LA words). Restores every register it touched.
  StreamStats run(const std::function<bool(const uint8_t *bank, const BankInfo &)> &on_bank,
                  const std::atomic<bool> *stop = nullptr);

private:
  Session &_s;
  StreamCfg _c;
};

// 8822B LA dma_type with both RX paths in one word: path A I/Q [11:0]/[23:12], path B [35:24]/[47:36],
// post-DFIR at every rate; B leads A by ~146 ns (FINDINGS "8822B dual-path tap"). dma_type 0 is A alone.
constexpr int kDmaBothPaths8822b = 4;

// LA word -> 12-bit I / Q (path A layout, dma_type 0/1).
inline int la_i(uint64_t w) { return static_cast<int>(static_cast<int32_t>(static_cast<uint32_t>(w & 0xFFF) << 20) >> 20); }
inline int la_q(uint64_t w) { return static_cast<int>(static_cast<int32_t>(static_cast<uint32_t>((w >> 12) & 0xFFF) << 20) >> 20); }

} // namespace wsdr
