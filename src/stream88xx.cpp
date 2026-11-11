// SPDX-License-Identifier: GPL-2.0-only
#include "stream88xx.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>

#include "bulkin88xx.h"
#include "c2h88xx.h"
#include "cpu88xx.h"
#include "stream8822b.h"

namespace wsdr {

namespace {
using clk = std::chrono::steady_clock;

constexpr uint32_t kStateOff = 0x1000;  // DMEM offset of the program's state block
constexpr uint32_t kRecOff = 0x1100;    // bank records (0x40 apart)
constexpr uint32_t kMagic = 0x57535431u;
constexpr uint32_t kTrailerOcp = 0x18701000u; // RX SRAM; C2H address 0x1000
constexpr uint32_t kBankCpu[2] = {0xB87A0000u, 0xB87B0000u};
constexpr uint32_t kBankC2h[2] = {0x16000u, 0x06000u}; // TXFF 0x20000 / 0x30000 aliases
constexpr uint8_t kSentinel[8] = {0xE1, 0xF1, 0x47, 0x5E, 0xA5, 0xA5, 0xA5, 0xA5}; // stream8822b.S

constexpr unsigned kStartRetries = 2;
constexpr unsigned kMaxLateRun = 64; // soft_deadline: ~13 ms of late banks at 40 MS/s means the chip is not catching up

size_t chunk_off(int k) { return Stream8822b::kPrefix + static_cast<size_t>(k) * Stream8822b::kChunk; }

void desc(uint8_t *d, size_t payload) {
  std::memset(d, 0, 24);
  d[0] = static_cast<uint8_t>(payload & 0xFF);
  d[1] = static_cast<uint8_t>((payload >> 8) & 0x3F);
  d[3] = 0x80;  // OWN
  d[11] = 0x10; // C2H
}

std::string hex12(const uint8_t *p) {
  char b[40];
  for (int i = 0; i < 12; i++)
    std::snprintf(b + 3 * i, 4, "%02x%s", p[i], i < 11 ? "." : "");
  return b;
}

uint8_t fill(int bank, size_t i) { // synthetic pattern
  uint32_t x = static_cast<uint32_t>(i) * 0x9E3779B1u ^ (bank ? 0x5bd1e995u : 0x1b873593u);
  x ^= x >> 15;
  x *= 0x2c1b3c6du;
  return static_cast<uint8_t>(x >> 24);
}

// Where the CPU program got to, for a missed first packet: its DMEM state block (magic, seq, state,
// t_arm), the chip clock, the C2H trigger and the LA engine.
std::string chip_state(Reg &r, Cpu8822b &cpu) {
  try {
    const uint32_t now = r.r32(0x0568);
    uint32_t s[5];
    cpu.dmem_read(kStateOff, s, sizeof s);
    char b[160];
    std::snprintf(b, sizeof b, "; magic %08x seq %u state %u t_arm %u now %u r294 %08x r7c0 %08x", s[0], s[1],
                  s[2], s[4], now, r.r32(0x0294), r.r32(0x07C0));
    return b;
  } catch (const std::exception &e) {
    return std::string("; state unreadable: ") + e.what();
  }
}

} // namespace

StreamStats Stream8822b::run(const std::function<bool(const uint8_t *, const BankInfo &)> &on_bank,
                             const std::atomic<bool> *stop) {
  StreamStats st;
  if (_c.gapped && _c.synthetic) {
    st.error = "gapped mode needs live data";
    return st;
  }
  Reg &r = _s.reg();
  std::optional<C2hRxPath> rx; // its constructor reads registers: built inside the try (unplug -> error)
  La8822b la(r);
  Cpu8822b cpu(r);
  InOrderReader rd(_s.handle(), _s.context(), _c.depth);
  const uint32_t period5 = 512u << _c.la.rate_sel;
  const int32_t guard5 = _c.guard_us * 5;

  uint8_t data_desc[24], trl_desc[24];
  desc(data_desc, kChunk);
  desc(trl_desc, kTrailerPayload);
  std::vector<uint8_t> bank(kBankBytes);
  bool seen[2] = {false, false};
  const auto t0 = clk::now();
  // The whole start sequence; repeatable (load() halts and platform-resets the CPU first).
  const auto start = [&] {
    cpu.load(stream8822b_bin, stream8822b_bin_len);
    rx->apply(true); // MAC RX on, no frames accepted: the LA writer needs it
    la.configure(_c.la);
    if (_c.synthetic) {
      std::vector<uint8_t> pat(kBankBytes);
      for (int b = 0; b < 2; b++) {
        for (size_t i = 0; i < kBankBytes; i++)
          pat[i] = fill(b, i);
        r.sram_write(La8822b::kRingOcp + static_cast<uint32_t>(b) * kBankBytes, pat.data(), pat.size());
      }
    }
    uint8_t trl[24 + kTrailerPayload] = {};
    std::memcpy(trl, trl_desc, 24);
    r.sram_write(kTrailerOcp, trl, sizeof trl);

    uint32_t state[10] = {0, 0, 0, 0, 0, _c.synthetic ? 0u : 1u, period5, static_cast<uint32_t>(guard5), 0,
                          _c.gapped ? 1u : 0u};
    std::memcpy(&state[8], data_desc, 4);
    cpu.dmem_write(kStateOff, state, sizeof state);
    uint32_t rec[2][16] = {};
    for (int b = 0; b < 2; b++) {
      rec[b][0] = kBankCpu[b];
      for (int k = 0; k < 4; k++) {
        const uint32_t d = static_cast<uint32_t>(chunk_off(k) - 24);
        rec[b][1 + 2 * k] = kBankCpu[b] + d;
        rec[b][2 + 2 * k] = c2h_request(kBankC2h[b] + d);
      }
    }
    cpu.dmem_write(kRecOff, rec, sizeof rec);

    rd.start();
    rd.drain(30);
    cpu.start();
    // Now and then (~1 start in 30, issue #2) the CPU stalls at its first MAC-register access (the LA
    // arm) until the host itself reads the MAC block; a read of 0x0003 or 0x0080 does not release it.
    // MEASURED: a stalled CPU armed exactly at the host's next 0x0568 read, ~20 ms later; polling
    // 0x7C0 from here on, 0 stalls in ~3000 starts against 47 in 1088 without (armed by the 1st read
    // in 97 %, else the 2nd).
    // Bounded: a synthetic run never arms, and in gapped mode the CPU may stop the LA again.
    const auto p0 = clk::now();
    while (r.r8(0x07C0) != 0xCB && clk::now() - p0 < std::chrono::milliseconds(1)) {
    }
  };
  // rd.next in slices of at most 20 ms, so that a stop request ends a long wait early.
  const auto next = [&](InOrderReader::Done &d, unsigned ms) {
    for (unsigned waited = 0;; waited += 20) {
      if (rd.next(d, std::min(20u, ms - waited)))
        return true;
      if (waited + 20 >= ms || (stop && *stop))
        return false;
    }
  };
  // The first bank is due one period + guard after the start. If it does not come within a few
  // periods, start again (a stalled start used to cost the 200 ms pkt_timeout_ms and a Receiver
  // restart); the last attempt waits the full pkt_timeout_ms before calling it a fault.
  const unsigned first_ms =
      std::max(20u, static_cast<unsigned>((4 * period_us() + std::max(_c.guard_us, 0)) / 1000));
  try {
    rx.emplace(r);
    la.snapshot(); // before cpu.load(): its SRAM writes move the debug window (0x0106 / 0x0140)
    start();

    const double budget5 = static_cast<double>(period5) - guard5;
    unsigned late_run = 0; // soft_deadline: consecutive late banks
    for (uint32_t seq = 0;; seq++) {
      if (stop && *stop)
        break;
      const int b = _c.gapped ? 0 : static_cast<int>(seq & 1);
      int part = 0;
      BankInfo bi;
      bi.seq = seq;
      bi.bank = b;
      bool stopped = false;
      while (part < 5) {
        InOrderReader::Done d;
        const bool first = seq == 0 && part == 0;
        const bool retry = first && st.start_retries < kStartRetries;
        if (!next(d, retry ? first_ms : _c.pkt_timeout_ms)) {
          if (stop && *stop) {
            stopped = true;
            break;
          }
          const std::string why = first ? chip_state(r, cpu) : ""; // mid-stream the chip may not answer
          if (retry) {
            std::fprintf(stderr, "stream: no first packet in %u ms, starting again%s\n", first_ms, why.c_str());
            st.start_retries++;
            cpu.halt();
            rd.stop();
            start();
            continue;
          }
          throw std::runtime_error("no packet within timeout (bank " + std::to_string(seq) + " part " +
                                   std::to_string(part) + ")" + why);
        }
        if (d.status != LIBUSB_TRANSFER_COMPLETED)
          throw std::runtime_error("bulk-IN status " + std::to_string(d.status));
        const bool c2h = d.len >= 24 && (d.data[11] & 0x10);
        if (!c2h) {
          if (st.strays < 8) {
            std::fprintf(stderr, "stream: stray bank %u part %d len %d:", seq, part, d.len);
            for (int i = 0; i < std::min(d.len, 24); i++)
              std::fprintf(stderr, " %02x", d.data[i]);
            std::fprintf(stderr, "\n");
          }
          st.strays++;
          rd.release();
          continue;
        }
        if (part < 4 && _c.soft_deadline && static_cast<size_t>(d.len) == 24 + kTrailerPayload &&
            std::memcmp(d.data, trl_desc, 12) == 0) {
          // The trailer came while data was still due: the chip cut data packets short during a host
          // stall (they arrived as headerless strays). It still closes this bank (its seq is checked
          // below): zero the missing chunks and keep streaming instead of restarting.
          for (int k = part; k < 4; k++)
            std::memset(bank.data() + chunk_off(k), 0, kChunk);
          bi.damaged = true;
          part = 4;
        }
        const bool trailer = part == 4;
        const uint8_t *want = trailer ? trl_desc : data_desc;
        const size_t want_len = 24 + (trailer ? kTrailerPayload : kChunk);
        if (static_cast<size_t>(d.len) != want_len || std::memcmp(d.data, want, 12) != 0)
          throw std::runtime_error("framing: bank " + std::to_string(seq) + " part " + std::to_string(part) +
                                   " len " + std::to_string(d.len) + " desc " + hex12(d.data));
        if (!trailer) {
          std::memcpy(bank.data() + chunk_off(part), d.data + 24, kChunk);
        } else {
          std::memcpy(bank.data(), d.data + 24, kPrefix);
          uint32_t stamp[4];
          std::memcpy(stamp, d.data + 24 + kPrefix, sizeof stamp);
          if (stamp[0] != seq)
            throw std::runtime_error("trailer seq " + std::to_string(stamp[0]) + " != " + std::to_string(seq));
          bi.late_us = static_cast<int32_t>(stamp[2] - stamp[1]) / 5.0;
          bi.ship_us = static_cast<int32_t>(stamp[3] - stamp[2]) / 5.0;
          bi.end_us = static_cast<int32_t>(stamp[3] - stamp[1]) / 5.0;
          if (bi.damaged && ++late_run > kMaxLateRun)
            throw std::runtime_error("framing: " + std::to_string(late_run) + " damaged/late banks in a row");
          if (!_c.gapped && bi.end_us * 5 > budget5) { // gapped: the LA is stopped while a bank ships
            if (!_c.soft_deadline || (!bi.damaged && ++late_run > kMaxLateRun))
              throw std::runtime_error("deadline: bank " + std::to_string(seq) + " ended " +
                                       std::to_string(bi.end_us) + " us after due (late " +
                                       std::to_string(bi.late_us) + ", ship " + std::to_string(bi.ship_us) + ")");
            bi.late = true;
          } else if (!bi.damaged) {
            late_run = 0;
          }
        }
        rd.release();
        part++;
      }
      if (stopped)
        break;
      // CPU-written windows: descriptors ([104,128) and the 24 bytes before chunks 1..3) and the
      // sentinel in the last sample. In live data the ADC must have overwritten all of them.
      uint8_t *tail = bank.data() + kBankBytes - 8;
      if (_c.synthetic) {
        for (size_t i = 0; i < kBankBytes - 8 && !bi.pattern_bad; i++) {
          bool win = false;
          for (int k = 0; k < 4; k++)
            win |= i >= chunk_off(k) - 24 && i < chunk_off(k);
          bi.pattern_bad = !win && bank[i] != fill(b, i);
        }
      } else if (seen[b]) {
        for (int k = 0; k < 4; k++)
          bi.stale |= std::memcmp(bank.data() + chunk_off(k) - 24, data_desc, 12) == 0;
        bi.stale |= std::memcmp(tail, kSentinel, 8) == 0;
      }
      seen[b] = true;
      st.banks++;
      st.stale += bi.stale;
      st.late += bi.late;
      st.damaged += bi.damaged;
      st.pattern_bad += bi.pattern_bad;
      st.late_max_us = std::max(st.late_max_us, bi.late_us);
      st.ship_max_us = std::max(st.ship_max_us, bi.ship_us);
      st.end_max_us = std::max(st.end_max_us, bi.end_us);
      if (!on_bank(bank.data(), bi))
        break;
    }
  } catch (const std::exception &e) {
    st.error = e.what();
  }
  st.wall_s = std::chrono::duration<double>(clk::now() - t0).count();
  try {
    cpu.halt();
  } catch (...) {
  }
  rd.stop();
  la.restore();
  if (rx)
    rx->restore();
  return st;
}

} // namespace wsdr
