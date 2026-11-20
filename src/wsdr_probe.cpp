// SPDX-License-Identifier: GPL-2.0-only
// wsdr_probe — incremental hardware probes for the streaming path (8822B first).
//
//   wsdr_probe ring   --pid 0xb812 [--channel 36] [--rate 2] [--dbg 0x880] [--cck 0|1] [--filter 0|1]
//                     [--dump PREFIX]
//       Arm the LA engine as a free-running ring (never triggered), read the 128 KiB ring twice over
//       EP0 and report how much changed: a live writer changes ~every word between reads; a frozen
//       one (ac54's trap: MAC RX off, or CCK holding the bus) reads back identical.
//   wsdr_probe marker --pid 0xb812 [--channel 36]
//       Load the MIPS16e2 marker program behind the vendor prologue, run it, and read its DMEM block
//       twice: proves upload, execution, DMEM writes and the REG 0x568 microsecond counter.
//   wsdr_probe c2h --pid 0xb812 [--source rx|tx0|tx1] [--bytes 16352] [--count 64] [--depth 16]
//       Host-issued C2H requests (no CPU code): stage one PRBS packet, write REG 0x294, verify the
//       bulk-IN EP 0x84 completion. `tx0`/`tx1` exercise ac54's LA-TX -> C2H alias on this unit.
//   wsdr_probe c2hbench --pid 0xb812 [--bytes 16352] [--count 1024] [--depth 16] [--dump FILE]
//       Run c2hbench8822b.S on the embedded CPU (RAM only): per-request C2H latency from REG 0x568,
//       read back from DMEM. This is the transport-tail measurement, not live ADC.
//   wsdr_probe latime --pid 0xb812 [--rate 5] [--d-us 2000]
//       Run latime8822b.S: the CPU arms the LA ring, waits D us on REG 0x568 and fires the trigger;
//       the ring position at stop vs D * fs gives the sample rate and arm-to-first-sample latency.
//   wsdr_probe regs --pid 0xb812 --dump PREFIX
//       Dump MAC 0x000-0x7ff, 0x1000-0x17ff and 0xfe00-0xfe7f after bring-up (PREFIX-init.txt) and after
//       the C2H RX-path setup (PREFIX-c2h.txt), in rtw88 debugfs mac_N layout, for register diffs.
//   wsdr_probe stream --pid 0xb812 [--rate 5] [--seconds 10] [--out PREFIX] [--synthetic] [--guard-us 20]
//       Continuous LA streaming (stream8822b.S): every ring bank shipped over C2H, reassembled and
//       checked in order. --out writes PREFIX.cs16 (int16 I,Q, 12-bit << 4) + PREFIX.json. Path A;
//       --path b|ab with --dma 4 (or 5): path B (bits [47:24] of the word), ab adds PREFIX-b.cs16. The
//       USB callback only copies each bank into a 128 MiB ring; a writer thread unpacks and writes, so
//       a slow disk write cannot make the chip miss its deadline (a disk that falls 0.4 s behind ends
//       the run). --gapped: one bank per round, banks not contiguous; also runs at --rate 0 (80 MS/s).
//       The printed rms is path A.
//   LA options (ring/snap/latime/stream/tapscan): --rate N --dbg 0xNNN --dma N --igi N --nocca --dfir 20|40|80
//       (--dfir: vendor RX DFIR for that width; with --bw 80 --dfir 80 --rate 0 the dma-0 tap is a genuine 80 MS/s,
//       ~76 MHz band; --bw 40 --dfir 40 --rate 1 gives 40 MS/s / ~40 MHz. Note --channel is the PRIMARY 20 MHz
//       channel: the centre is ch+2 at --bw 40 and ch+6 at --bw 80.)
//   wsdr_probe dbgscan --pid 0xb812 [--lo 0x0] [--hi 0xfff] [--reads 8]
//       BB dbg-port survey: for each 0x8fc selector, read the dbg-port value (0xfa0) N times and report
//       how many distinct values and which bits toggled. 0x8fc/0x198c are restored afterwards.
//   wsdr_probe tapscan --pid 0xb812 --plan FILE --dump PREFIX [LA options as for snap]
//       One session, many LA configs. Each plan line: `label [dbg=N] [dma=N] [rate=N] [w=ADDR/MASK/VAL]...`
//       (w= are BB/MAC 32-bit masked writes applied after the LA config, restored after the entry).
//       Per entry: two contiguous snapshots; PREFIX-label.u64 = snapshot 0 unrolled (raw u64 words);
//       `count=N` appends N-2 further snapshots (16384 words each) for burst hunting.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "c2h88xx.h"
#include "c2hbench8822b.h"
#include "cpu88xx.h"
#include "la88xx.h"
#include "latime8822b.h"
#include "logger.h"
#include "marker8822b.h"
#include "session.h"
#include "stream88xx.h"

using namespace wsdr;
using clk = std::chrono::steady_clock;

namespace {

struct Args {
  std::string cmd;
  OpenOpts open;
  LaRingCfg la;
  std::string dump;
  std::string source = "rx"; // c2h: rx | tx0 | tx1
  bool restage = false;      // c2h: rewrite the descriptor before each request
  bool rx_on = false;        // c2h/c2hbench: keep MAC RX on + filter frames (streamer config)
  bool halt_cpu = false;     // c2h: halt the vendor CPU before issuing requests
  int gap_us = 0;            // c2h: delay after each verified packet
  int retry_ms = 0;          // c2h: re-write the request if the trigger has not cleared
  int cr = -1;               // c2h: force REG_CR (0x0100) before the run, -1 = leave
  bool no_poll = false;      // c2h: do not poll 0x294; wait for the bulk packet instead
  bool abort_retry = false;  // c2h: on poll timeout write 0 to 0x294 before re-triggering
  int rx_off = 0x1000;       // c2h: RX SRAM byte offset for the source packet
  int agg_th = -1;           // c2h: override RXDMA agg page threshold (0x0280[7:0]), -1 = leave
  bool fwff_off = false;     // c2h/c2hbench: disable the firmware-packet (FWFF) engine
  int rxdma_mode = -1;       // c2h: override RXDMA mode (0x0290), -1 = leave at 14
  bool no_lpm = false;       // c2h: clear USB U1/U2 (CLEAR_FEATURE 48/49) before the run
  bool vary = false;         // c2h: rotate over 3 (rx) / 4 (tx) 16 KiB slots, as the real streamer does
  int rw_release = -1;       // c2h: 0/1 = force 0x0284 BIT18 (RW_RELEASE_EN), -1 = leave
  bool unique = false;       // c2h: fresh payload every request (rules out stale-match/duplicate ambiguity)
  bool check_desc = false;   // c2h: verify descriptor bytes 0..11 before each request
  std::vector<std::pair<uint16_t, uint32_t>> sets; // c2h: --set ADDR=VAL (32-bit writes after setup)
  int bytes = 16352;         // c2h/c2hbench: C2H payload size
  int depth = 16;            // c2h/c2hbench: async bulk-IN queue depth
  int xfer = 16384;          // c2h/c2hbench: bulk-IN transfer size
  int reads = 2;
  int count = 0;      // snap/c2h/c2hbench: command default when 0
  int settle_ms = 50; // snap: wait after arm before the first trigger
  int d_us = 2000;    // latime: arm-to-trigger delay on the chip clock
  bool rate_given = false;
  double seconds = 10; // stream
  bool synthetic = false; // stream: host-filled banks, LA stopped
  bool gapped = false;    // stream: one bank per round (banks not contiguous)
  int guard_us = 20;      // stream
  std::string out;        // stream: output prefix
  std::string plan;       // tapscan: plan file
  std::string path = "a"; // stream: a, b or ab (b needs --dma 4: both paths in one word)
  int lo = 0, hi = 0xfff; // dbgscan: selector range
  int hop_igi = -1;       // stream (spike): >= 0: alternate IGI between --igi and this while streaming
  int hop_ms = 500;       // stream (spike): time between IGI writes
  int hop_mode = 0;       // stream (spike): 0 m32, 1 CCA on around the write, 2 byte write, 3 as 1 but w8
};

unsigned long num(const char *s) { return std::strtoul(s, nullptr, 0); }

bool parse(int argc, char **argv, Args &a) {
  if (argc < 2)
    return false;
  a.cmd = argv[1];
  for (int i = 2; i < argc; i++) {
    std::string k = argv[i];
    auto v = [&]() -> const char * { return i + 1 < argc ? argv[++i] : "0"; };
    if (k == "--pid") a.open.pid = static_cast<uint16_t>(num(v()));
    else if (k == "--vid") a.open.vid = static_cast<uint16_t>(num(v()));
    else if (k == "--bus") a.open.bus = static_cast<int>(num(v()));
    else if (k == "--port") a.open.port = v();
    else if (k == "--channel") a.open.channel = static_cast<int>(num(v()));
    else if (k == "--bw") a.open.bw = static_cast<int>(num(v()));
    else if (k == "--no-reset") a.open.reset = false;
    else if (k == "--no-thermal") a.open.thermal_track = false;
    else if (k == "--rate") { a.la.rate_sel = static_cast<int>(num(v())); a.rate_given = true; }
    else if (k == "--dbg") a.la.dbg_port = static_cast<uint32_t>(num(v()));
    else if (k == "--dma") a.la.dma_type = static_cast<int>(num(v()));
    else if (k == "--dfir") a.la.dfir = static_cast<int>(num(v()));
    else if (k == "--plan") a.plan = v();
    else if (k == "--path") a.path = v();
    else if (k == "--lo") a.lo = static_cast<int>(num(v()));
    else if (k == "--hi") a.hi = static_cast<int>(num(v()));
    else if (k == "--cck") a.la.cck_off = num(v()) == 0;
    else if (k == "--filter") a.la.filter_rx = num(v()) != 0;
    else if (k == "--dbgclk") a.la.dbgport_clk = num(v()) != 0;
    else if (k == "--dump") a.dump = v();
    else if (k == "--source") a.source = v();
    else if (k == "--bytes") a.bytes = static_cast<int>(num(v()));
    else if (k == "--depth") a.depth = static_cast<int>(num(v()));
    else if (k == "--xfer") a.xfer = static_cast<int>(num(v()));
    else if (k == "--restage") a.restage = true;
    else if (k == "--rx-on") a.rx_on = true;
    else if (k == "--halt-cpu") a.halt_cpu = true;
    else if (k == "--gap-us") a.gap_us = static_cast<int>(num(v()));
    else if (k == "--retry-ms") a.retry_ms = static_cast<int>(num(v()));
    else if (k == "--cr") a.cr = static_cast<int>(num(v()));
    else if (k == "--no-poll") a.no_poll = true;
    else if (k == "--abort") a.abort_retry = true;
    else if (k == "--rx-off") a.rx_off = static_cast<int>(num(v()));
    else if (k == "--agg-th") a.agg_th = static_cast<int>(num(v()));
    else if (k == "--fwff-off") a.fwff_off = true;
    else if (k == "--rxdma-mode") a.rxdma_mode = static_cast<int>(num(v()));
    else if (k == "--no-lpm") a.no_lpm = true;
    else if (k == "--vary") a.vary = true;
    else if (k == "--rw-release") a.rw_release = static_cast<int>(num(v()));
    else if (k == "--unique") a.unique = true;
    else if (k == "--check-desc") a.check_desc = true;
    else if (k == "--set") {
      std::string sv = v();
      const size_t eq = sv.find('=');
      if (eq == std::string::npos) { std::fprintf(stderr, "--set needs ADDR=VAL\n"); return false; }
      a.sets.push_back({static_cast<uint16_t>(num(sv.substr(0, eq).c_str())),
                        static_cast<uint32_t>(num(sv.substr(eq + 1).c_str()))});
    }
    else if (k == "--igi-hop") a.hop_igi = static_cast<int>(num(v()));
    else if (k == "--hop-mode") a.hop_mode = static_cast<int>(num(v()));
    else if (k == "--hop-ms") a.hop_ms = static_cast<int>(num(v()));
    else if (k == "--igi") a.la.igi = static_cast<int>(std::strtol(v(), nullptr, 0));
    else if (k == "--ofdm") a.la.ofdm_off = num(v()) == 0;
    else if (k == "--nocca") a.la.no_cca = true;
    else if (k == "--reads") a.reads = static_cast<int>(num(v()));
    else if (k == "--count") a.count = static_cast<int>(num(v()));
    else if (k == "--settle") a.settle_ms = static_cast<int>(num(v()));
    else if (k == "--d-us") a.d_us = static_cast<int>(num(v()));
    else if (k == "--seconds") a.seconds = std::strtod(v(), nullptr);
    else if (k == "--synthetic") a.synthetic = true;
    else if (k == "--guard-us") a.guard_us = static_cast<int>(std::strtol(v(), nullptr, 0));
    else if (k == "--gapped") a.gapped = true;
    else if (k == "--out") a.out = v();
    else { std::fprintf(stderr, "unknown arg %s\n", k.c_str()); return false; }
  }
  return a.open.pid != 0;
}

double ms_since(clk::time_point t) {
  return std::chrono::duration<double, std::milli>(clk::now() - t).count();
}

int cmd_ring(Session &s, const Args &a) {
  Reg &r = s.reg();
  La8822b la(r);
  la.snapshot();
  int rc = 0;
  try {
    la.configure(a.la);
    la.arm();
    std::printf("{\"ev\":\"ring.cfg\",\"igi_a\":\"0x%02x\",\"igi_b\":\"0x%02x\",\"r808\":\"0x%08x\",\"r95c\":\"0x%08x\","
                "\"r8ac\":\"0x%08x\",\"r8c4\":\"0x%08x\"}\n",
                r.r32(0x0C50) & 0x7F, r.r32(0x0E50) & 0x7F, r.r32(0x0808), r.r32(0x095C), r.r32(0x08AC),
                r.r32(0x08C4));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const size_t n = La8822b::kRingBytes / 8;
    std::vector<std::vector<uint64_t>> rd(static_cast<size_t>(a.reads), std::vector<uint64_t>(n));
    for (int k = 0; k < a.reads; k++) {
      const uint32_t c0 = r.r32(0x07C0);
      auto t0 = clk::now();
      r.sram_read(La8822b::kRingOcp, rd[k].data(), La8822b::kRingBytes);
      const double ms = ms_since(t0);
      const uint32_t c1 = r.r32(0x07C0);
      std::printf("{\"ev\":\"ring.read\",\"k\":%d,\"ms\":%.1f,\"mbps\":%.2f,\"r7c0_before\":\"0x%08x\","
                  "\"r7c0_after\":\"0x%08x\"}\n",
                  k, ms, La8822b::kRingBytes / ms / 1e3, c0, c1);
      if (!a.dump.empty()) {
        std::string fn = a.dump + "-read" + std::to_string(k) + ".u64";
        if (FILE *f = std::fopen(fn.c_str(), "wb")) {
          std::fwrite(rd[k].data(), 8, n, f);
          std::fclose(f);
        }
      }
    }
    for (int k = 0; k < a.reads; k++) {
      std::map<uint64_t, unsigned> hi;
      size_t adj = 0, zero = 0;
      for (size_t i = 0; i < n; i++) {
        hi[rd[k][i] >> 24]++;
        if (i && (rd[k][i] & 0xFFFFFF) == (rd[k][i - 1] & 0xFFFFFF)) adj++;
        if ((rd[k][i] & 0xFFFFFF) == 0) zero++;
      }
      uint64_t top = 0;
      unsigned topn = 0;
      for (auto [v, c] : hi)
        if (c > topn) { top = v; topn = c; }
      double si = 0, sq = 0, si2 = 0, sq2 = 0;
      for (size_t i = 0; i < n; i++) {
        const double I = static_cast<double>(static_cast<int32_t>(static_cast<uint32_t>(rd[k][i] & 0xFFF) << 20) >> 20);
        const double Q =
            static_cast<double>(static_cast<int32_t>(static_cast<uint32_t>((rd[k][i] >> 12) & 0xFFF) << 20) >> 20);
        si += I; sq += Q; si2 += I * I; sq2 += Q * Q;
      }
      const double rms = std::sqrt((si2 - si * si / n + sq2 - sq * sq / n) / n);
      std::printf("{\"ev\":\"ring.stats\",\"k\":%d,\"rms\":%.1f,\"dc_i\":%.1f,\"dc_q\":%.1f,\"adj_dup\":%.4f,"
                  "\"zero\":%.4f,\"hi_distinct\":%zu,\"hi_top\":\"0x%010" PRIx64 "\",\"hi_top_frac\":%.4f}\n",
                  k, rms, si / n, sq / n, double(adj) / n, double(zero) / n, hi.size(), top, double(topn) / n);
    }
    for (int k = 1; k < a.reads; k++) {
      size_t same = 0;
      for (size_t i = 0; i < n; i++)
        same += (rd[k][i] & 0xFFFFFF) == (rd[k - 1][i] & 0xFFFFFF);
      std::printf("{\"ev\":\"ring.fresh\",\"pair\":\"%d-%d\",\"identical\":%.4f,\"verdict\":\"%s\"}\n", k - 1, k,
                  double(same) / n, double(same) / n >= 0.95 ? "FROZEN" : "LIVE");
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ring: %s\n", e.what());
    rc = 1;
  }
  la.restore();
  std::printf("{\"ev\":\"ring.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

// Contiguous snapshot: run the ring free, fire the manual trigger (0x7c0[5] edge; the post-trigger
// time 0x7c0[14:8] is 0), wait for the engine to stop, read the ring and unroll it oldest-first
// from the finish address (devourer LaCapture::readback semantics). Output: DVLA v1 files.
int cmd_snap(Session &s, const Args &a) {
  Reg &r = s.reg();
  La8822b la(r);
  la.snapshot();
  int rc = 0;
  try {
    la.configure(a.la);
    std::vector<uint64_t> out;
    const int count = a.count > 0 ? a.count : 1;
    for (int k = 0; k < count; k++) {
      const SnapInfo si = la.snap(out, static_cast<unsigned>(k ? 5 : a.settle_ms) * 1000u);
      const uint32_t fin = (si.r7c0 & 0x7FFF0000u) >> 16; // 8-byte units, TXFF address
      std::printf("{\"ev\":\"snap\",\"k\":%d,\"done\":%d,\"r7c0\":\"0x%08x\",\"wrap\":%d,\"samples\":%zu,\"read_ms\":%.1f}\n",
                  k, si.done ? 1 : 0, si.r7c0, si.wrap ? 1 : 0, si.samples, si.read_ms);
      if (!a.dump.empty() && si.samples) {
        uint8_t hdr[32] = {'D', 'V', 'L', 'A', 1, 1, 3, static_cast<uint8_t>(a.la.rate_sel),
                           static_cast<uint8_t>(a.la.dma_type), static_cast<uint8_t>(si.wrap)};
        const uint32_t n32 = static_cast<uint32_t>(si.samples);
        std::memcpy(hdr + 12, &n32, 4);
        std::memcpy(hdr + 16, &fin, 4);
        std::string fn = a.dump + "-snap" + std::to_string(k) + ".bin";
        if (FILE *f = std::fopen(fn.c_str(), "wb")) {
          std::fwrite(hdr, 1, 32, f);
          std::fwrite(out.data(), 8, si.samples, f);
          std::fclose(f);
        }
      }
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "snap: %s\n", e.what());
    rc = 1;
  }
  la.restore();
  std::printf("{\"ev\":\"snap.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

int cmd_marker(Session &s, const Args &) {
  Reg &r = s.reg();
  Cpu8822b cpu(r);
  int rc = 0;
  try {
    const uint32_t r80 = r.r16(0x0080);
    cpu.load(marker8822b_bin, marker8822b_bin_len);
    std::printf("{\"ev\":\"cpu.loaded\",\"bytes\":%zu,\"prog\":%u,\"r80_before\":\"0x%04x\"}\n",
                cpu.loaded_bytes(), marker8822b_bin_len, r80);
    const uint32_t zero[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    cpu.dmem_write(0x1000, zero, sizeof zero);
    cpu.start();
    uint32_t a[8], b[8];
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    cpu.dmem_read(0x1000, a, sizeof a);
    auto t0 = clk::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    cpu.dmem_read(0x1000, b, sizeof b);
    const double wall_us = ms_since(t0) * 1e3;
    const bool ok = a[0] == 0xCAFEF00Du && b[1] > a[1] && b[2] != a[2];
    std::printf("{\"ev\":\"cpu.marker\",\"ok\":%d,\"magic\":\"0x%08x\",\"count_a\":%u,\"count_b\":%u,"
                "\"loops_per_s\":%.0f,\"timer_a\":%u,\"timer_b\":%u,\"timer_delta\":%u,\"wall_us\":%.0f,"
                "\"r80\":\"0x%04x\",\"lui_ori\":\"0x%08x\",\"lui_xori\":\"0x%08x\",\"lui_andi\":\"0x%08x\"}\n",
                ok, a[0], a[1], b[1], (b[1] - a[1]) / (wall_us / 1e6), a[2], b[2], b[2] - a[2], wall_us,
                r.r16(0x0080), b[4], b[5], b[6]);
    rc = ok ? 0 : 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "marker: %s\n", e.what());
    rc = 1;
  }
  try {
    cpu.halt();
  } catch (...) {
  }
  std::printf("{\"ev\":\"marker.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

uint32_t pctl(const std::vector<uint32_t> &v, unsigned p) {
  if (v.empty())
    return 0;
  return v[(v.size() - 1) * p / 100];
}

// Host-issued C2H: validates the request encoding, descriptor framing and the bulk-IN path without
// any on-chip code. `tx0`/`tx1` additionally confirm ac54's LA-TX -> C2H alias on this unit.
int cmd_c2h(Session &s, const Args &a) {
  Reg &r = s.reg();
  C2hRxPath rx(r);
  int rc = 1;
  int completed = 0;
  unsigned desc_bad = 0, stray_seen = 0, timeout_seen = 0, retries = 0;
  C2hReader rd(s.handle(), s.context(), static_cast<size_t>(std::max(a.xfer, 4096)), a.depth);
  Cpu8822b cpu(r);
  try {
    if (a.halt_cpu)
      cpu.halt();
    rx.apply(a.rx_on);
    if (a.cr >= 0)
      r.w16(0x0100, static_cast<uint16_t>(a.cr));
    if (a.agg_th >= 0)
      r.m32(0x0280, 0xFF, static_cast<uint32_t>(a.agg_th));
    if (a.rxdma_mode >= 0)
      r.w8(0x0290, static_cast<uint8_t>(a.rxdma_mode));
    if (a.no_lpm) {
      for (unsigned f = 48; f <= 49; f++)
        libusb_control_transfer(s.handle(), 0x00, 1, static_cast<uint16_t>(f), 0, nullptr, 0, 1000);
    }
    for (auto [addr, val] : a.sets)
      r.w32(addr, val);
    if (a.rw_release >= 0) {
      uint32_t v = r.r32(0x0284);
      v = a.rw_release ? (v | (1u << 18)) : (v & ~(1u << 18));
      r.w32(0x0284, v);
    }
    const size_t payload = static_cast<size_t>(std::clamp(a.bytes, 64, 16352));
    const int count = a.count > 0 ? a.count : 64;
    uint32_t ocp = 0, addr = 0;
    if (a.source == "rx") {
      ocp = 0x18700000u + static_cast<uint32_t>(a.rx_off & ~7);
      addr = static_cast<uint32_t>(a.rx_off & ~7);
    } else if (a.source == "tx0") {
      ocp = La8822b::kRingOcp;
      addr = 0x16000;
    } else if (a.source == "tx1") {
      ocp = La8822b::kRingOcp + 0x10000;
      addr = 0x6000;
    } else {
      throw std::runtime_error("--source must be rx, tx0 or tx1");
    }
    const int slots = a.vary ? (a.source == "rx" ? 3 : 4) : 1;
    std::vector<C2hPacket> pkts;
    for (int s = 0; s < slots; s++)
      pkts.push_back(C2hPacket::make(payload, 0x67a72b31u + static_cast<uint32_t>(s) * 0x9e3779b9u));
    for (int s = 0; s < slots; s++)
      c2h_stage(r, ocp + static_cast<uint32_t>(s) * 0x4000u, pkts[s]);
    if (!rd.start())
      throw std::runtime_error("bulk-IN submit failed");
    rd.drain(30);
    const auto t0 = clk::now();
    for (int i = 0; i < count; i++) {
      const int slot = a.vary ? (i % slots) : 0;
      const uint32_t socp = ocp + static_cast<uint32_t>(slot) * 0x4000u;
      const uint32_t saddr = addr + static_cast<uint32_t>(slot) * 0x4000u;
      C2hPacket unique_pkt;
      const C2hPacket *pkt = &pkts[static_cast<size_t>(slot)];
      if (a.unique) {
        unique_pkt = C2hPacket::make(payload, 0x67a72b31u + static_cast<uint32_t>(i) * 0x9e3779b9u);
        c2h_stage(r, socp, unique_pkt);
        pkt = &unique_pkt;
      } else if (a.restage || a.vary) {
        c2h_stage(r, socp, *pkt);
      }
      if (a.check_desc) {
        uint8_t d[24];
        r.sram_read(socp, d, sizeof d);
        if (std::memcmp(d, pkt->frame.data(), 12) != 0)
          desc_bad++;
      }
      const uint32_t req = c2h_request(saddr);
      r.w32(0x0294, req);
      if (!a.no_poll) {
        bool clear = false;
        const int attempt_ms = a.retry_ms > 0 ? a.retry_ms : 2000;
        for (int attempt = 0; !clear && attempt <= 5; attempt++) {
          const auto dl = clk::now() + std::chrono::milliseconds(attempt_ms);
          while (!clear && clk::now() < dl)
            clear = (r.r32(0x0294) & 0x10000u) == 0;
          if (!clear && a.retry_ms > 0 && attempt < 5) {
            if (a.abort_retry) {
              r.w32(0x0294, 0);
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            r.w32(0x0294, req);
            retries++;
          }
        }
        if (!clear)
          throw std::runtime_error("C2H request did not complete");
      }
      if (rd.wait(*pkt, true, 2000) != 1)
        throw std::runtime_error("no verified bulk-IN completion");
      completed = i + 1;
      if (a.gap_us > 0)
        std::this_thread::sleep_for(std::chrono::microseconds(a.gap_us));
    }
    const double ms = ms_since(t0);
    std::printf("{\"ev\":\"c2h.host\",\"source\":\"%s\",\"bytes\":%zu,\"count\":%d,\"verified\":%u,"
                "\"stray\":%u,\"timeouts\":%u,\"retries\":%u,\"ms\":%.1f,\"mbps\":%.2f}\n",
                a.source.c_str(), payload, count, rd.verified(), rd.stray(), rd.timeouts(), retries, ms,
                ms > 0 ? payload * count / ms / 1e3 : 0.0);
    rc = 0;
  } catch (const std::exception &e) {
    stray_seen = rd.stray();
    timeout_seen = rd.timeouts();
    uint32_t r294 = 0, r288 = 0, r280 = 0, r10c = 0, r284 = 0;
    try {
      r294 = r.r32(0x0294);
      r288 = r.r32(0x0288);
      r280 = r.r32(0x0280);
      r10c = r.r32(0x010C);
      r284 = r.r32(0x0284);
    } catch (...) {
    }
    std::printf("{\"ev\":\"c2h.fail\",\"completed\":%d,\"desc_bad\":%u,\"stray\":%u,"
                "\"timeouts\":%u,\"retries\":%u,\"r294\":\"0x%08x\",\"r288\":\"0x%08x\","
                "\"r280\":\"0x%08x\",\"r10c\":\"0x%08x\",\"r284\":\"0x%08x\"}\n",
                completed, desc_bad, stray_seen, timeout_seen, retries, r294, r288, r280, r10c, r284);
    std::fprintf(stderr, "c2h: %s\n", e.what());
  }
  rx.restore();
  std::printf("{\"ev\":\"c2h.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

// CPU-issued C2H: per-request latency from REG 0x568, stamped on-chip. Measures the transport tail
// (the 40 Msps blocker) independently of host USB round-trips.
int cmd_c2hbench(Session &s, const Args &a) {
  Reg &r = s.reg();
  C2hRxPath rx(r);
  Cpu8822b cpu(r);
  const int count = std::clamp(a.count > 0 ? a.count : 1024, 1, 1600);
  const size_t payload = static_cast<size_t>(std::clamp(a.bytes, 64, 16352));
  C2hReader rd(s.handle(), s.context(), static_cast<size_t>(std::max(a.xfer, 4096)), a.depth);
  int rc = 1;
  try {
    const C2hPacket pkt = C2hPacket::make(payload);
    cpu.load(c2hbench8822b_bin, c2hbench8822b_bin_len);
    rx.apply(a.rx_on, a.fwff_off);
    c2h_stage(r, 0x18701000, pkt);
    const uint32_t seed = static_cast<uint32_t>(count);
    cpu.dmem_write(0x1000 + 0x18, &seed, sizeof seed);
    if (!rd.start())
      throw std::runtime_error("bulk-IN submit failed");
    rd.drain(20);
    const auto t0 = clk::now();
    cpu.start();
    unsigned ok = 0;
    const auto dl = clk::now() + std::chrono::seconds(15);
    while (ok < static_cast<unsigned>(count) && clk::now() < dl) {
      if (rd.wait(pkt, true, 1500) == 1)
        ok++;
      else
        break;
    }
    const double wall_ms = ms_since(t0);
    cpu.halt();
    rd.stop();

    uint32_t hdr[16] = {0};
    cpu.dmem_read(0x1000, hdr, sizeof hdr);
    const uint32_t issued = std::min<uint32_t>(hdr[1], static_cast<uint32_t>(count));
    std::vector<uint32_t> dur(issued);
    if (issued)
      cpu.dmem_read(0x1000 + 0x40, dur.data(), dur.size() * 4);
    std::vector<uint32_t> sorted = dur;
    std::sort(sorted.begin(), sorted.end());
    uint64_t sum = 0;
    unsigned over = 0;
    for (uint32_t d : dur) {
      sum += d;
      if (d > 204)
        over++;
    }
    const double mean = dur.empty() ? 0 : static_cast<double>(sum) / dur.size();
    if (!a.dump.empty() && !dur.empty()) {
      if (FILE *f = std::fopen(a.dump.c_str(), "w")) {
        for (size_t i = 0; i < dur.size(); i++)
          std::fprintf(f, "%zu,%u\n", i, dur[i]);
        std::fclose(f);
      }
    }
    std::printf("{\"ev\":\"c2h.bench\",\"n\":%d,\"issued\":%u,\"verified\":%u,\"stray\":%u,"
                "\"timeouts\":%u,\"magic\":\"0x%08x\",\"done\":%u,\"error\":%u,\"min_us\":%u,"
                "\"p50_us\":%u,\"p90_us\":%u,\"p99_us\":%u,\"max_us\":%u,\"over_204us\":%u,"
                "\"mean_us\":%.1f,\"wall_ms\":%.1f,\"mbps\":%.2f}\n",
                count, hdr[1], ok, rd.stray(), rd.timeouts(), hdr[0], hdr[2], hdr[3],
                sorted.empty() ? 0 : sorted.front(), pctl(sorted, 50), pctl(sorted, 90),
                pctl(sorted, 99), sorted.empty() ? 0 : sorted.back(), over, mean, wall_ms,
                wall_ms > 0 ? payload * ok / wall_ms / 1e3 : 0.0);
    rc = (hdr[0] == 0x0C2B0001u && hdr[2] == 1u && hdr[1] == static_cast<uint32_t>(count) &&
          ok == static_cast<unsigned>(count))
             ? 0
             : 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "c2hbench: %s\n", e.what());
    try {
      cpu.halt();
    } catch (...) {
    }
  }
  rx.restore();
  std::printf("{\"ev\":\"c2h.bench.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

// CPU-timed LA arm -> trigger: ring position after D us of the chip's own clock. Not wrapped
// (D < ring period) it measures fs and the arm latency; wrapped, pos mod 16384 checks drift.
int cmd_latime(Session &s, const Args &a) {
  Reg &r = s.reg();
  La8822b la(r);
  Cpu8822b cpu(r);
  la.snapshot();
  int rc = 1;
  try {
    cpu.load(latime8822b_bin, latime8822b_bin_len);
    la.configure(a.la); // leaves the engine stopped; the CPU arms it
    const uint32_t d = static_cast<uint32_t>(a.d_us);
    const uint32_t seed[8] = {0, 0, 0, 0, 0, 0, d, 0};
    cpu.dmem_write(0x1000, seed, sizeof seed);
    cpu.start();
    std::this_thread::sleep_for(std::chrono::microseconds(d + 20000));
    uint32_t st[8] = {0};
    for (int i = 0; i < 50 && st[4] != 1; i++) {
      cpu.dmem_read(0x1000, st, sizeof st);
      if (st[4] != 1)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    bool stopped = false;
    for (int i = 0; i < 200 && !stopped; i++)
      stopped = !(r.r8(0x07C0) & 0x02);
    const uint32_t r7c0 = r.r32(0x07C0);
    cpu.halt();
    const uint32_t fin = (r7c0 & 0x7FFF0000u) >> 16;
    const int wrap = static_cast<int>(r7c0 >> 31);
    const double fs = 80e6 / (1u << a.la.rate_sel);
    const int64_t pos = static_cast<int64_t>(fin) - 0x4000;
    const double elapsed = static_cast<double>(st[3] - st[2]); // us, arm -> after trigger
    const double expect = elapsed * fs / 1e6;
    std::printf("{\"ev\":\"latime\",\"rate\":%d,\"fs\":%.0f,\"d_us\":%u,\"magic\":\"0x%08x\",\"done\":%u,"
                "\"arm_write_us\":%u,\"elapsed_us\":%.0f,\"r7c0\":\"0x%08x\",\"stopped\":%d,\"wrap\":%d,"
                "\"pos\":%lld,\"expect_mod\":%.1f,\"err_samples\":%.1f,\"lat_us\":%.2f}\n",
                a.la.rate_sel, fs, d, st[0], st[4], st[2] - st[1], elapsed, r7c0, stopped ? 1 : 0, wrap,
                static_cast<long long>(pos), std::fmod(expect, 16384.0),
                std::remainder(expect - static_cast<double>(pos), 16384.0),
                std::remainder(expect - static_cast<double>(pos), 16384.0) / fs * 1e6);
    rc = st[0] == 0x1A7E0001u && st[4] == 1 && stopped ? 0 : 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "latime: %s\n", e.what());
    try {
      cpu.halt();
    } catch (...) {
    }
  }
  la.restore();
  std::printf("{\"ev\":\"latime.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

void dump_regs(Reg &r, const std::string &fn) {
  FILE *f = std::fopen(fn.c_str(), "w");
  if (!f)
    throw std::runtime_error("cannot write " + fn);
  for (uint32_t base : {0x0000u, 0x1000u})
    for (uint32_t a = base; a < base + 0x800; a += 16) {
      std::fprintf(f, "%08x  ", a);
      for (uint32_t o = 0; o < 16; o += 4)
        std::fprintf(f, "%08x    ", r.r32(static_cast<uint16_t>(a + o)));
      std::fprintf(f, "\n");
    }
  for (uint32_t a = 0xfe00; a < 0xfe80; a += 4)
    std::fprintf(f, "0x%x reg 0x%x: 0x%08x\n", a, a, r.r32(static_cast<uint16_t>(a)));
  std::fclose(f);
}

int cmd_regs(Session &s, const Args &a) {
  Reg &r = s.reg();
  if (a.dump.empty())
    throw std::runtime_error("regs needs --dump PREFIX");
  dump_regs(r, a.dump + "-init.txt");
  C2hRxPath rx(r);
  rx.apply(a.rx_on);
  dump_regs(r, a.dump + "-c2h.txt");
  rx.restore();
  std::printf("{\"ev\":\"regs\",\"files\":\"%s-{init,c2h}.txt\",\"alive\":%d}\n", a.dump.c_str(),
              r.alive() ? 1 : 0);
  return 0;
}

// BB dbg-port survey (phydm_dbg_port_dump-style): 0x8fc = selector, value at 0xfa0.
int cmd_dbgscan(Session &s, const Args &a) {
  Reg &r = s.reg();
  const uint32_t s8fc = r.r32(0x08FC), s198c = r.r32(0x198C), s8f8 = r.r32(0x08F8);
  int rc = 0;
  try {
    r.m32(0x198C, 0x7, 0x7);
    r.m32(0x08F8, 0x3C00000u, 0);
    const int n = std::clamp(a.reads, 2, 64);
    for (int sel = a.lo; sel <= a.hi; sel++) {
      r.w32(0x08FC, static_cast<uint32_t>(sel));
      std::vector<uint32_t> v(static_cast<size_t>(n));
      for (auto &x : v)
        x = r.r32(0x0FA0);
      uint32_t tog = 0;
      for (auto x : v)
        tog |= x ^ v[0];
      std::vector<uint32_t> u = v;
      std::sort(u.begin(), u.end());
      const size_t distinct = static_cast<size_t>(std::unique(u.begin(), u.end()) - u.begin());
      std::printf("{\"ev\":\"dbg\",\"sel\":\"0x%03x\",\"v0\":\"0x%08x\",\"distinct\":%zu,\"toggle\":\"0x%08x\"}\n", sel,
                  v[0], distinct, tog);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "dbgscan: %s\n", e.what());
    rc = 1;
  }
  try {
    r.w32(0x08FC, s8fc);
    r.w32(0x198C, s198c);
    r.w32(0x08F8, s8f8);
  } catch (...) {
  }
  std::printf("{\"ev\":\"dbgscan.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

struct PlanEntry {
  std::string label;
  LaRingCfg la;
  int count = 1;                          // snapshots appended to the dump
  std::vector<std::array<uint32_t, 3>> w; // addr, mask, val
};

// Many LA configurations in one session (no per-config ~1 s bring-up).
int cmd_tapscan(Session &s, const Args &a) {
  Reg &r = s.reg();
  if (a.plan.empty() || a.dump.empty())
    throw std::runtime_error("tapscan needs --plan FILE and --dump PREFIX");
  std::vector<PlanEntry> plan;
  if (FILE *f = std::fopen(a.plan.c_str(), "r")) {
    char line[1024];
    while (std::fgets(line, sizeof line, f)) {
      std::string l = line;
      if (const size_t h = l.find('#'); h != std::string::npos)
        l.resize(h);
      PlanEntry e;
      e.la = a.la;
      size_t pos = 0;
      while (pos < l.size()) {
        const size_t b = l.find_first_not_of(" \t\r\n", pos);
        if (b == std::string::npos)
          break;
        const size_t en = l.find_first_of(" \t\r\n", b);
        const std::string tok = l.substr(b, en == std::string::npos ? std::string::npos : en - b);
        pos = en == std::string::npos ? l.size() : en;
        const size_t eq = tok.find('=');
        if (e.label.empty() && eq == std::string::npos) { e.label = tok; continue; }
        const std::string k = tok.substr(0, eq), val = eq == std::string::npos ? "" : tok.substr(eq + 1);
        if (k == "dbg") e.la.dbg_port = static_cast<uint32_t>(num(val.c_str()));
        else if (k == "dma") e.la.dma_type = static_cast<int>(num(val.c_str()));
        else if (k == "rate") e.la.rate_sel = static_cast<int>(num(val.c_str()));
        else if (k == "count") e.count = std::max(1, static_cast<int>(num(val.c_str())));
        else if (k == "w") {
          const size_t s1 = val.find('/'), s2 = val.find('/', s1 + 1);
          if (s1 == std::string::npos || s2 == std::string::npos)
            throw std::runtime_error("bad w= " + val);
          e.w.push_back({static_cast<uint32_t>(num(val.substr(0, s1).c_str())),
                         static_cast<uint32_t>(num(val.substr(s1 + 1, s2 - s1 - 1).c_str())),
                         static_cast<uint32_t>(num(val.substr(s2 + 1).c_str()))});
        } else
          throw std::runtime_error("bad plan token " + tok);
      }
      if (!e.label.empty())
        plan.push_back(e);
    }
    std::fclose(f);
  } else {
    throw std::runtime_error("cannot read " + a.plan);
  }
  La8822b la(r);
  la.snapshot();
  std::map<uint32_t, uint32_t> orig; // extra-write originals, restored after every entry
  for (auto &e : plan)
    for (auto &w : e.w)
      if (!orig.count(w[0]))
        orig[w[0]] = r.r32(static_cast<uint16_t>(w[0]));
  int rc = 0;
  const size_t n = La8822b::kRingBytes / 8;
  try {
    for (auto &e : plan) {
      la.configure(e.la);
      for (auto &w : e.w)
        r.m32(static_cast<uint16_t>(w[0]), w[1], w[2]);
      std::vector<uint64_t> out0, out1, raw0, raw1;
      const SnapInfo s0 = la.snap(out0, static_cast<unsigned>(a.settle_ms) * 1000u, &raw0);
      const SnapInfo s1 = la.snap(out1, 2000u, &raw1);
      size_t same = 0, adj = 0, adj64 = 0;
      for (size_t i = 0; i < n; i++)
        same += raw0[i] == raw1[i];
      for (size_t i = 1; i < out0.size(); i++) {
        adj += (out0[i] & 0xFFFFFF) == (out0[i - 1] & 0xFFFFFF);
        adj64 += out0[i] == out0[i - 1];
      }
      std::vector<uint64_t> u = out0;
      std::sort(u.begin(), u.end());
      const size_t distinct = static_cast<size_t>(std::unique(u.begin(), u.end()) - u.begin());
      uint64_t tog = 0;
      for (auto x : out0)
        tog |= x ^ (out0.empty() ? 0 : out0[0]);
      std::printf("{\"ev\":\"tap\",\"label\":\"%s\",\"dbg\":\"0x%03x\",\"dma\":%d,\"rate\":%d,\"done\":%d,\"done1\":%d,"
                  "\"wrap\":%d,\"samples\":%zu,\"same_raw\":%.4f,\"adj24\":%.4f,\"adj64\":%.4f,\"distinct\":%zu,"
                  "\"toggle\":\"0x%016" PRIx64 "\",\"r95c\":\"0x%08x\",\"r8fc\":\"0x%08x\",\"r8f8\":\"0x%08x\","
                  "\"r948\":\"0x%08x\",\"r94c\":\"0x%08x\",\"rc20\":\"0x%08x\",\"r8ac\":\"0x%08x\",\"r8c4\":\"0x%08x\"}\n",
                  e.label.c_str(), e.la.dbg_port, e.la.dma_type, e.la.rate_sel, s0.done ? 1 : 0, s1.done ? 1 : 0,
                  s0.wrap ? 1 : 0, s0.samples, double(same) / n, out0.empty() ? 0.0 : double(adj) / out0.size(),
                  out0.empty() ? 0.0 : double(adj64) / out0.size(), distinct, tog, r.r32(0x095C), r.r32(0x08FC),
                  r.r32(0x08F8), r.r32(0x0948), r.r32(0x094C), r.r32(0x0C20), r.r32(0x08AC), r.r32(0x08C4));
      std::fflush(stdout);
      if (!out0.empty())
        if (FILE *f = std::fopen((a.dump + "-" + e.label + ".u64").c_str(), "wb")) {
          std::fwrite(out0.data(), 8, out0.size(), f);
          for (int k = 2; k < e.count; k++) { // extra snapshots (snapshot 1 is the freshness check)
            la.snap(out1, 2000u);
            std::fwrite(out1.data(), 8, out1.size(), f);
          }
          std::fclose(f);
        }
      for (auto it = orig.rbegin(); it != orig.rend(); ++it)
        r.w32(static_cast<uint16_t>(it->first), it->second);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "tapscan: %s\n", e.what());
    rc = 1;
  }
  for (auto it = orig.rbegin(); it != orig.rend(); ++it)
    try {
      r.w32(static_cast<uint16_t>(it->first), it->second);
    } catch (...) {
    }
  la.restore();
  std::printf("{\"ev\":\"tapscan.done\",\"alive\":%d}\n", r.alive() ? 1 : 0);
  return rc;
}

std::atomic<bool> g_stop{false};

int cmd_stream(Session &s, const Args &a) {
  StreamCfg c;
  c.la = a.la;
  if (!a.rate_given)
    c.la.rate_sel = 5;
  c.synthetic = a.synthetic;
  // >= 20 MS/s needs the Receiver's depth: 256 URBs (5 a bank: ~51 banks, ~10.5 ms at 40 MS/s); 32
  // (~6 banks, ~1.3 ms) missed the deadline within 8-14 s of a 30-s run
  c.depth = std::clamp(a.depth > 16 ? a.depth : c.la.rate_sel <= 2 ? 256 : 32, 4, 512);
  c.guard_us = a.guard_us;
  c.gapped = a.gapped;
  Stream8822b st(s, c);
  if (!(a.seconds > 0) || (!a.gapped && !(a.seconds < 1e6)))
    throw std::runtime_error("--seconds must be > 0 (and finite unless --gapped)");
  if (!a.gapped && c.la.rate_sel == 0)
    throw std::runtime_error("80 MS/s (--rate 0) cannot stream: add --gapped");
  const uint64_t want = a.gapped ? UINT64_MAX : static_cast<uint64_t>(a.seconds * st.fs() / Stream8822b::kBankSamples + 0.5);
  const auto t_start = std::chrono::steady_clock::now();
  if (a.path != "a" && a.path != "b" && a.path != "ab")
    throw std::runtime_error("--path is a, b or ab");
  const bool want_b = a.path != "a";
  if (want_b && c.la.dma_type != 4 && c.la.dma_type != 5) // other formats carry no path B in [47:24]
    throw std::runtime_error("--path b|ab needs --dma 4 or 5 (path B in bits [47:24])");
  // the file for the first path written (A, or B alone), and PREFIX-b.cs16 for B with --path ab
  FILE *f = nullptr, *fb = nullptr;
  if (!a.out.empty() && !(f = std::fopen((a.out + ".cs16").c_str(), "wb"))) {
    std::fprintf(stderr, "stream: cannot open %s.cs16\n", a.out.c_str());
    return 1;
  }
  if (!a.out.empty() && a.path == "ab" && !(fb = std::fopen((a.out + "-b.cs16").c_str(), "wb"))) {
    std::fprintf(stderr, "stream: cannot open %s-b.cs16\n", a.out.c_str());
    std::fclose(f);
    return 1;
  }
  std::signal(SIGINT, [](int) { g_stop = true; });
  // The USB callback only copies each bank into a ring of preallocated buffers; a writer thread
  // unpacks and writes them. A file write can sleep for a long time (the kernel throttles a process
  // with much dirty data, balance_dirty_pages), and the callback has the chip's deadline: writing from
  // it missed the deadline in 4 of 4 3-s runs of path A alone (2026-10-27). Zeroed up front, so the
  // ring never page-faults in the callback.
  constexpr uint64_t kRing = 2048; // banks: 128 MiB, ~0.4 s at 40 MS/s
  constexpr size_t kB = Stream8822b::kBankBytes;
  std::vector<uint8_t> ring(kRing * kB);
  std::vector<BankInfo> infos(kRing);
  std::atomic<uint64_t> put{0}, got{0};
  std::atomic<bool> done{false}, overflow{false};
  const uint64_t every = a.gapped ? 250 : std::max<uint64_t>(1, static_cast<uint64_t>(st.fs() / Stream8822b::kBankSamples));
  double e2 = 0;
  std::atomic<bool> wrote{true};
  std::thread writer([&] {
    std::vector<int16_t> iq(2 * Stream8822b::kBankSamples), iqb(want_b ? 2 * Stream8822b::kBankSamples : 0);
    for (uint64_t k = 0;; k++) {
      uint64_t p;
      while ((p = put.load(std::memory_order_acquire)) == k) { // (polled: a wait could miss the end)
        if (done.load(std::memory_order_acquire) && put.load(std::memory_order_acquire) == k)
          return;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      const uint8_t *bank = ring.data() + (k % kRing) * kB;
      const BankInfo &bi = infos[k % kRing];
      double acc = 0;
      for (size_t i = 0; i < Stream8822b::kBankSamples; i++) {
        uint64_t w;
        std::memcpy(&w, bank + 8 * i, 8);
        if (want_b) { // path B: the same 12+12 layout, 24 bits up
          iqb[2 * i] = static_cast<int16_t>(la_i(w >> 24) * 16);
          iqb[2 * i + 1] = static_cast<int16_t>(la_q(w >> 24) * 16);
        }
        const int I = la_i(w), Q = la_q(w);
        iq[2 * i] = static_cast<int16_t>(I * 16);
        iq[2 * i + 1] = static_cast<int16_t>(Q * 16);
        acc += double(I) * I + double(Q) * Q;
      }
      e2 += acc;
      const std::vector<int16_t> &first = a.path == "b" ? iqb : iq;
      if (wrote && ((f && std::fwrite(first.data(), 2, first.size(), f) != first.size()) ||
                    (fb && std::fwrite(iqb.data(), 2, iqb.size(), fb) != iqb.size())))
        wrote = false; // (the callback sees it and ends the run)
      if ((k + 1) % every == 0) {
        std::printf("{\"ev\":\"stream.tick\",\"banks\":%llu,\"seq\":%u,\"rms\":%.1f,\"late_us\":%.1f,"
                    "\"ship_us\":%.1f,\"end_us\":%.1f,\"stale\":%d,\"queued\":%llu}\n",
                    static_cast<unsigned long long>(k + 1), bi.seq, std::sqrt(acc / Stream8822b::kBankSamples),
                    bi.late_us, bi.ship_us, bi.end_us, bi.stale ? 1 : 0, static_cast<unsigned long long>(p - k));
        std::fflush(stdout);
      }
      got.store(k + 1, std::memory_order_release);
    }
  });
  struct Join { // the writer ends and is joined on every way out (an exception too)
    std::atomic<bool> &done;
    std::thread &t;
    ~Join() {
      done = true;
      if (t.joinable())
        t.join();
    }
  } join{done, writer};
  uint64_t n = 0;
  // Spike: IGI rewritten between banks, on the thread that reaps the stream (no restart).
  struct Hop { uint64_t bank; int igi; double us; uint32_t c50, e50, pre, r838; };
  std::vector<Hop> hops;
  const uint64_t hop_every = std::max<uint64_t>(1, static_cast<uint64_t>(a.hop_ms * 1e-3 * st.fs() / Stream8822b::kBankSamples));
  // mode 6: one EP0 transfer per bank. 0 idle, 1 read C50 (so a write that lands with old data is a no-op),
  // 2 write C50, 3 read C50 (differs -> 2), 4 write E50, 5 read E50 (differs -> 3).
  int ph = 0, want_igi = 0;
  uint32_t xfers = 0;
  uint64_t hop_bank = 0;
  const StreamStats ss = st.run(
      [&](const uint8_t *bank, const BankInfo &bi) {
        if (a.hop_igi >= 0 && a.hop_mode == 7) { // as 6, but step 1 reads the wanted value from the table
          Reg &hr = s.reg();
          if (!ph && n && n % hop_every == 0) {
            want_igi = hops.size() % 2 ? a.la.igi : a.hop_igi;
            ph = 1, xfers = 0, hop_bank = n;
          }
          if (ph) {
            xfers++;
            if (ph == 1) {
              const int got = hr.r8(static_cast<uint16_t>(0x8800 + want_igi));
              if (got != want_igi && hops.size() < 4)
                std::fprintf(stderr, "hop: table read %02x, wanted %02x\n", got, want_igi);
              ph = 2;
            }
            else if (ph == 2) hr.w8(0x0C50, static_cast<uint8_t>(want_igi)), ph = 3;
            else if (ph == 3) ph = hr.r8(0x0C50) == want_igi ? 4 : 1;
            else if (ph == 4) hr.w8(0x0E50, static_cast<uint8_t>(want_igi)), ph = 5;
            else if (hr.r8(0x0E50) == want_igi) {
              hops.push_back({hop_bank, want_igi, static_cast<double>(n - hop_bank + 1), xfers, 1, 0, 0});
              ph = 0;
            } else ph = 3;
          }
          goto hopped;
        }
        if (a.hop_igi >= 0 && a.hop_mode == 6) {
          Reg &hr = s.reg();
          if (!ph && n && n % hop_every == 0) {
            want_igi = hops.size() % 2 ? a.la.igi : a.hop_igi;
            ph = 1, xfers = 0, hop_bank = n;
          }
          if (ph) {
            xfers++;
            if (ph == 1) hr.r8(0x0C50), ph = 2;
            else if (ph == 2) hr.w8(0x0C50, static_cast<uint8_t>(want_igi)), ph = 3;
            else if (ph == 3) ph = hr.r8(0x0C50) == want_igi ? 4 : 2;
            else if (ph == 4) hr.w8(0x0E50, static_cast<uint8_t>(want_igi)), ph = 5;
            else if (hr.r8(0x0E50) == want_igi) {
              hops.push_back({hop_bank, want_igi, static_cast<double>(n - hop_bank + 1), xfers, 1, 0, 0});
              ph = 0;
            } else ph = 3;
          }
          goto hopped;
        }
        if (a.hop_igi >= 0 && n && n % hop_every == 0) {
          const int igi = hops.size() % 2 ? a.la.igi : a.hop_igi;
          const auto t = std::chrono::steady_clock::now();
          Reg &hr = s.reg();
          uint32_t pre = 0, r838 = 0;
          if (a.hop_mode == 4) { // no reads at all; every write twice
            for (uint16_t reg : {0x0C50, 0x0C50, 0x0E50, 0x0E50})
              hr.w8(reg, static_cast<uint8_t>(igi));
            hops.push_back({n, igi, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t).count(), 0, 0, 0, 0});
            goto hopped;
          }
          if (a.hop_mode == 5) { // byte write, read back twice, repeat until it holds; only IGI values ever on EP0
            uint32_t tries[2] = {0, 0};
            int k = 0;
            for (uint16_t reg : {0x0C50, 0x0E50}) {
              do {
                hr.w8(reg, static_cast<uint8_t>(igi));
                tries[k]++;
              } while ((hr.r8(reg) != igi || hr.r8(reg) != igi) && tries[k] < 64);
              k++;
            }
            hops.push_back({n, igi, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t).count(),
                            tries[0], tries[1], 0, 0});
            goto hopped;
          }
          pre = hr.r32(0x0C50), r838 = hr.r32(0x0838);
          if (a.hop_mode == 1 || a.hop_mode == 3)
            hr.w32(0x0838, r838 & ~2u);
          if (a.hop_mode >= 2) {
            hr.w8(0x0C50, static_cast<uint8_t>(igi));
            hr.w8(0x0E50, static_cast<uint8_t>(igi));
          } else {
            hr.m32(0x0C50, 0x7F, static_cast<uint32_t>(igi));
            hr.m32(0x0E50, 0x7F, static_cast<uint32_t>(igi));
          }
          if (a.hop_mode == 1 || a.hop_mode == 3)
            hr.w32(0x0838, r838 | 2u);
          const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t).count();
          hops.push_back({n, igi, us, hr.r32(0x0C50), hr.r32(0x0E50), pre, r838});
        }
      hopped:
        if (n - got.load(std::memory_order_acquire) >= kRing) { // the writer fell behind by 0.4 s
          overflow = true;
          return false;
        }
        std::memcpy(ring.data() + (n % kRing) * kB, bank, kB);
        infos[n % kRing] = bi;
        put.store(++n, std::memory_order_release);
        if (!wrote.load(std::memory_order_relaxed))
          return false;
        if (a.gapped)
          return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() < a.seconds;
        return n < want;
      },
      &g_stop);
  done = true;
  writer.join(); // (before e2, wrote and the files are used)
  for (const Hop &h : hops)
    std::printf("{\"ev\":\"stream.hop\",\"bank\":%llu,\"sample\":%llu,\"igi\":%d,\"write_us\":%.0f,\"pre\":\"%08x\",\"c50\":\"%08x\",\"e50\":\"%08x\",\"r838\":\"%08x\"}\n",
                static_cast<unsigned long long>(h.bank), static_cast<unsigned long long>(h.bank * Stream8822b::kBankSamples),
                h.igi, h.us, h.pre, h.c50, h.e50, h.r838);
  if (overflow)
    std::fprintf(stderr, "stream: the writer fell behind (%llu banks queued): the disk cannot keep up\n",
                 static_cast<unsigned long long>(kRing));
  if (!wrote)
    std::fprintf(stderr, "stream: writing %s failed\n", a.out.c_str());
  if (f)
    std::fclose(f);
  if (fb)
    std::fclose(fb);
  const double secs = ss.banks * Stream8822b::kBankSamples / st.fs();
  const bool ok = wrote && !overflow && ss.error.empty() && ss.stale == 0 && ss.pattern_bad == 0 &&
                  (a.gapped ? ss.banks > 0 : ss.banks >= want);
  char summary[1024];
  std::snprintf(summary, sizeof summary,
                "{\"ev\":\"stream\",\"ok\":%d,\"synthetic\":%d,\"rate_sel\":%d,\"fs\":%.0f,\"channel\":%d,"
                "\"banks\":%llu,\"samples\":%llu,\"stream_s\":%.3f,\"wall_s\":%.3f,\"mbps_wire\":%.2f,"
                "\"strays\":%u,\"start_retries\":%u,\"stale\":%u,\"pattern_bad\":%u,\"late_max_us\":%.1f,"
                "\"ship_max_us\":%.1f,\"end_max_us\":%.1f,\"budget_us\":%.1f,\"rms\":%.1f,\"error\":\"%s\"}",
                ok ? 1 : 0, c.synthetic ? 1 : 0, c.la.rate_sel, st.fs(), a.open.channel,
                static_cast<unsigned long long>(ss.banks),
                static_cast<unsigned long long>(ss.banks * Stream8822b::kBankSamples), secs, ss.wall_s,
                ss.wall_s > 0 ? ss.banks * 65536.0 / ss.wall_s / 1e6 : 0.0, ss.strays, ss.start_retries, ss.stale,
                ss.pattern_bad, ss.late_max_us, ss.ship_max_us, ss.end_max_us, st.period_us() - c.guard_us,
                n ? std::sqrt(e2 / (n * Stream8822b::kBankSamples)) : 0.0, ss.error.c_str());
  std::printf("%s\n", summary);
  if (!a.out.empty())
    if (FILE *j = std::fopen((a.out + ".json").c_str(), "w")) {
      std::fprintf(j, "{\"format\":\"cs16\",\"layout\":\"int16 I,Q interleaved, 12-bit LA value << 4\","
                      "\"sample_rate\":%.0f,\"chip\":\"RTL8822B\",\"path\":\"%s\",\"dma\":%d,\"channel\":%d,"
                      "\"igi\":%d,\"result\":%s}\n",
                   st.fs(), a.path == "a" ? "A" : a.path == "b" ? "B" : "A (+B in -b.cs16)", c.la.dma_type,
                   a.open.channel, a.la.igi, summary);
      std::fclose(j);
    }
  std::printf("{\"ev\":\"stream.done\",\"alive\":%d}\n", s.reg().alive() ? 1 : 0);
  return ok ? 0 : 1;
}

} // namespace

int main(int argc, char **argv) {
  Args a;
  if (!parse(argc, argv, a)) {
    std::fprintf(stderr,
                 "usage: wsdr_probe ring|snap|marker|c2h|c2hbench|latime|stream|regs|dbgscan|tapscan --pid 0xNNNN [options] "
                 "(see source header)\n");
    return 2;
  }
  auto logger = std::make_shared<Logger>();
  Session s(logger);
  if (a.cmd == "stream")
    a.open.thermal_track = false; // no host register traffic while streaming
  try {
    s.open(a.open);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "open/bring-up failed: %s\n", e.what());
    return 3;
  }
  std::printf("{\"ev\":\"session\",\"chip\":\"%s\",\"superspeed\":%d,\"channel\":%d,\"bw\":%d}\n",
              s.chip().c_str(), s.superspeed() ? 1 : 0, a.open.channel, a.open.bw);
  if (a.cmd == "ring")
    return cmd_ring(s, a);
  if (a.cmd == "snap")
    return cmd_snap(s, a);
  if (a.cmd == "marker")
    return cmd_marker(s, a);
  if (a.cmd == "c2h")
    return cmd_c2h(s, a);
  if (a.cmd == "c2hbench")
    return cmd_c2hbench(s, a);
  if (a.cmd == "latime")
    return cmd_latime(s, a);
  if (a.cmd == "regs")
    return cmd_regs(s, a);
  try {
    if (a.cmd == "stream")
      return cmd_stream(s, a);
    if (a.cmd == "dbgscan")
      return cmd_dbgscan(s, a);
    if (a.cmd == "tapscan")
      return cmd_tapscan(s, a);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "%s: %s\n", a.cmd.c_str(), e.what());
    return 1;
  }
  std::fprintf(stderr, "unknown command %s\n", a.cmd.c_str());
  return 2;
}
