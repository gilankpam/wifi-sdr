// SPDX-License-Identifier: GPL-2.0-only
// wsdr_txprobe — cabled-bench TX experiments for the RTL8822B (RTL8812BU).
//
// Every TX mode is gated on lab/test-setup.conf (cabled=yes, attenuation > 0,
// tx_paths non-empty, tx_adapter bus/port match) and lab/allowed_channels.txt.
// All register writes after bring-up are logged as JSONL (address, mask, old,
// new, timestamp) and restored in reverse order on exit. The hold is capped at
// 10 s; SIGINT/SIGTERM and a hard alarm end it, and the supervisor also resets
// the adapter after every run so residual TX cannot survive a crash.
//
//   wsdr_txprobe cw   --pid 0xb812 [--bus 7] [--port 1] --channel 36
//                     [--gain 0..31] [--seconds 3] [--log FILE]
//   wsdr_txprobe cont --pid 0xb812 ... [--seconds 3]
//   wsdr_txprobe ook  --pid 0xb812 ... [--pattern 1010] [--rate-hz 1000]
//                     [--ook-src lo|gain] [--gain N] [--seconds 3]
//   wsdr_txprobe raw  --pid 0xb812 ... [--bb ADDR=MASK:VALUE] [--rf P:ADDR=MASK:VALUE]
//                     [--seconds 3]
//   wsdr_txprobe reset --pid 0xb812 [--bus 7] [--port 1]
//       libusb_reset_device only; no bring-up, no TX. Used by the supervisor.
//
// The raw ops use vendor phy_set_bb_reg / rf_set semantics: VALUE is unshifted
// and shifted into MASK. With ADDR=VALUE (no ':') the full 32 bits are written.
#include <libusb.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/time.h>
#include <unistd.h>

#include "logger.h"
#include "rtlreg.h"
#include "txbench8822b.h"
#include "tx914sweep8822b.h"
#include "txfsk8822b.h"
#include "cpu88xx.h"
#include "session.h"

using namespace wsdr;

namespace {

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop = true; }

uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

std::string utc_stamp() {
  char buf[32];
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::strftime(buf, sizeof buf, "%Y%m%d-%H%M%S", &tm);
  return buf;
}

std::string trim(std::string s) {
  const auto notspace = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
  s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
  return s;
}

struct LabCfg {
  bool cabled = false;
  int attenuation_db = 0;
  std::string tx_paths;
  int tx_bus = -1;
  std::string tx_port;
  std::set<int> channels;
};

// tx_adapter=Bus 007.Port 001: Dev 001 -> 7 / "1"
bool parse_adapter(const std::string &s, int &bus, std::string &port) {
  const size_t b = s.find("Bus");
  const size_t p = s.find("Port", b == std::string::npos ? 0 : b);
  if (b == std::string::npos || p == std::string::npos)
    return false;
  size_t i = b + 3;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i])))
    i++;
  const size_t b0 = i;
  while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])))
    i++;
  if (i == b0)
    return false;
  bus = std::stoi(s.substr(b0, i - b0));
  i = p + 4;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i])))
    i++;
  const size_t p0 = i;
  while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '.'))
    i++;
  if (i == p0)
    return false;
  port = s.substr(p0, i - p0);
  while (port.size() > 1 && port.front() == '0' && port[1] != '.')
    port.erase(port.begin());
  return true;
}

bool load_lab(const std::string &conf_path, const std::string &chan_path, LabCfg &c, std::string &err) {
  std::ifstream conf(conf_path);
  if (!conf) {
    err = "cannot read " + conf_path;
    return false;
  }
  std::string line;
  bool have_adapter = false;
  while (std::getline(conf, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos)
      line.erase(hash);
    line = trim(line);
    if (line.empty())
      continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos)
      continue;
    const std::string k = trim(line.substr(0, eq));
    const std::string v = trim(line.substr(eq + 1));
    if (k == "cabled")
      c.cabled = (v == "yes" || v == "1" || v == "true");
    else if (k == "attenuation_db")
      c.attenuation_db = std::atoi(v.c_str());
    else if (k == "tx_paths")
      c.tx_paths = v;
    else if (k == "tx_adapter") {
      have_adapter = parse_adapter(v, c.tx_bus, c.tx_port);
      if (!have_adapter) {
        err = "cannot parse tx_adapter: " + v;
        return false;
      }
    }
  }
  if (!c.cabled) {
    err = "lab/test-setup.conf says cabled=no (or missing)";
    return false;
  }
  if (c.attenuation_db <= 0) {
    err = "attenuation_db must be > 0";
    return false;
  }
  if (c.tx_paths.empty()) {
    err = "tx_paths is empty";
    return false;
  }
  if (!have_adapter) {
    err = "tx_adapter missing";
    return false;
  }
  std::ifstream chans(chan_path);
  if (!chans) {
    err = "cannot read " + chan_path;
    return false;
  }
  std::string chan_line;
  while (std::getline(chans, chan_line)) {
    const size_t hash = chan_line.find('#');
    if (hash != std::string::npos)
      chan_line.erase(hash);
    for (size_t i = 0; i < chan_line.size();) {
      if (!std::isdigit(static_cast<unsigned char>(chan_line[i]))) {
        i++;
        continue;
      }
      const size_t j = i;
      while (i < chan_line.size() && std::isdigit(static_cast<unsigned char>(chan_line[i])))
        i++;
      c.channels.insert(std::stoi(chan_line.substr(j, i - j)));
    }
  }
  if (c.channels.empty()) {
    err = "allowed_channels list is empty";
    return false;
  }
  return true;
}

struct Args {
  std::string mode;
  OpenOpts open;
  std::string conf = "lab/test-setup.conf";
  std::string allowed = "lab/allowed_channels.txt";
  std::string log;
  int gain = 0;
  double seconds = 3;
  std::string pattern = "1010";
  double rate_hz = 1000;
  std::string ook_src = "lo";
  std::vector<std::string> bb_ops;
  std::vector<std::string> rf_ops;
  bool wait_stdin = false;
  std::string ready_file;
  int tone_index = 0x08;
  int tone_path = 0;
  double tone_repeat_ms = 0.0;
  bool tone_full = false;
  int tx_power = -1;
  uint32_t fw_reg = 0x1bcc;
  uint32_t fw_value = 0;
  uint32_t fw_count = 100000;
  uint32_t fsk_f1 = 8;
  uint32_t fsk_f2 = 16;
  uint32_t fsk_dwell_us = 1000;
  uint32_t fsk_cycles = 0;
  uint32_t fsk_amp = 0;
};

unsigned long num(const char *s) { return std::strtoul(s, nullptr, 0); }

bool parse(int argc, char **argv, Args &a) {
  if (argc < 2)
    return false;
  a.mode = argv[1];
  for (int i = 2; i < argc; i++) {
    const std::string k = argv[i];
    auto v = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
    if (k == "--pid") a.open.pid = static_cast<uint16_t>(num(v()));
    else if (k == "--vid") a.open.vid = static_cast<uint16_t>(num(v()));
    else if (k == "--bus") a.open.bus = static_cast<int>(num(v()));
    else if (k == "--port") a.open.port = v();
    else if (k == "--channel") a.open.channel = static_cast<int>(num(v()));
    else if (k == "--bw") a.open.bw = static_cast<int>(num(v()));
    else if (k == "--gain") a.gain = static_cast<int>(num(v()));
    else if (k == "--seconds") a.seconds = std::atof(v());
    else if (k == "--pattern") a.pattern = v();
    else if (k == "--rate-hz") a.rate_hz = std::atof(v());
    else if (k == "--ook-src") a.ook_src = v();
    else if (k == "--tone-index") a.tone_index = static_cast<int>(num(v()));
    else if (k == "--tone-path") a.tone_path = static_cast<int>(num(v()));
    else if (k == "--tone-repeat-ms") a.tone_repeat_ms = std::atof(v());
    else if (k == "--tone-full") a.tone_full = true;
    else if (k == "--tx-power") a.tx_power = static_cast<int>(num(v()));
    else if (k == "--fw-reg") a.fw_reg = static_cast<uint32_t>(num(v()));
    else if (k == "--fw-value") a.fw_value = static_cast<uint32_t>(num(v()));
    else if (k == "--fw-count") a.fw_count = static_cast<uint32_t>(num(v()));
    else if (k == "--fsk-f1") a.fsk_f1 = static_cast<uint32_t>(num(v()));
    else if (k == "--fsk-f2") a.fsk_f2 = static_cast<uint32_t>(num(v()));
    else if (k == "--fsk-dwell-us") a.fsk_dwell_us = static_cast<uint32_t>(num(v()));
    else if (k == "--fsk-cycles") a.fsk_cycles = static_cast<uint32_t>(num(v()));
    else if (k == "--fsk-amp") a.fsk_amp = static_cast<uint32_t>(num(v()));
    else if (k == "--bb") a.bb_ops.push_back(v());
    else if (k == "--rf") a.rf_ops.push_back(v());
    else if (k == "--log") a.log = v();
    else if (k == "--wait-stdin") a.wait_stdin = true;
    else if (k == "--ready-file") a.ready_file = v();
    else if (k == "--conf") a.conf = v();
    else if (k == "--allowed") a.allowed = v();
    else if (k == "--no-thermal") a.open.thermal_track = false;
    else return false;
  }
  if (!a.open.pid)
    a.open.pid = 0xb812;
  a.seconds = std::clamp(a.seconds, 0.1, 10.0);
  a.gain = std::clamp(a.gain, 0, 31);
  a.rate_hz = std::clamp(a.rate_hz, 1.0, 100000.0);
  a.tone_index = std::clamp(a.tone_index, 0, 0xfff);
  a.tone_path = std::clamp(a.tone_path, 0, 1);
  a.tone_repeat_ms = std::clamp(a.tone_repeat_ms, 0.0, 100.0);
  a.tx_power = std::clamp(a.tx_power, -1, 63);
  a.fw_reg &= 0xffffu;
  a.fw_count = std::clamp(a.fw_count, 1u, 20000000u);
  a.fsk_f1 &= 0xfff;
  a.fsk_f2 &= 0xfff;
  a.fsk_dwell_us = std::clamp(a.fsk_dwell_us, 1u, 1000000u);
  a.fsk_amp = std::clamp(a.fsk_amp, 0u, 63u);
  a.open.thermal_track = false;
  if (a.pattern.empty())
    a.pattern = "1";
  return true;
}

// JSONL register journal: first-seen old value per (rf,path,addr), written in
// reverse on restore. All writes after bring-up go through here.
class TxLog {
public:
  TxLog(Reg &r, const std::string &path) : _r(r) {
    if (path.empty())
      return;
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    _f = std::fopen(path.c_str(), "w");
    if (!_f)
      throw std::runtime_error("cannot write register log " + path);
  }
  ~TxLog() {
    if (_f)
      std::fclose(_f);
  }
  uint64_t writes() const { return _writes; }

  void bb_w(uint16_t addr, uint32_t val) {
    const uint32_t old = _r.r32(addr);
    save(false, 0, addr, old);
    _r.w32(addr, val);
    emit("bb_w", addr, 0xffffffffu, val, old, val);
  }
  void bb_m(uint16_t addr, uint32_t mask, uint32_t val) {
    const uint32_t old = _r.r32(addr);
    const uint32_t nv = (old & ~mask) | ((val << bshift(mask)) & mask);
    save(false, 0, addr, old);
    _r.w32(addr, nv);
    emit("bb_m", addr, mask, val, old, nv);
  }
  void bb_nr(uint16_t addr, uint32_t val) {
    _r.w32(addr, val);
    emit("bb_nr", addr, 0xffffffffu, val, 0, val);
  }
  uint32_t bb_r(uint16_t addr) { return _r.r32(addr); }
  uint32_t rf_get(int path, uint32_t addr) {
    return _r.r32(static_cast<uint16_t>((path ? 0x2c00u : 0x2800u) + ((addr & 0xff) << 2))) & 0xfffffu;
  }
  void rf_m(int path, uint32_t addr, uint32_t mask, uint32_t val) {
    const uint32_t old = rf_get(path, addr);
    uint32_t nv = val;
    if (mask != 0xfffffu)
      nv = (old & ~mask) | ((val << bshift(mask)) & mask);
    save(true, path, addr, old);
    rf_raw(path, addr, nv);
    emit("rf_m", static_cast<uint16_t>(addr), mask, val, old, nv);
  }
  void rf_raw(int path, uint32_t addr, uint32_t val) {
    const uint16_t lssi = path ? 0x0e90 : 0x0c90;
    const uint32_t data = (((addr & 0xff) << 20) | (val & 0xfffffu)) & 0x0fffffffu;
    _r.w32(lssi, data);
  }
  void restore() {
    for (auto it = _saved.rbegin(); it != _saved.rend(); ++it) {
      if (it->rf) {
        rf_raw(it->path, it->addr, it->old);
        emit("rf_restore", static_cast<uint16_t>(it->addr), 0xfffffu, it->old, it->old, it->old);
      } else {
        _r.w32(it->addr, it->old);
        emit("bb_restore", it->addr, 0xffffffffu, it->old, it->old, it->old);
      }
    }
    _saved.clear();
    _seen.clear();
  }

private:
  struct Saved {
    bool rf;
    int path;
    uint32_t addr;
    uint32_t old;
  };
  static unsigned bshift(uint32_t mask) {
    unsigned s = 0;
    while (s < 31 && !(mask & (1u << s)))
      s++;
    return s;
  }
  void save(bool rf, int path, uint32_t addr, uint32_t old) {
    const uint64_t key = (rf ? 0x100000000ull : 0ull) | (static_cast<uint64_t>(path) << 16) | addr;
    if (_seen.emplace(key, true).second)
      _saved.push_back({rf, path, addr, old});
  }
  void emit(const char *op, uint16_t addr, uint32_t mask, uint32_t val, uint32_t old, uint32_t nv) {
    _writes++;
    if (!_f)
      return;
    std::fprintf(_f,
                 "{\"t_ns\":%llu,\"op\":\"%s\",\"addr\":\"0x%x\",\"mask\":\"0x%x\","
                 "\"val\":\"0x%x\",\"old\":\"0x%x\",\"new\":\"0x%x\"}\n",
                 static_cast<unsigned long long>(now_ns()), op, addr, mask, val, old, nv);
    std::fflush(_f);
  }

  Reg &_r;
  FILE *_f = nullptr;
  uint64_t _writes = 0;
  std::vector<Saved> _saved;
  std::map<uint64_t, bool> _seen;
};

void hold_seconds(double seconds) {
  const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double>(seconds));
  while (!g_stop && Clock::now() < end)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// ---- register sequences (vendor hal_mpt_SetSingleToneTx 8822B branch) ----
void cw_arm(TxLog &l, int gain) {
  l.bb_m(0x0808, (1u << 29) | (1u << 28), 0x0);
  l.rf_m(0, 0x00, 0xf0000, 0x2);
  l.rf_m(0, 0x00, 0x1f, static_cast<uint32_t>(gain));
  l.rf_m(0, 0x58, 0x2, 0x1);
  l.bb_w(0x0cb0, 0x77777777);
  l.bb_w(0x0eb0, 0x77777777);
  l.bb_m(0x0cb4, 0xffff, 0x7777);
  l.bb_m(0x0eb4, 0xffff, 0x7777);
  l.bb_m(0x0cbc, 0xfff, 0xb);
  l.bb_m(0x0ebc, 0xfff, 0x830);
}

void cw_disarm(TxLog &l) {
  l.rf_m(0, 0x58, 0x2, 0x0);
  l.bb_m(0x0808, (1u << 29) | (1u << 28), 0x3);
  l.restore();
}

// ---- 0x914[18:16] OFDM continuous TX (devourer StartContinuousTx) ----
void cont_arm(TxLog &l) {
  l.bb_m(0x0838, 0xff, 0x6d);
  l.bb_m(0x0800, 1u << 25, 0x1);
  l.bb_m(0x0a00, 0x3, 0x0);
  l.bb_m(0x0a00, 0x8, 0x1);
  l.bb_m(0x0914, 0x70000, 0x1);
  l.bb_m(0x0b00, 0x100, 0x1);
  l.bb_m(0x0b04, 0xf, 0x4);
}

void cont_disarm(TxLog &l) {
  l.bb_m(0x0b04, 0xf, 0x2);
  l.bb_m(0x0914, 0x70000, 0x0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  l.bb_m(0x0b00, 0x100, 0x0);
  l.bb_m(0x0100, 0x100, 0x0);
  l.bb_m(0x0100, 0x100, 0x1);
  l.restore();
}


static void usleep_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

static void rf_set_check(TxLog &l, int path, uint32_t addr, uint32_t val) {
  l.rf_m(path, addr, 0xfffff, val);
  for (int i = 0; i < 100; i++) {
    if (l.rf_get(path, addr) == val)
      break;
    usleep_us(10);
    l.rf_m(path, addr, 0xfffff, val);
  }
}

static void tone_bb_reset(TxLog &l) {
  l.rf_m(0, 0x00, 0xfffff, 0x10000);
  l.rf_m(1, 0x00, 0xfffff, 0x10000);
  l.bb_m(0x08f8, 0x0ff00000, 0x0);
  for (int count = 0; count < 20000; count++) {
    l.bb_nr(0x08fc, 0x0);
    l.bb_m(0x198c, 0x7, 0x7);
    const bool cca = (l.bb_r(0x0fa0) & (1u << 3)) != 0;
    if (cca) {
      usleep_us(10);
      continue;
    }
    l.bb_m(0x0808, 0xff, 0x0);
    l.bb_m(0x0a04, (1u << 27) | (1u << 26) | (1u << 25) | (1u << 24), 0x0);
    l.bb_m(0x0000, 1u << 16, 0x0);
    l.bb_m(0x0000, 1u << 16, 0x1);
    if (l.bb_r(0x0660) & (1u << 16))
      l.bb_w(0x06b4, 0x89000006);
    break;
  }
}

static void tone_configure_macbb(TxLog &l) {
  l.bb_m(0x0522, 0xff, 0x7f);
  l.bb_m(0x0550, (1u << 11) | (1u << 3), 0x0);
  l.bb_m(0x090c, 1u << 15, 0x1);
  l.bb_m(0x0c94, 1u << 0, 0x1);
  l.bb_m(0x0e94, 1u << 0, 0x1);
  l.bb_m(0x0c94, (1u << 11) | (1u << 10), 0x1);
  l.bb_m(0x0e94, (1u << 11) | (1u << 10), 0x1);
  l.bb_w(0x0c00, 0x00000004);
  l.bb_w(0x0e00, 0x00000004);
  l.bb_m(0x0b00, 1u << 8, 0x0);
  l.bb_m(0x0808, 1u << 28, 0x0);
  l.bb_m(0x0838, (1u << 3) | (1u << 2) | (1u << 1), 0x7);
}

static void tone_afe(TxLog &l) {
  l.bb_w(0x0c60, 0x50000000);
  l.bb_w(0x0c60, 0x70070040);
  l.bb_w(0x0e60, 0x50000000);
  l.bb_w(0x0e60, 0x70070040);
  l.bb_w(0x0c58, 0xd8000402);
  l.bb_w(0x0c5c, 0xd1000120);
  l.bb_w(0x0c6c, 0x00000a15);
  l.bb_w(0x0e58, 0xd8000402);
  l.bb_w(0x0e5c, 0xd1000120);
  l.bb_w(0x0e6c, 0x00000a15);
  tone_bb_reset(l);
  l.bb_m(0x09a4, 1u << 31, 0x0);
}

static void tone_rfe(TxLog &l) {
  l.bb_w(0x0cb0, 0x77777777);
  l.bb_w(0x0cb4, 0x00007777);
  l.bb_m(0x0cbc, 0xfff, 0x100);
  l.bb_w(0x0eb0, 0x77777777);
  l.bb_w(0x0eb4, 0x00007777);
  l.bb_m(0x0ebc, 0xfff, 0x100);
}

static void tone_rf_setting(TxLog &l) {
  l.bb_nr(0x1b00, 0xf8000008);
  l.bb_w(0x1bb8, 0x00000000);
  for (int path = 0; path < 2; path++) {
    uint32_t tmp = l.rf_get(path, 0xdf);
    tmp = (tmp & ~(1u << 4)) | (1u << 1) | (1u << 11);
    rf_set_check(l, path, 0xdf, tmp);
    l.rf_m(path, 0x65, 0xfffff, 0x09000);
    l.rf_m(path, 0xef, 1u << 19, 0x1);
    l.rf_m(path, 0x33, 0xfffff, 0x00026);
    l.rf_m(path, 0x3e, 0xfffff, 0x00037);
    l.rf_m(path, 0x3f, 0xfffff, 0xdefce);
    l.rf_m(path, 0xef, 1u << 19, 0x0);
  }
}

static void tone_txk_setting(TxLog &l, int path) {
  l.bb_nr(0x1b00, 0xf8000008u | (static_cast<uint32_t>(path) << 1));
  l.bb_w(0x1bcc, 0x9);
  l.bb_w(0x1b20, 0x01440008);
  l.bb_nr(0x1b00, path == 0 ? 0xf800000a : 0xf8000008);
  l.bb_w(0x1bcc, 0x3f);
  l.rf_m(path, 0x56, 0xfffff, 0x500ef);
  l.rf_m(path, 0x8f, 0xfffff, 0xa9c00);
  l.bb_m(0x1b2b, 0xff, 0x80);
}

static void tone_full_arm(TxLog &l, int path, int index) {
  tone_configure_macbb(l);
  tone_afe(l);
  tone_rfe(l);
  tone_rf_setting(l);
  tone_txk_setting(l, path);
  l.rf_m(path, 0x00, 0xf0000, 0x2);
  l.rf_m(path, 0x00, 0x1f, 0x0);
  l.rf_m(path, 0x58, 0x2, 0x1);
  const uint32_t txk = 0xf8000418u | (1u << (path + 4));
  l.bb_nr(0x1b00, txk);
  l.bb_nr(0x1b00, txk + 1);
  usleep_us(10000);

  l.bb_nr(0x1b00, 0xf8000008u | (static_cast<uint32_t>(path) << 1));
  l.bb_w(0x1bc8, 0x80000000);
  l.bb_w(0x08f8, 0x41400080);
  l.bb_w(0x1b0c, 0x00003000);
  l.bb_w(0x1b1c, 0xa2193c32);
  l.bb_m(0x1b15, 0xff, 0x00);
  l.bb_w(0x1b20, (static_cast<uint32_t>(index) << 20) | 0x00040008);
  l.bb_w(0x1b24, (static_cast<uint32_t>(index) << 20) | 0x00060008);
  l.bb_w(0x1b2c, 0x07);
  l.bb_w(0x1b38, 0x20000000);
  l.bb_w(0x1b3c, 0x20000000);
  l.bb_w(0x1bcc, 0x0f);
  l.bb_nr(0x1b34, 0x801);
  l.bb_nr(0x1b34, 0x800);
}

// ---- IQK tone-engine setup (8822B _iqk_tximr_selfcheck_8822b) ----
void tone_arm(TxLog &l, int path, int index, double repeat_ms) {
  l.bb_nr(0x1b00, 0xf8000008u | (static_cast<uint32_t>(path) << 1));
  l.bb_w(0x1bc8, 0x80000000);
  l.bb_w(0x08f8, 0x41400080);
  l.bb_w(0x1b0c, 0x00003000);
  l.bb_w(0x1b1c, 0xa2193c32);
  l.bb_m(0x1b15, 0xff, 0x00);
  l.bb_w(0x1b20, (static_cast<uint32_t>(index) << 20) | 0x00040008);
  l.bb_w(0x1b24, (static_cast<uint32_t>(index) << 20) | 0x00060008);
  l.bb_w(0x1b2c, 0x07);
  l.bb_w(0x1b38, 0x20000000);
  l.bb_w(0x1b3c, 0x20000000);
  l.rf_m(path, 0x56, 0xfffff, 0x500ef);
  l.rf_m(path, 0x8f, 0xfffff, 0xa9c00);
  l.bb_m(0x1b2b, 0xff, 0x80);
  l.bb_nr(0x1b00, 0xf8000008u | (static_cast<uint32_t>(path) << 1));
  l.bb_w(0x1bcc, 0x0f);
  l.bb_nr(0x1b34, 0x801);
  l.bb_nr(0x1b34, 0x800);
  (void)repeat_ms;
}

void tone_disarm(TxLog &l) {
  l.bb_nr(0x1b34, 0x0);
  l.bb_w(0x1b2c, 0x0);
  l.bb_w(0x1bcc, 0x0);
  l.restore();
}

// ADDR=MASK:VALUE or ADDR=VALUE
void parse_bb_op(const std::string &s, uint16_t &addr, uint32_t &mask, uint32_t &val) {
  const size_t eq = s.find('=');
  if (eq == std::string::npos)
    throw std::runtime_error("bad --bb op: " + s);
  addr = static_cast<uint16_t>(std::strtoul(s.substr(0, eq).c_str(), nullptr, 0));
  std::string rhs = s.substr(eq + 1);
  const size_t colon = rhs.find(':');
  if (colon == std::string::npos) {
    mask = 0xffffffffu;
    val = static_cast<uint32_t>(std::strtoul(rhs.c_str(), nullptr, 0));
  } else {
    mask = static_cast<uint32_t>(std::strtoul(rhs.substr(0, colon).c_str(), nullptr, 0));
    val = static_cast<uint32_t>(std::strtoul(rhs.substr(colon + 1).c_str(), nullptr, 0));
  }
}

void parse_rf_op(const std::string &s, int &path, uint32_t &addr, uint32_t &mask, uint32_t &val) {
  const size_t eq = s.find('=');
  const size_t c1 = s.find(':');
  if (eq == std::string::npos || c1 == std::string::npos || c1 > eq)
    throw std::runtime_error("bad --rf op (want PATH:ADDR=MASK:VALUE): " + s);
  path = std::stoi(s.substr(0, c1));
  addr = static_cast<uint32_t>(std::strtoul(s.substr(c1 + 1, eq - c1 - 1).c_str(), nullptr, 0));
  std::string rhs = s.substr(eq + 1);
  const size_t colon = rhs.find(':');
  if (colon == std::string::npos) {
    mask = 0xfffffu;
    val = static_cast<uint32_t>(std::strtoul(rhs.c_str(), nullptr, 0));
  } else {
    mask = static_cast<uint32_t>(std::strtoul(rhs.substr(0, colon).c_str(), nullptr, 0));
    val = static_cast<uint32_t>(std::strtoul(rhs.substr(colon + 1).c_str(), nullptr, 0));
  }
}

int find_device(libusb_context *ctx, const OpenOpts &o, libusb_device **out) {
  libusb_device **list = nullptr;
  const ssize_t n = libusb_get_device_list(ctx, &list);
  int rc = LIBUSB_ERROR_NOT_FOUND;
  for (ssize_t i = 0; i < n; i++) {
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
    *out = libusb_ref_device(list[i]);
    rc = 0;
    break;
  }
  if (list)
    libusb_free_device_list(list, 1);
  return rc;
}

int cmd_reset(const Args &a) {
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0)
    return 3;
  libusb_device *dev = nullptr;
  int rc = find_device(ctx, a.open, &dev);
  if (rc != 0) {
    std::fprintf(stderr, "reset: adapter %04x:%04x bus %d port %s not found\n", a.open.vid,
                 a.open.pid, a.open.bus, a.open.port.c_str());
    libusb_exit(ctx);
    return 3;
  }
  libusb_device_handle *h = nullptr;
  rc = libusb_open(dev, &h);
  if (rc == 0) {
    rc = libusb_reset_device(h);
    libusb_close(h);
  }
  libusb_unref_device(dev);
  libusb_exit(ctx);
  std::printf("{\"ev\":\"reset\",\"ok\":%d,\"rc\":%d}\n", rc == 0 ? 1 : 0, rc);
  return rc == 0 ? 0 : 1;
}

int run_tx(Args a) {
  LabCfg cfg;
  std::string err;
  if (!load_lab(a.conf, a.allowed, cfg, err)) {
    std::fprintf(stderr, "lab guard: %s\n", err.c_str());
    return 2;
  }
  if (cfg.channels.count(a.open.channel) == 0) {
    std::fprintf(stderr, "lab guard: channel %d not in allowed list\n", a.open.channel);
    return 2;
  }
  if (a.open.pid != 0xb812) {
    std::fprintf(stderr, "lab guard: refusing pid 0x%04x (TX adapter is 0xb812)\n", a.open.pid);
    return 2;
  }
  if (a.open.bus < 0)
    a.open.bus = cfg.tx_bus;
  if (a.open.port.empty())
    a.open.port = cfg.tx_port;
  if (a.open.bus != cfg.tx_bus || a.open.port != cfg.tx_port) {
    std::fprintf(stderr, "lab guard: requested bus %d port %s != tx_adapter bus %d port %s\n",
                 a.open.bus, a.open.port.c_str(), cfg.tx_bus, cfg.tx_port.c_str());
    return 2;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGALRM, on_signal);

  auto logger = std::make_shared<Logger>();
  Session s(logger);
  try {
    s.open(a.open);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "open/bring-up failed: %s\n", e.what());
    return 3;
  }
  if (a.tx_power >= 0)
    s.radio()->SetTxPower(static_cast<uint8_t>(a.tx_power));
  std::printf("{\"ev\":\"session\",\"chip\":\"%s\",\"superspeed\":%d,\"channel\":%d,"
              "\"tx_bus\":%d,\"tx_port\":\"%s\",\"attenuation_db\":%d,\"tx_paths\":\"%s\"}\n",
              s.chip().c_str(), s.superspeed() ? 1 : 0, a.open.channel, a.open.bus,
              a.open.port.c_str(), cfg.attenuation_db, cfg.tx_paths.c_str());

  std::string logpath = a.log;
  if (logpath.empty())
    logpath = "tx_research/regs/" + utc_stamp() + "-" + a.mode + ".jsonl";
  TxLog reglog(s.reg(), logpath);

  if (!a.ready_file.empty()) {
    std::ofstream rf(a.ready_file, std::ios::trunc);
    rf << now_ns() << "\n";
  }
  if (a.wait_stdin) {
    std::printf("{\"ev\":\"ready\",\"mode\":\"%s\",\"channel\":%d,\"log\":\"%s\"}\n",
                a.mode.c_str(), a.open.channel, logpath.c_str());
    std::fflush(stdout);
    char c = 0;
    const ssize_t got = ::read(STDIN_FILENO, &c, 1);
    if (got != 1) {
      std::fprintf(stderr, "stdin closed before arm; aborting without TX\n");
      return 1;
    }
  }
  if (g_stop) {
    std::fprintf(stderr, "stop requested before arm; aborting without TX\n");
    return 1;
  }
  alarm(static_cast<unsigned>(a.seconds) + 3);

  const uint64_t t0_ns = now_ns();
  bool armed = false;
  int rc = 0;
  try {
    if (a.mode == "cw") {
      cw_arm(reglog, a.gain);
      armed = true;
      std::printf("{\"ev\":\"armed\",\"mode\":\"cw\",\"gain\":%d,\"log\":\"%s\"}\n", a.gain,
                  logpath.c_str());
      hold_seconds(a.seconds);
      cw_disarm(reglog);
      armed = false;
    } else if (a.mode == "cont") {
      cont_arm(reglog);
      armed = true;
      std::printf("{\"ev\":\"armed\",\"mode\":\"cont\",\"log\":\"%s\"}\n", logpath.c_str());
      hold_seconds(a.seconds);
      cont_disarm(reglog);
      armed = false;
    } else if (a.mode == "ook") {
      if (a.ook_src != "lo" && a.ook_src != "gain")
        throw std::runtime_error("--ook-src must be lo or gain");
      cw_arm(reglog, a.gain);
      armed = true;
      const auto period = std::chrono::nanoseconds(
          static_cast<int64_t>(1e9 / a.rate_hz));
      const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                          std::chrono::duration<double>(a.seconds));
      auto next = Clock::now();
      size_t idx = 0;
      uint64_t edges = 0;
      std::printf("{\"ev\":\"armed\",\"mode\":\"ook\",\"pattern\":\"%s\",\"rate_hz\":%.1f,"
                  "\"src\":\"%s\",\"gain\":%d,\"log\":\"%s\"}\n",
                  a.pattern.c_str(), a.rate_hz, a.ook_src.c_str(), a.gain, logpath.c_str());
      while (!g_stop && Clock::now() < end) {
        const bool on = a.pattern[idx % a.pattern.size()] == '1';
        if (a.ook_src == "lo")
          reglog.rf_m(0, 0x58, 0x2, on ? 0x1 : 0x0);
        else
          reglog.rf_m(0, 0x00, 0x1f, on ? static_cast<uint32_t>(a.gain) : 0u);
        idx++;
        edges++;
        next += period;
        const auto now = Clock::now();
        if (next > end)
          break;
        if (next > now)
          std::this_thread::sleep_until(next);
        else if (now - next > std::chrono::milliseconds(100))
          next = now;
      }
      const uint64_t elapsed_ns = now_ns() - t0_ns;
      std::printf("{\"ev\":\"ook.edges\",\"edges\":%llu,\"elapsed_ns\":%llu,\"edges_per_s\":%.1f,"
                  "\"requested_hz\":%.1f}\n",
                  static_cast<unsigned long long>(edges),
                  static_cast<unsigned long long>(elapsed_ns),
                  elapsed_ns ? edges * 1e9 / elapsed_ns : 0.0, a.rate_hz);
      reglog.rf_m(0, 0x58, 0x2, 0x0);
      cw_disarm(reglog);
      armed = false;
    } else if (a.mode == "tone") {
      if (a.tone_full)
        tone_full_arm(reglog, a.tone_path, a.tone_index);
      else
        tone_arm(reglog, a.tone_path, a.tone_index, a.tone_repeat_ms);
      armed = true;
      std::printf("{\"ev\":\"armed\",\"mode\":\"tone\",\"path\":%d,\"index\":%d,"
                  "\"repeat_ms\":%.1f,\"log\":\"%s\"}\n",
                  a.tone_path, a.tone_index, a.tone_repeat_ms, logpath.c_str());
      if (a.tone_repeat_ms > 0) {
        const auto period = std::chrono::duration<double, std::milli>(a.tone_repeat_ms);
        auto next = Clock::now();
        const auto end = next + std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(a.seconds));
        while (!g_stop && Clock::now() < end) {
          reglog.bb_nr(0x1b34, 0x801);
          reglog.bb_nr(0x1b34, 0x800);
          next += std::chrono::duration_cast<Clock::duration>(period);
          const auto now = Clock::now();
          if (next > now)
            std::this_thread::sleep_until(next);
          else if (now - next > std::chrono::milliseconds(100))
            next = now;
        }
      } else {
        hold_seconds(a.seconds);
      }
      tone_disarm(reglog);
      armed = false;
    } else if (a.mode == "fwbench") {
      Cpu8822b cpu(s.reg());
      cpu.load(txbench8822b_bin, txbench8822b_bin_len);
      struct Bench {
        uint32_t magic, reg, value, count, elapsed, done;
      } b{0x54584231u, a.fw_reg, a.fw_value, a.fw_count, 0, 0};
      cpu.dmem_write(0x1000, &b, sizeof b);
      cpu.start();
      const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(a.seconds));
      while (!g_stop && Clock::now() < deadline) {
        cpu.dmem_read(0x1000, &b, sizeof b);
        if (b.done)
          break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      cpu.halt();
      const double rate = b.elapsed ? b.count * 1e6 / b.elapsed : 0.0;
      std::printf("{\"ev\":\"fwbench\",\"reg\":\"0x%x\",\"value\":\"0x%x\","
                  "\"count\":%u,\"elapsed_us\":%u,\"writes_per_s\":%.1f,\"done\":%u}\n",
                  a.fw_reg, a.fw_value, b.count, b.elapsed, rate, b.done);
      armed = false;
    } else if (a.mode == "fw914sweep") {
      Cpu8822b cpu(s.reg());
      cpu.load(tx914sweep8822b_bin, tx914sweep8822b_bin_len);
      reglog.bb_m(0x0838, 0xff, 0x6d);
      reglog.bb_m(0x0800, 1u << 25, 0x1);
      reglog.bb_m(0x0a00, 0x3, 0x0);
      reglog.bb_m(0x0a00, 0x8, 0x1);
      reglog.bb_m(0x0914, 0x70000, 0x4);
      reglog.bb_m(0x0b00, 1u << 8, 0x1);
      reglog.bb_m(0x0b04, 0xf, 0x4);
      cpu.start();
      armed = true;
      std::printf("{\"ev\":\"armed\",\"mode\":\"fw914sweep\",\"seconds\":%.1f}\n",
                  a.seconds);
      hold_seconds(a.seconds);
      cpu.halt();
      uint32_t res[3] = {0, 0, 0};
      cpu.dmem_read(0x1000, res, sizeof res);
      std::printf("{\"ev\":\"fw914sweep.done\",\"magic\":\"0x%x\",\"state\":%u,"
                  "\"sweeps\":%u}\n", res[0], res[1], res[2]);
      armed = false;
    } else if (a.mode == "fwfsk") {
      Cpu8822b cpu(s.reg());
      cpu.load(txfsk8822b_bin, txfsk8822b_bin_len);
      s.tune(a.open.channel, a.open.bw);
      reglog.bb_m(0x0838, 0xff, 0x6d);
      reglog.bb_m(0x0800, 1u << 25, 0x1);
      reglog.bb_m(0x0a00, 0x3, 0x0);
      reglog.bb_m(0x0a00, 0x8, 0x1);
      reglog.bb_m(0x0914, 0x70000, 0x4);
      reglog.bb_m(0x0910, 0xfff, a.fsk_f1);
      reglog.bb_m(0x0b00, 1u << 8, 0x1);
      reglog.bb_m(0x0b04, 0xf, 0x4);
      struct Fsk {
        uint32_t magic, f1, f2, dwell, transitions, reserved, done, count, ampword;
      } f{0x46534b31u, a.fsk_f1, a.fsk_f2, a.fsk_dwell_us, a.fsk_cycles, 0,
          0, 0, a.fsk_amp * 0x01010101u};
      cpu.dmem_write(0x1000, &f, sizeof f);
      cpu.start();
      armed = true;
      std::printf("{\"ev\":\"armed\",\"mode\":\"fwfsk\",\"f1\":%u,\"f2\":%u,"
                  "\"dwell_us\":%u,\"amp\":%u}\n",
                  a.fsk_f1, a.fsk_f2, a.fsk_dwell_us, a.fsk_amp);
      hold_seconds(a.seconds);
      cpu.halt();
      cpu.dmem_read(0x1000, &f, sizeof f);
      std::printf("{\"ev\":\"fwfsk.done\",\"magic\":\"0x%x\",\"transitions\":%u,"
                  "\"done\":%u,\"bb910\":\"0x%x\"}\n", f.magic, f.count, f.done,
                  s.reg().r32(0x0910));
      armed = false;
    } else if (a.mode == "fsk") {
      reglog.bb_m(0x0838, 0xff, 0x6d);
      reglog.bb_m(0x0800, 1u << 25, 0x1);
      reglog.bb_m(0x0a00, 0x3, 0x0);
      reglog.bb_m(0x0a00, 0x8, 0x1);
      reglog.bb_m(0x0914, 0x70000, 0x4);
      reglog.bb_m(0x0910, 0xfff, a.fsk_f1);
      reglog.bb_m(0x0b00, 1u << 8, 0x1);
      reglog.bb_m(0x0b04, 0xf, 0x4);
      armed = true;
      const auto period = std::chrono::nanoseconds(
          static_cast<int64_t>(1e9 / a.rate_hz));
      const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                          std::chrono::duration<double>(a.seconds));
      auto next = Clock::now();
      size_t idx = 0;
      uint64_t edges = 0;
      std::printf("{\"ev\":\"armed\",\"mode\":\"fsk\",\"f1\":%u,\"f2\":%u,"
                  "\"rate_hz\":%.1f}\n", a.fsk_f1, a.fsk_f2, a.rate_hz);
      while (!g_stop && Clock::now() < end) {
        const uint32_t f = (idx & 1) ? a.fsk_f2 : a.fsk_f1;
        reglog.bb_m(0x0914, 0x70000, 0x0);
        reglog.bb_m(0x0910, 0xfff, f);
        reglog.bb_m(0x0914, 0x70000, 0x4);
        reglog.bb_m(0x0b04, 0xf, 0x4);
        idx++;
        edges++;
        next += period;
        const auto now = Clock::now();
        if (next > end)
          break;
        if (next > now)
          std::this_thread::sleep_until(next);
        else if (now - next > std::chrono::milliseconds(100))
          next = now;
      }
      const uint64_t elapsed_ns = now_ns() - t0_ns;
      std::printf("{\"ev\":\"fsk.edges\",\"edges\":%llu,\"elapsed_ns\":%llu,"
                  "\"edges_per_s\":%.1f}\n",
                  static_cast<unsigned long long>(edges),
                  static_cast<unsigned long long>(elapsed_ns),
                  elapsed_ns ? edges * 1e9 / elapsed_ns : 0.0);
      reglog.bb_m(0x0914, 0x70000, 0x0);
      reglog.bb_m(0x0b04, 0xf, 0x2);
      reglog.bb_m(0x0b00, 1u << 8, 0x0);
      reglog.bb_m(0x0100, 0x100, 0x0);
      reglog.bb_m(0x0100, 0x100, 0x1);
      reglog.restore();
      armed = false;
    } else if (a.mode == "raw") {
      for (const auto &op : a.bb_ops) {
        uint16_t addr;
        uint32_t mask, val;
        parse_bb_op(op, addr, mask, val);
        if (mask == 0xffffffffu)
          reglog.bb_w(addr, val);
        else
          reglog.bb_m(addr, mask, val);
      }
      for (const auto &op : a.rf_ops) {
        int path;
        uint32_t addr, mask, val;
        parse_rf_op(op, path, addr, mask, val);
        reglog.rf_m(path, addr, mask, val);
      }
      armed = true;
      std::printf("{\"ev\":\"armed\",\"mode\":\"raw\",\"bb_ops\":%zu,\"rf_ops\":%zu,"
                  "\"log\":\"%s\"}\n",
                  a.bb_ops.size(), a.rf_ops.size(), logpath.c_str());
      hold_seconds(a.seconds);
      reglog.restore();
      armed = false;
    } else {
      std::fprintf(stderr, "unknown TX mode %s\n", a.mode.c_str());
      return 2;
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "%s: %s\n", a.mode.c_str(), e.what());
    rc = 1;
  }
  if (armed) {
    try {
      reglog.rf_m(0, 0x58, 0x2, 0x0);
      reglog.bb_m(0x0914, 0x70000, 0x0);
      reglog.bb_m(0x0808, (1u << 29) | (1u << 28), 0x3);
      reglog.restore();
    } catch (const std::exception &e) {
      std::fprintf(stderr, "emergency stop failed: %s\n", e.what());
      rc = 1;
    }
  }
  // The TX probe always leaves the analog LO and the 0x914/PMAC test paths
  // off, even when the pre-run state had them on. This is the recovery rule.
  try {
    reglog.rf_m(0, 0x58, 0x2, 0x0);
    reglog.rf_m(1, 0x58, 0x2, 0x0);
    reglog.bb_m(0x0b04, 0xf, 0x2);
    reglog.bb_m(0x0914, 0x70000, 0x0);
    reglog.bb_m(0x0b00, 0x100, 0x0);
    reglog.bb_m(0x0100, 0x100, 0x0);
    reglog.bb_m(0x0100, 0x100, 0x1);
    reglog.bb_m(0x0808, (1u << 29) | (1u << 28), 0x3);
    reglog.bb_w(0x1d00, 0x0);
    reglog.bb_w(0x1d80, 0x0);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "final TX-off writes failed: %s\n", e.what());
    rc = 1;
  }
  const uint64_t elapsed_ns = now_ns() - t0_ns;
  const bool lo_off = (reglog.rf_get(0, 0x58) & 0x2) == 0;
  std::printf("{\"ev\":\"txprobe.done\",\"mode\":\"%s\",\"ok\":%d,\"writes\":%llu,"
              "\"writes_per_s\":%.1f,\"elapsed_s\":%.3f,\"lo_off\":%d,\"transfers\":%llu,"
              "\"alive\":%d,\"log\":\"%s\"}\n",
              a.mode.c_str(), rc == 0 && lo_off ? 1 : 0,
              static_cast<unsigned long long>(reglog.writes()),
              elapsed_ns ? reglog.writes() * 1e9 / elapsed_ns : 0.0, elapsed_ns / 1e9,
              lo_off ? 1 : 0, s.reg().transfers(), s.reg().alive() ? 1 : 0, logpath.c_str());
  return rc == 0 && lo_off ? 0 : 1;
}

} // namespace

int main(int argc, char **argv) {
  Args a;
  if (!parse(argc, argv, a)) {
    std::fprintf(stderr,
                 "usage: wsdr_txprobe cw|cont|ook|raw|reset --pid 0xb812 [--bus 7] [--port 1]\n"
                 "       [--channel 36] [--seconds 3] [--gain N] [--log FILE] [--conf FILE]\n"
                 "       ook: [--pattern 1010] [--rate-hz 1000] [--ook-src lo|gain]\n"
                 "       raw: [--bb ADDR=MASK:VALUE]... [--rf PATH:ADDR=MASK:VALUE]...\n"
                 "       tone: [--tone-index N] [--tone-path 0|1] [--tone-repeat-ms MS]"
                 " [--tone-full]\n"
                 "       [--tx-power 0..63] [--fw-reg R --fw-value V --fw-count N]\n"
                 "       fw914sweep: 0x914 low-word sweep firmware\n"
                 "       fwfsk: --fsk-f1 N --fsk-f2 N --fsk-dwell-us N --fsk-amp N\n"
                 "       fsk: --fsk-f1 N --fsk-f2 N --rate-hz N (host-timed)\n");
    return 2;
  }
  if (a.mode == "reset")
    return cmd_reset(a);
  if (a.mode == "cw" || a.mode == "cont" || a.mode == "ook" || a.mode == "tone" ||
      a.mode == "raw" || a.mode == "fwbench" || a.mode == "fw914sweep" ||
      a.mode == "fwfsk" || a.mode == "fsk")
    return run_tx(a);
  std::fprintf(stderr, "unknown mode %s\n", a.mode.c_str());
  return 2;
}
