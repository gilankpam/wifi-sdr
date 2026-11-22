// SPDX-License-Identifier: GPL-2.0-only
#include "receiver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <tuple>

#include "ChannelFreq.h"
#include "stream88xx.h"
#include "stream_cu.h"

namespace wsdr {

using clk = std::chrono::steady_clock;

const char *rx_state_name(RxState s) {
  switch (s) {
  case RxState::Closed: return "closed";
  case RxState::Opening: return "opening";
  case RxState::Starting: return "starting";
  case RxState::Streaming: return "streaming";
  case RxState::Lost: return "lost";
  case RxState::Error: return "error";
  }
  return "?";
}

// The RF is tuned to the centre channel. devourer takes the primary 20 MHz channel and where the
// centre sits from it (ChannelOffset): at 40 MHz 1 = +2 channels (+10 MHz) and 2 = -2; at 80 MHz
// 1..4 = +6/+2/-2/-6 (+30 MHz measured from primaries 1, 3, 36, 98, 100; wide-tap.md). Centres above
// the primary are used wherever the primary stays a channel; at the bottom of 2.4 GHz a wide span still
// centres on channel 1 (primary above it), so 40/80 MHz spans reach 2392/2372 MHz there, and 2504/2524
// MHz centred on channel 14. 5 GHz centres keep the primaries 16..233 (5080-6165 MHz).
int Receiver::channel_for(double freq, int width) {
  const double mhz = freq / 1e6;
  if (mhz < 3700) {
    const int g = std::clamp(static_cast<int>(std::lround((mhz - 2407) / 5)), 1, 13);
    return std::abs(mhz - 2484) < std::abs(mhz - devourer::chan_to_freq(g)) ? 14 : g;
  }
  const int lo = width == 80 ? 22 : width == 40 ? 18 : 16, hi = width == 80 ? 227 : width == 40 ? 231 : 233;
  return std::clamp(static_cast<int>(std::lround((mhz - 5000) / 5)), lo, hi);
}

// Stream runs: each Receiver counts its own up by one (a consumer that keeps only the low byte, as
// wsdr-studio's rows do, always sees the next run change it), starting above every number any Receiver
// of the process has used, so a close and reopen never repeats the last run's number (a consumer that
// resets on a new gen, like a spectrum, would keep the old run's frequency range).
static std::atomic<uint32_t> g_gen_max{0};
static void note_gen(uint32_t g) {
  uint32_t m = g_gen_max.load();
  while (m < g && !g_gen_max.compare_exchange_weak(m, g)) {
  }
}

double Receiver::centre_of(int centre) { return devourer::chan_to_freq(centre) * 1e6; }

Receiver::Primary Receiver::primary_for(int centre, int width) {
  if (width <= 20)
    return {centre, 0};
  const bool g2 = centre <= 14;
  if (width == 40)
    return !g2 || centre >= 3 ? Primary{centre - 2, 1} : Primary{centre + 2, 2};
  // (80 MHz: the 8822B only; devourer's 8822C/E path ignores the offset there, always +6, and
  // width_of keeps those at 40 MHz)
  if (!g2 || centre >= 7)
    return {centre - 6, 1};
  return centre >= 3 ? Primary{centre - 2, 2} : Primary{centre + 2, 3};
}

int Receiver::igi_for(double db, bool cu) {
  return std::clamp(0x50 - static_cast<int>(std::lround(db / gain_step(cu))), 0x1C, 0x50);
}

Receiver::Receiver(Logger_t log) : _log(std::move(log)) {}

Receiver::~Receiver() { close(); }

void Receiver::set_state(RxState s, const std::string &err) {
  std::lock_guard<std::mutex> l(_m);
  _st.state = s;
  if (!err.empty())
    _st.error = err;
}

void Receiver::open(uint16_t pid, const RxSettings &initial, int bus, const std::string &port, int sys_fd) {
  close();
  if (initial.rate_sel < 0 || initial.rate_sel > 5)
    throw std::invalid_argument("rate_sel must be 0..5");
  {
    std::lock_guard<std::mutex> l(_m);
    _set = initial;
    _st = {};
    _st.state = RxState::Opening;
  }
  const bool cu = jaguar3_pid(pid); // 8812CU and 8812EU: the 8822C path throughout
  OpenOpts o;
  o.pid = pid;
  o.bus = bus;
  o.port = port;
  o.sys_fd = sys_fd; // (Session::open then skips the USB reset)
  o.bw = cu && !initial.allow_gapped ? 20 : width_of(initial, cu); // 8812CU at 40 MS/s only gapped
  const int centre = channel_for(initial.freq, o.bw);
  const Primary p = primary_for(centre, o.bw);
  o.channel = p.channel;
  o.offset = p.offset;
  o.thermal_track = false; // no host register traffic while streaming
  auto s = std::make_unique<Session>(_log);
  try {
    s->open(o);
  } catch (...) {
    set_state(RxState::Closed);
    throw;
  }
  if (cu)
    s->reg().set_cap(4); // the 8822C hangs on >= 4 KiB EP0 block transfers (notes/cu-custom-code.md)
  _oo = o;
  if (o.sys_fd < 0) { // reopen() finds this very device again, whatever bus/port (or none) was asked for
    libusb_device *dev = libusb_get_device(s->handle());
    uint8_t ports[8];
    const int np = libusb_get_port_numbers(dev, ports, sizeof ports);
    _oo.bus = libusb_get_bus_number(dev);
    _oo.port.clear();
    for (int k = 0; k < np; k++)
      _oo.port += (k ? "." : "") + std::to_string(ports[k]);
  }
  _caps = {};
  _caps.pid = pid;
  _caps.cu = cu;
  _caps.superspeed = s->superspeed();
  _caps.chip = s->chip();
  // USB 2 carries ~44 MB/s of C2H (5 MS/s); USB 3 carries 40 MS/s. The 8812CU and 8812EU are USB 2 only.
  _caps.min_rate_sel = s->superspeed() && !cu ? 1 : 4;
  // Both streamers can also capture one bank at any rate, stop, ship and re-arm (gapped mode; on the
  // 8812CU measured at 10/20/40 MS/s: zero stale banks, ~36/22/12 % of the time sampled). An 8812BU on
  // USB 3 streams every rate up to 40 MS/s and runs 80 MS/s gapped (33 % sampled), on USB 2 too (6 %;
  // both measured, evidence/gapped-80msps.txt).
  _caps.gapped_min_rate_sel = cu ? 1 : 0;
  _caps.paths = 2; // the 8822B streams kDmaBothPaths8822b (path B costs no extra USB bandwidth)
  _caps.gain_step = gain_step(cu);
  _caps.gain_max = gain_max(cu);
  _hw_channel = centre;
  _hw_width = o.bw;
  _gen = g_gen_max.load(); // (the first run is numbered above every earlier one)
  _s = std::move(s);
  _open = true;
  std::lock_guard<std::mutex> l(_m);
  _sweep.clear();
  _sweep_step = 0;
  _sweep_pass = 0;
  _set.rate_sel = std::max(_set.rate_sel, lowest_rate_sel(_set));
  _st.state = RxState::Closed;
  _st.channel = _hw_channel;
  _st.width = _hw_width;
}

void Receiver::close() {
  stop();
  _open = false;
  _s.reset();
  std::lock_guard<std::mutex> l(_m);
  _st.state = RxState::Closed;
}

void Receiver::start(BankSink sink) {
  if (!_s)
    throw std::logic_error("Receiver::start before open");
  if (_worker.joinable()) {
    {
      std::lock_guard<std::mutex> l(_m);
      if (_st.running)
        return;
    }
    _worker.join(); // it ended itself (Lost / Error)
  }
  _sink = std::move(sink);
  {
    std::lock_guard<std::mutex> l(_m);
    _st.running = true;
  }
  _active = true;
  _restart = false;
  _worker = std::thread([this] { worker(); });
}

void Receiver::stop() {
  _active = false;
  _restart = true;
  if (_worker.joinable())
    _worker.join();
  std::lock_guard<std::mutex> l(_m);
  if (_st.state != RxState::Lost)
    _st.state = RxState::Closed;
}

void Receiver::set(const RxSettings &s) {
  {
    std::lock_guard<std::mutex> l(_m);
    RxSettings n = s;
    n.rate_sel = std::clamp(n.rate_sel, lowest_rate_sel(n), 5);
    n.gain_db = std::clamp(n.gain_db, 0.0, _caps.gain_max);
    const auto hw = [&](const RxSettings &x) {
      const int w = width_of(x, _caps.cu);
      return std::make_tuple(channel_for(x.freq, w), w, x.rate_sel, x.agc, x.agc ? 0 : igi_for(x.gain_db, _caps.cu));
    };
    const bool same_hw = hw(n) == hw(_set);
    _set = n;
    if (same_hw)
      return; // nothing for the chip to do: keep streaming
    _last_change = clk::now();
    _cfg++;
  }
  _restart = true;
}

RxSettings Receiver::settings() const {
  std::lock_guard<std::mutex> l(_m);
  return _set;
}

RxStatus Receiver::status() const {
  std::lock_guard<std::mutex> l(_m);
  RxStatus s = _st;
  s.settling = _cfg != _st.cfg && _st.state != RxState::Closed;
  return s;
}

void Receiver::sweep(const std::vector<double> &centres, double dwell_s, double gap_s) {
  {
    std::lock_guard<std::mutex> l(_m);
    if (centres.empty() && _sweep.empty())
      return; // not sweeping: nothing to stop
    _sweep = centres;
    _dwell_s = dwell_s;
    _gap_s = gap_s;
    _sweep_step = 0;
    _sweep_pass = 0;
    _sweep_id++;
    _last_change = clk::now();
    _cfg++;
  }
  _restart = true;
}

bool Receiver::sweeping() const {
  std::lock_guard<std::mutex> l(_m);
  return !_sweep.empty();
}

void Receiver::reopen() {
  OpenOpts o = _oo;
  o.bw = _hw_width;
  const Primary p = primary_for(_hw_channel, _hw_width);
  o.channel = p.channel;
  o.offset = p.offset;
  _s.reset(); // the interface can be claimed once: release it before the new bring-up claims it
  auto s = std::make_unique<Session>(_log);
  s->open(o);
  if (_caps.cu)
    s->reg().set_cap(4);
  if (s->superspeed() != _caps.superspeed) // its rates would no longer be the ones the caller was given
    throw std::runtime_error(std::string("the adapter came back at USB ") + (s->superspeed() ? "3" : "2"));
  _s = std::move(s);
}

void Receiver::worker() {
  struct Done { // the worker ends (stop, Lost, Error): say so in the status
    Receiver *r;
    ~Done() {
      std::lock_guard<std::mutex> l(r->_m);
      r->_st.running = false;
    }
  } done{this};
  uint64_t seq = 0;
  int consecutive_faults = 0;
  while (_active) {
    // Let a burst of setter calls settle before restarting.
    for (;;) {
      clk::time_point last;
      {
        std::lock_guard<std::mutex> l(_m);
        last = _last_change;
      }
      if (!_active || clk::now() - last >= std::chrono::milliseconds(30))
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!_active)
      break;
    _restart = false;
    RxSettings cur;
    uint32_t gen, cfg;
    int step = -1; // sweep step of this run
    uint32_t pass = 0, sweep_id = 0;
    double dwell_s = 0, gap_s = 0;
    {
      std::lock_guard<std::mutex> l(_m);
      cur = _set;
      if (!_sweep.empty()) {
        step = static_cast<int>(_sweep_step % _sweep.size());
        cur.freq = _sweep[step];
        pass = _sweep_pass;
        sweep_id = _sweep_id;
        dwell_s = _dwell_s;
        gap_s = _gap_s;
      }
      gen = ++_gen;
      note_gen(gen);
      cfg = _cfg;
      _st.state = RxState::Starting;
      _st.step = step;
      _st.pass = pass;
    }
    const int width = width_of(cur, _caps.cu);
    const int ch = channel_for(cur.freq, width);
    const double fs = rate_of(cur.rate_sel);
    const double centre = centre_of(ch);
    try {
      if (ch != _hw_channel || width != _hw_width) {
        const Primary p = primary_for(ch, width);
        _s->tune(p.channel, width, p.offset);
        _hw_channel = ch;
        _hw_width = width;
      }
    } catch (const std::exception &e) {
      _hw_channel = 0; // devourer may have half-programmed the new channel: tune again next time
      if (!_s->reg().alive()) {
        set_state(RxState::Lost, std::string("adapter lost during retune: ") + e.what());
        break;
      }
      if (++consecutive_faults >= 5) { // a retune that never succeeds is fatal, like a stream that never runs
        set_state(RxState::Error, std::string("retune to channel ") + std::to_string(ch) + " keeps failing: " + e.what());
        break;
      }
      set_state(RxState::Error, std::string("retune to channel ") + std::to_string(ch) + " failed: " + e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    StreamCfg c;
    c.la.rate_sel = cur.rate_sel;
    // >= 20 MS/s needs ~13 ms of posted URBs; the 8812CU wants ~100 ms (FINDINGS §4)
    c.depth = _caps.cu || cur.rate_sel <= 2 ? 256 : 64;
    c.la.igi = cur.agc ? -1 : igi_for(cur.gain_db, _caps.cu);
    c.la.no_cca = !cur.agc;
    c.la.dfir = !_caps.cu && width >= 40 ? width : 0; // 0 leaves devourer's DFIR (8822B: BW20 set; 8822C: per width)
    if (!_caps.cu)
      c.la.dma_type = kDmaBothPaths8822b;
    c.gapped = cur.rate_sel < _caps.min_rate_sel;
    BankMeta meta;
    meta.gen = gen;
    meta.cfg = cfg;
    meta.fs = fs;
    meta.centre_hz = centre;
    meta.shift_hz = cur.freq - centre;
    meta.paths = _caps.paths;
    meta.cu = _caps.cu;
    meta.gapped = c.gapped;
    meta.step = step;
    meta.pass = pass;
    {
      std::lock_guard<std::mutex> l(_m);
      _st.channel = ch;
      _st.width = width;
      _st.centre_hz = centre;
      _st.fs = fs;
      _st.gen = gen;
      _st.cfg = cfg;
      _st.gapped = c.gapped;
      _st.state = RxState::Streaming;
    }
    bool clean_start = false;
    // A sweep step ends once dwell_s of samples has been delivered (gapped: of wall time, as its banks
    // are separate captures).
    const double dwell_banks = std::ceil(dwell_s * fs / (Stream8822b::kBankBytes / 8.0));
    uint64_t run_banks = 0;
    clk::time_point run_t0{};
    bool step_done = false;
    const auto on_bank = [&](const uint8_t *bank, const BankInfo &) {
      if (!clean_start) {
        clean_start = true;
        consecutive_faults = 0;
        std::lock_guard<std::mutex> l(_m);
        _st.error.clear();
      }
      meta.seq = seq++;
      {
        std::lock_guard<std::mutex> l(_m);
        _st.banks++;
        // A frequency change inside the same step needs no restart (set() leaves _cfg alone), so the
        // shift follows _set; after a hardware change (_cfg moved on) this run keeps its own frequency.
        meta.shift_hz = (step < 0 && _cfg == cfg ? _set.freq : cur.freq) - centre;
      }
      _sink(bank, Stream8822b::kBankBytes, meta);
      if (step >= 0) {
        if (run_banks++ == 0)
          run_t0 = clk::now();
        step_done = c.gapped ? clk::now() - run_t0 >= std::chrono::duration<double>(dwell_s)
                             : static_cast<double>(run_banks) >= dwell_banks;
        if (step_done)
          return false; // next step
      }
      return _active.load();
    };
    StreamStats ss;
    try {
      ss = _caps.cu ? Stream8822c(*_s, c).run(on_bank, &_restart) : Stream8822b(*_s, c).run(on_bank, &_restart);
    } catch (const std::exception &e) {
      ss.error = e.what();
    }
    {
      std::lock_guard<std::mutex> l(_m);
      _st.start_retries += ss.start_retries;
      if (step_done && _sweep_id == sweep_id && !_sweep.empty() && ++_sweep_step >= _sweep.size()) {
        _sweep_step = 0;
        _sweep_pass++;
      }
    }
    if (step_done && gap_s > 0 && !_restart && _active)
      std::this_thread::sleep_for(std::chrono::duration<double>(gap_s));
    if (step_done || ss.error.empty() || _restart || !_active)
      continue; // next sweep step, settings change or stop
    // The count and the reason change together: a status() read between them would show a new
    // restart with the old (or no) text, and the reason is the only record of why.
    RxState next = RxState::Starting;
    std::string why = "stream fault (restarting): " + ss.error;
    if (!_s->reg().alive()) {
      next = RxState::Lost;
      why = "adapter lost: " + ss.error;
    } else if (++consecutive_faults >= 5) {
      next = RxState::Error;
      why = "stream keeps failing: " + ss.error;
    }
    {
      std::lock_guard<std::mutex> l(_m);
      _st.restarts++;
      _st.state = next;
      _st.error = why;
      _st.last_fault = why;
    }
    if (next != RxState::Starting)
      break;
    // MEASURED (now-studio's 5 GHz sweep, 8812BU at 40 MS/s, 2026-11-22): now and then the RX DMA
    // locks mid-stream. The CPU program still arms and the LA runs, but no packet reaches bulk-IN, and
    // no restart (CPU reload + LA configure) brings it back; a full bring-up does. So the 2nd and 4th
    // consecutive fault re-open the adapter in place (not with a granted fd: it cannot be re-opened).
    if ((consecutive_faults == 2 || consecutive_faults == 4) && _oo.sys_fd < 0) {
      std::fprintf(stderr, "receiver: %d stream faults in a row, re-opening the adapter (%s)\n",
                   consecutive_faults, ss.error.c_str());
      set_state(RxState::Opening);
      try {
        reopen();
        _hw_channel = 0; // (reopen() tuned it; tune again anyway, with the run's own settings)
        std::lock_guard<std::mutex> l(_m);
        _st.reopens++;
      } catch (const OpenError &e) {
        set_state(e.kind == OpenError::NotFound ? RxState::Lost : RxState::Error,
                  std::string("re-open after stream faults failed: ") + e.what());
        break;
      } catch (const std::exception &e) {
        set_state(RxState::Error, std::string("re-open after stream faults failed: ") + e.what());
        break;
      }
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} // namespace wsdr
