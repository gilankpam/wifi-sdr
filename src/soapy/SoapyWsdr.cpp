// SPDX-License-Identifier: GPL-2.0-only
// SoapySDR driver "wsdr": the RTL8812BU (RTL8822B) and RTL8812CU (RTL8822C) LA streams as SDR receivers.
//
// Build and use (no system install needed):
//   cmake -S . -B build-soapy -DWSDR_SOAPY=ON && cmake --build build-soapy -j --target wsdrSupport
//   export SOAPY_SDR_PLUGIN_PATH=$PWD/build-soapy
//   SoapySDRUtil --find=driver=wsdr
//   gqrx: device string  soapy=0,driver=wsdr      SDR++: SoapySDR source, device "wsdr"
//   adapter: pid=0xb812 (8812BU, the default when several are plugged in), pid=0xc812 (8812CU) or pid=0xa81a (8812EU);
//            bus=N [port=P] picks one of several (as SoapySDRUtil --find lists them)
// The client opens the adapter itself, so it needs USB access: install tools/70-wsdr.rules (udev) or
// run the client as root. Keep the rtw88 guard ON so the kernel driver does not claim the adapter.
//
// What the client sees:
//   frequency    a double, so 5 GHz needs no tricks; snapped to the 5 MHz synthesizer grid
//                (2.4 GHz ch1..14, 5 GHz ch16..233), the residual (<= 2.5 MHz) shifted digitally
//   sample rate  native LA rates, no resampling: 2.5 / 5 MS/s on USB 2; on USB 3 also 10 / 20 / 40 MS/s.
//                At 40 MS/s the radio runs at 40 MHz channel width with the vendor 40 MHz RX DFIR, so
//                the whole +-20 MHz is real (bandwidth 40 MHz; setBandwidth <= 25 MHz forces the 20 MHz
//                path). Below 40 MS/s it stays at 20 MHz width (bandwidth min(rate, 18.4 MHz)):
//                devourer's BW20 DFIR is what limits the band (FINDINGS §4, notes/wide-tap.md)
//   8812CU       USB 2 only here: 2.5 / 5 MS/s, 20 MHz path.
//   channels     two RX channels on both chips: 0 = path A, 1 = path B, from the same LA word (the
//                8812BU streams LA dma_type 4 for that). B leads A, not compensated: ~146 ns on the
//                8812BU, ~370-400 ns on the 8812CU. On the 8812BU coherent at 40 MS/s and at 20 MS/s
//                (20 MHz width, as here); its lower rates select samples without a filter, so there
//                the two channels are separate receivers
//   gain         one element "IGI", 0..49 dB in ~0.95 dB steps (8812CU: 0..52 dB, ~1.0 dB; fixed-gain
//                mode, OFDM CCA off);
//                gain mode "automatic" hands gain to the chip's AGC (gain steps during frames)
//   format       CS16 (native: 12-bit << 4) or CF32 (/2048); DC removed
//                unless DC offset mode is turned off
// Every setting change restarts the stream (tens of ms; ~150 ms for a retune), so the chip is never
// touched while it streams. USB 3 works on a good host controller (here xHCI 67:00.3; 65:00.3 wedges
// C2H, FINDINGS §4); a wedged stream is logged and restarted.
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <libusb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "iqdsp.h"
#include "logger.h"
#include "receiver.h"
#include "session.h"
#include "stream88xx.h"
#include "stream_cu.h"

using namespace wsdr;
using clk = std::chrono::steady_clock;

namespace {

// Measured dB per IGI step: 8822B 0.95, 8822C ~1.0 (FINDINGS §4). Range IGI 0x1C..0x50.
constexpr double kGainStepB = 0.95, kGainStepC = 1.0;
double gain_step(bool cu) { return cu ? kGainStepC : kGainStepB; }
double gain_max(bool cu) { return (0x50 - 0x1C) * gain_step(cu); }

struct Settings {
  double freq = 5180e6;
  int rate_sel = 5; // fs = 80 MHz >> rate_sel: 5 = 2.5, 4 = 5, 3 = 10, 2 = 20, 1 = 40 MS/s
  bool agc = false;
  double gain_db = 30;
  bool dc_remove = true;
  bool wide = true; // at 40 MS/s: 40 MHz channel width + vendor BW40 DFIR (setBandwidth <= 25 MHz clears)
};

// Channel width in use: 40 MHz only at 40 MS/s (a 40 MHz band would alias at lower rates).
int width_of(const Settings &s) { return s.rate_sel == 1 && s.wide ? 40 : 20; }

double rate_of(int rate_sel) { return 80e6 / (1 << rate_sel); }
constexpr double kAnalogBw = 18.4e6; // measured -3 dB width of the LA tap (+-9.2 MHz), any channel width

int igi_for(double db, bool cu) {
  return std::clamp(0x50 - static_cast<int>(std::lround(db / gain_step(cu))), 0x1C, 0x50);
}

// Single-producer / single-consumer sample ring (complex float, LA units). Overflow drops the
// incoming block and is reported once to the reader.
class Ring {
public:
  void reset(size_t cap) {
    std::lock_guard<std::mutex> l(_m);
    _buf.assign(cap, cf(0));
    _head = _tail = _count = 0;
    _ovf = false;
  }
  void push(const cf *x, size_t n) {
    {
      std::lock_guard<std::mutex> l(_m);
      if (n > _buf.size() - _count) {
        _ovf = true;
        return;
      }
      const size_t first = std::min(n, _buf.size() - _head);
      std::memcpy(&_buf[_head], x, first * sizeof(cf));
      std::memcpy(&_buf[0], x + first, (n - first) * sizeof(cf));
      _head = (_head + n) % _buf.size();
      _count += n;
    }
    _cv.notify_one();
  }
  // >0 samples copied, 0 timeout, -1 overflow pending (cleared), -2 woken by stop.
  int pop(cf *out, size_t n, long timeout_us, const std::atomic<bool> &active) {
    std::unique_lock<std::mutex> l(_m);
    _cv.wait_for(l, std::chrono::microseconds(std::max(0L, timeout_us)),
                 [&] { return _count > 0 || _ovf || !active; });
    if (_ovf) {
      _ovf = false;
      return -1;
    }
    if (!_count)
      return active ? 0 : -2;
    const size_t k = std::min(n, _count);
    const size_t first = std::min(k, _buf.size() - _tail);
    std::memcpy(out, &_buf[_tail], first * sizeof(cf));
    std::memcpy(out + first, &_buf[0], (k - first) * sizeof(cf));
    _tail = (_tail + k) % _buf.size();
    _count -= k;
    return static_cast<int>(k);
  }
  void wake() { _cv.notify_all(); }

private:
  std::mutex _m;
  std::condition_variable _cv;
  std::vector<cf> _buf;
  size_t _head = 0, _tail = 0, _count = 0;
  bool _ovf = false;
};

struct WsdrStream {
  std::string format;
  std::vector<size_t> channels; // 0 = path A, 1 = path B
};

SoapySDR::KwargsList find_wsdr(const SoapySDR::Kwargs &args);

class SoapyWsdr : public SoapySDR::Device {
public:
  explicit SoapyWsdr(const SoapySDR::Kwargs &args) {
    if (args.count("freq"))
      _set.freq = std::stod(args.at("freq"));
    OpenOpts o;
    o.pid = kPid8812bu;
    if (args.count("pid")) {
      o.pid = static_cast<uint16_t>(std::stoul(args.at("pid"), nullptr, 0));
    } else {
      const auto found = find_wsdr(args); // 8812BU entries first
      if (!found.empty())
        o.pid = static_cast<uint16_t>(std::stoul(found.front().at("pid"), nullptr, 0));
    }
    if (args.count("bus")) { // pick one of several identical adapters (as find_wsdr lists them)
      const std::string &b = args.at("bus");
      char *end = nullptr;
      const long v = std::strtol(b.c_str(), &end, 10);
      if (b.empty() || *end || v < 0)
        throw std::runtime_error("wsdr: bad bus=" + b);
      o.bus = static_cast<int>(v);
    }
    if (args.count("port"))
      o.port = args.at("port");
    _cu = jaguar3_pid(o.pid);
    _eu = o.pid == kPid8812eu;
    o.channel = Receiver::channel_for(_set.freq, 20); // (opens at 20 MHz: the centre is the primary)
    o.thermal_track = false; // no host register traffic while streaming
    _s = std::make_unique<Session>(std::make_shared<Logger>());
    _s->open(o); // throws on failure; SoapySDR reports it to the client
    if (_cu)
      _s->reg().set_cap(4); // the 8822C hangs on >= 4 KiB EP0 block transfers (notes/cu-custom-code.md)
    _channel = o.channel;
    SoapySDR::logf(SOAPY_SDR_INFO, "wsdr: %s on USB %d, sample rates up to %.1f MS/s", _s->chip().c_str(),
                   _s->superspeed() ? 3 : 2, rate_of(min_rate_sel()) / 1e6);
  }

  ~SoapyWsdr() override { stop_worker(); }

  // ---- identification ----
  std::string getDriverKey() const override { return "wsdr"; }
  std::string getHardwareKey() const override { return _s->chip(); } // RTL8822B / RTL8822C / RTL8822E
  SoapySDR::Kwargs getHardwareInfo() const override {
    return {{"chip", _s->chip()}, {"usb", _s->superspeed() ? "3" : "2"},
            {"path", std::string("A (ch 0), B (ch 1); B leads A by ") + (_cu ? "~370-400 ns" : "~146 ns")},
            {"source", "LA ring, C2H bank streamer"}};
  }

  // ---- channels ----
  size_t getNumChannels(const int dir) const override { return dir == SOAPY_SDR_RX ? nch() : 0; }
  bool getFullDuplex(const int, const size_t) const override { return false; }

  // ---- streams ----
  std::vector<std::string> getStreamFormats(const int dir, const size_t) const override {
    if (dir != SOAPY_SDR_RX)
      return {};
    return {SOAPY_SDR_CS16, SOAPY_SDR_CF32};
  }
  std::string getNativeStreamFormat(const int, const size_t, double &fullScale) const override {
    fullScale = 32768;
    return SOAPY_SDR_CS16;
  }
  SoapySDR::Stream *setupStream(const int dir, const std::string &format, const std::vector<size_t> &channels,
                                const SoapySDR::Kwargs &) override {
    if (dir != SOAPY_SDR_RX)
      throw std::runtime_error("wsdr: RX only");
    if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32)
      throw std::runtime_error("wsdr: format must be CS16 or CF32");
    std::vector<size_t> chans = channels.empty() ? std::vector<size_t>{0} : channels;
    for (size_t i = 0; i < chans.size(); i++)
      if (chans[i] >= nch() || std::count(chans.begin(), chans.end(), chans[i]) > 1)
        throw std::runtime_error("wsdr: channels 0 (path A) and/or 1 (path B)");
    std::lock_guard<std::mutex> l(_streamm);
    if (_stream)
      throw std::runtime_error("wsdr: stream already open");
    _stream = std::make_unique<WsdrStream>();
    _stream->format = format;
    _stream->channels = chans;
    return reinterpret_cast<SoapySDR::Stream *>(_stream.get());
  }
  void closeStream(SoapySDR::Stream *) override {
    stop_worker();
    std::lock_guard<std::mutex> l(_streamm);
    _stream.reset();
  }
  size_t getStreamMTU(SoapySDR::Stream *) const override { return Stream8822b::kBankSamples; }
  int activateStream(SoapySDR::Stream *, const int flags, const long long, const size_t) override {
    if (flags)
      return SOAPY_SDR_NOT_SUPPORTED;
    if (_active)
      return 0;
    _chans = reinterpret_cast<WsdrStream *>(_stream.get())->channels;
    _ring.reset(_chans.size() * static_cast<size_t>(rate_of(min_rate_sel()) / 4)); // 0.25 s at the fastest rate
    _active = true;
    _restart = false;
    _worker = std::thread([this] { worker(); });
    return 0;
  }
  int deactivateStream(SoapySDR::Stream *, const int flags, const long long) override {
    if (flags)
      return SOAPY_SDR_NOT_SUPPORTED;
    stop_worker();
    return 0;
  }
  int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs,
                 const long timeoutUs) override {
    flags = 0;
    timeNs = 0;
    auto *st = reinterpret_cast<WsdrStream *>(stream);
    const size_t w = st->channels.size(); // the ring holds frames of w samples (one per channel)
    if (_tmp.size() < numElems * w)
      _tmp.resize(numElems * w);
    const int got = _ring.pop(_tmp.data(), numElems * w, timeoutUs, _active);
    if (got == -1)
      return SOAPY_SDR_OVERFLOW;
    if (got == -2)
      return SOAPY_SDR_STREAM_ERROR;
    if (got == 0)
      return SOAPY_SDR_TIMEOUT;
    const int n = got / static_cast<int>(w);
    for (size_t k = 0; k < w; k++) {
      if (st->format == SOAPY_SDR_CS16) {
        auto *o = static_cast<int16_t *>(buffs[k]);
        for (int i = 0; i < n; i++) {
          const cf v = _tmp[i * w + k];
          o[2 * i] = static_cast<int16_t>(std::clamp(std::lrint(v.real() * 16.f), -32768L, 32767L));
          o[2 * i + 1] = static_cast<int16_t>(std::clamp(std::lrint(v.imag() * 16.f), -32768L, 32767L));
        }
      } else {
        auto *o = static_cast<float *>(buffs[k]);
        for (int i = 0; i < n; i++) {
          o[2 * i] = _tmp[i * w + k].real() / 2048.f;
          o[2 * i + 1] = _tmp[i * w + k].imag() / 2048.f;
        }
      }
    }
    return n;
  }

  // ---- antenna ----
  std::vector<std::string> listAntennas(const int dir, const size_t) const override {
    return dir == SOAPY_SDR_RX ? std::vector<std::string>{"RX"} : std::vector<std::string>{};
  }
  void setAntenna(const int, const size_t, const std::string &) override {}
  std::string getAntenna(const int, const size_t) const override { return "RX"; }

  // ---- DC offset (host-side DC removal) ----
  bool hasDCOffsetMode(const int dir, const size_t) const override { return dir == SOAPY_SDR_RX; }
  void setDCOffsetMode(const int, const size_t, const bool automatic) override {
    update([&](Settings &s) { s.dc_remove = automatic; });
  }
  bool getDCOffsetMode(const int, const size_t) const override { return get().dc_remove; }

  // ---- gain ----
  std::vector<std::string> listGains(const int dir, const size_t) const override {
    return dir == SOAPY_SDR_RX ? std::vector<std::string>{"IGI"} : std::vector<std::string>{};
  }
  bool hasGainMode(const int dir, const size_t) const override { return dir == SOAPY_SDR_RX; }
  void setGainMode(const int, const size_t, const bool automatic) override {
    update([&](Settings &s) { s.agc = automatic; });
  }
  bool getGainMode(const int, const size_t) const override { return get().agc; }
  void setGain(const int, const size_t, const double value) override {
    update([&](Settings &s) { s.gain_db = std::clamp(value, 0.0, gain_max(_cu)); });
  }
  void setGain(const int dir, const size_t ch, const std::string &, const double value) override {
    setGain(dir, ch, value);
  }
  double getGain(const int, const size_t) const override { return get().gain_db; }
  double getGain(const int dir, const size_t ch, const std::string &) const override { return getGain(dir, ch); }
  SoapySDR::Range getGainRange(const int, const size_t) const override {
    return {0, gain_max(_cu), gain_step(_cu)};
  }
  SoapySDR::Range getGainRange(const int dir, const size_t ch, const std::string &) const override {
    return getGainRange(dir, ch);
  }

  // ---- frequency ----
  std::vector<std::string> listFrequencies(const int dir, const size_t) const override {
    return dir == SOAPY_SDR_RX ? std::vector<std::string>{"RF"} : std::vector<std::string>{};
  }
  void setFrequency(const int, const size_t, const double frequency, const SoapySDR::Kwargs &) override {
    update([&](Settings &s) { s.freq = frequency; });
  }
  void setFrequency(const int dir, const size_t ch, const std::string &, const double frequency,
                    const SoapySDR::Kwargs &args) override {
    setFrequency(dir, ch, frequency, args);
  }
  double getFrequency(const int, const size_t) const override { return get().freq; }
  double getFrequency(const int dir, const size_t ch, const std::string &) const override {
    return getFrequency(dir, ch);
  }
  SoapySDR::RangeList getFrequencyRange(const int, const size_t) const override {
    if (_eu) // no 2.4 GHz front end on the 8812EU board (it tunes there, but streams the ADC floor)
      return {SoapySDR::Range(5.080e9, 6.165e9)};
    return {SoapySDR::Range(2.400e9, 2.500e9), SoapySDR::Range(5.080e9, 6.165e9)};
  }
  SoapySDR::RangeList getFrequencyRange(const int dir, const size_t ch, const std::string &) const override {
    return getFrequencyRange(dir, ch);
  }

  // ---- sample rate / bandwidth ----
  void setSampleRate(const int, const size_t, const double rate) override {
    int rs = 5;
    for (int r = 5; r >= min_rate_sel(); r--)
      if (std::abs(rate - rate_of(r)) < std::abs(rate - rate_of(rs)))
        rs = r;
    if (std::abs(rate - rate_of(rs)) > 1)
      SoapySDR::logf(SOAPY_SDR_WARNING, "wsdr: %.0f S/s not supported, using %.0f", rate, rate_of(rs));
    update([&](Settings &s) { s.rate_sel = rs; });
  }
  double getSampleRate(const int, const size_t) const override { return rate_of(get().rate_sel); }
  std::vector<double> listSampleRates(const int, const size_t) const override {
    std::vector<double> v;
    for (int r = 5; r >= min_rate_sel(); r--)
      v.push_back(rate_of(r));
    return v;
  }
  SoapySDR::RangeList getSampleRateRange(const int, const size_t) const override {
    SoapySDR::RangeList v;
    for (int r = 5; r >= min_rate_sel(); r--)
      v.emplace_back(rate_of(r), rate_of(r));
    return v;
  }
  void setBandwidth(const int, const size_t, const double bw) override {
    update([&](Settings &s) { s.wide = bw <= 0 || bw > 25e6; });
  }
  double getBandwidth(const int dir, const size_t ch) const override {
    return width_of(get()) == 40 ? 40e6 : std::min(getSampleRate(dir, ch), kAnalogBw);
  }
  std::vector<double> listBandwidths(const int dir, const size_t ch) const override {
    if (get().rate_sel == 1)
      return {kAnalogBw, 40e6};
    return {getBandwidth(dir, ch)};
  }

private:
  // USB 2 carries ~44 MB/s of C2H (5 MS/s at 8 B/sample); USB 3 carries 40 MS/s (320 MB/s). The
  // 8812CU streamer is verified on USB 2 only.
  int min_rate_sel() const { return _s->superspeed() && !_cu ? 1 : 4; }
  size_t nch() const { return 2; } // paths A and B (8812BU: LA dma_type 4)

  Settings get() const {
    std::lock_guard<std::mutex> l(_setm);
    return _set;
  }
  // Setters only record the change; the worker applies it between stream runs.
  template <class F> void update(F f) {
    {
      std::lock_guard<std::mutex> l(_setm);
      f(_set);
      _last_change = clk::now();
    }
    _restart = true;
  }

  void stop_worker() {
    _active = false;
    _restart = true;
    _ring.wake();
    if (_worker.joinable())
      _worker.join();
  }

  // Owns the device while active: (re)configure, stream until a setting changes or a fault, repeat.
  void worker() {
    constexpr size_t kN = Stream8822b::kBankSamples;
    static_assert(kN == Stream8822c::kBankSamples);
    const std::vector<size_t> chans = _chans;
    const size_t w = chans.size();
    std::vector<std::vector<cf>> x(w, std::vector<cf>(kN));
    std::vector<cf> frames(kN * w);
    while (_active) {
      // Let a burst of setter calls settle before restarting.
      for (;;) {
        clk::time_point last;
        {
          std::lock_guard<std::mutex> l(_setm);
          last = _last_change;
        }
        if (!_active || clk::now() - last >= std::chrono::milliseconds(30))
          break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (!_active)
        break;
      _restart = false;
      const Settings cur = get();
      const int width = width_of(cur);
      const int ch = Receiver::channel_for(cur.freq, width); // the centre channel (Receiver's mapping)
      const double src = rate_of(cur.rate_sel);
      const double centre = Receiver::centre_of(ch);
      const double shift = cur.freq - centre;
      if (std::abs(shift) > 0.05 * src)
        SoapySDR::logf(SOAPY_SDR_WARNING,
                       "wsdr: %.3f MHz is %.0f kHz off the 5 MHz grid; the band edge on that side is aliased",
                       cur.freq / 1e6, shift / 1e3);
      try {
        if (ch != _channel || width != _width) {
          const Receiver::Primary p = Receiver::primary_for(ch, width);
          _s->tune(p.channel, width, p.offset);
          _channel = ch;
          _width = width;
        }
      } catch (const std::exception &e) {
        _channel = 0; // devourer may have half-programmed the new channel: tune again next time (as Receiver)
        SoapySDR::logf(SOAPY_SDR_ERROR, "wsdr: retune to channel %d failed: %s", ch, e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      std::vector<DcBlock> dc(w);
      std::vector<Nco> nco(w);
      for (auto &n : nco)
        n.set(shift, src);
      StreamCfg c;
      c.la.rate_sel = cur.rate_sel;
      // >= 20 MS/s needs ~13 ms of posted URBs; the 8812CU wants ~100 ms (FINDINGS §4)
      c.depth = _cu || cur.rate_sel <= 2 ? 256 : 64;
      c.la.igi = cur.agc ? -1 : igi_for(cur.gain_db, _cu);
      c.la.no_cca = !cur.agc;
      c.la.dfir = width == 40 ? 40 : 0; // 0 leaves devourer's BW20 DFIR (the 20 MHz path); 8822B only
      if (!_cu)
        c.la.dma_type = kDmaBothPaths8822b; // paths A and B in one word
      const auto on_bank = [&](const uint8_t *bank, const BankInfo &) {
        for (size_t i = 0; i < kN; i++) {
          uint64_t v;
          std::memcpy(&v, bank + 8 * i, 8);
          for (size_t k = 0; k < w; k++)
            x[k][i] = chans[k] == 0
                          ? cf(static_cast<float>(la_i(v)), static_cast<float>(la_q(v)))
                          : cf(static_cast<float>(path_b_i(v, _cu)), static_cast<float>(path_b_q(v, _cu)));
        }
        for (size_t k = 0; k < w; k++) {
          if (cur.dc_remove)
            dc[k].process(x[k].data(), kN);
          nco[k].process(x[k].data(), kN);
        }
        if (w == 1) {
          _ring.push(x[0].data(), kN);
        } else {
          for (size_t i = 0; i < kN; i++)
            for (size_t k = 0; k < w; k++)
              frames[i * w + k] = x[k][i];
          _ring.push(frames.data(), frames.size());
        }
        return _active.load();
      };
      const StreamStats ss = _cu ? Stream8822c(*_s, c).run(on_bank, &_restart)
                                 : Stream8822b(*_s, c).run(on_bank, &_restart);
      if (!ss.error.empty() && !_restart && _active) {
        SoapySDR::logf(SOAPY_SDR_WARNING, "wsdr: stream fault after %llu banks (%s); restarting",
                       static_cast<unsigned long long>(ss.banks), ss.error.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  std::unique_ptr<Session> _s;
  bool _cu = false;              // RTL8812CU / RTL8812EU (Jaguar3)
  bool _eu = false;              // RTL8812EU: 5 GHz only
  std::vector<size_t> _chans{0}; // channels of the active stream
  int _channel = 36;
  int _width = 20;
  mutable std::mutex _setm;
  Settings _set;
  clk::time_point _last_change{};
  std::mutex _streamm;
  std::unique_ptr<WsdrStream> _stream;
  std::atomic<bool> _active{false}, _restart{false};
  std::thread _worker;
  Ring _ring;
  std::vector<cf> _tmp;
};

// Enumerate without opening: only USB descriptors, bus and port numbers.
SoapySDR::KwargsList find_wsdr(const SoapySDR::Kwargs &args) {
  SoapySDR::KwargsList out;
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) != 0)
    return out;
  libusb_device **list = nullptr;
  const ssize_t n = libusb_get_device_list(ctx, &list);
  for (ssize_t i = 0; i < n; i++) {
    libusb_device_descriptor d{};
    if (libusb_get_device_descriptor(list[i], &d) != 0)
      continue;
    const uint16_t rpid = realtek_pid(d.idVendor, d.idProduct); // an OEM alias lists as its Realtek pid
    if (rpid != kPid8812bu && !jaguar3_pid(rpid))
      continue;
    char pid[8];
    std::snprintf(pid, sizeof pid, "0x%04x", rpid);
    if (args.count("pid") && std::stoul(args.at("pid"), nullptr, 0) != rpid)
      continue;
    uint8_t ports[8];
    const int np = libusb_get_port_numbers(list[i], ports, sizeof ports);
    std::string port;
    for (int k = 0; k < np; k++)
      port += (k ? "." : "") + std::to_string(ports[k]);
    const std::string bus = std::to_string(libusb_get_bus_number(list[i]));
    const bool ss = libusb_get_device_speed(list[i]) >= LIBUSB_SPEED_SUPER;
    if (args.count("bus") && args.at("bus") != bus)
      continue;
    if (args.count("port") && args.at("port") != port)
      continue;
    SoapySDR::Kwargs k;
    k["driver"] = "wsdr";
    k["label"] = std::string(rpid == kPid8812cu ? "RTL8812CU (RTL8822C)" : rpid == kPid8812eu ? "RTL8812EU (RTL8822E)" : "RTL8812BU (RTL8822B)") +
                 " :: bus " + bus + " port " + port +
                 (ss ? " (USB 3)" : " (USB 2)");
    k["pid"] = pid;
    k["bus"] = bus;
    k["port"] = port;
    k["usb"] = ss ? "3" : "2";
    out.push_back(k);
  }
  if (list)
    libusb_free_device_list(list, 1);
  libusb_exit(ctx);
  // 8812BU first (the default adapter), then 8812CU, then 8812EU: by rank, not by the pid string
  const auto rank = [](const SoapySDR::Kwargs &k) { return k.at("pid") == "0xb812" ? 0 : k.at("pid") == "0xc812" ? 1 : 2; };
  std::stable_sort(out.begin(), out.end(), [&](const auto &a, const auto &b) { return rank(a) < rank(b); });
  return out;
}

SoapySDR::Device *make_wsdr(const SoapySDR::Kwargs &args) { return new SoapyWsdr(args); }

const SoapySDR::Registry kRegister("wsdr", &find_wsdr, &make_wsdr, SOAPY_SDR_ABI_VERSION);

} // namespace
