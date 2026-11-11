// SPDX-License-Identifier: GPL-2.0-only
// Bulk-IN EP 0x84 reader for the C2H streamers: a ring of async transfers reaped strictly in
// submission order (one transfer = one C2H packet), so packets come back in the order the chip sent them.
#pragma once

#include <libusb.h>

#include <chrono>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

namespace wsdr {

// A cancelled transfer that did not reap in time: give it, and the buffer the kernel may still
// copy into, to a callback that frees both once libusb reaps it, so its owner can go away now.
// libusb reads callback/user_data only at completion, on the thread that handles events.
inline void orphan_transfer(libusb_transfer *x, std::vector<uint8_t> &&buf) {
  struct Orphan {
    std::vector<uint8_t> buf; // a moved vector keeps its storage: the address the kernel has
    static void LIBUSB_CALL done(libusb_transfer *t) {
      delete static_cast<Orphan *>(t->user_data);
      libusb_free_transfer(t);
    }
  };
  x->user_data = new Orphan{std::move(buf)};
  x->callback = Orphan::done;
}

#if defined(__EMSCRIPTEN__) && defined(WSDR_JS_BULKIN)
} // namespace wsdr
#include "bulkin_js.h" // the web build's reader: WebUSB transfers kept queued from JavaScript
namespace wsdr {
#else
class InOrderReader {
public:
  InOrderReader(libusb_device_handle *h, libusb_context *ctx, int depth) : _h(h), _ctx(ctx) {
    _slots.resize(static_cast<size_t>(depth));
  }
  ~InOrderReader() { stop(); }

  void start() {
    for (size_t i = 0; i < _slots.size(); i++) {
      Slot &s = _slots[i];
      s.buf.resize(16384);
      s.x = libusb_alloc_transfer(0);
      if (!s.x)
        throw std::runtime_error("libusb_alloc_transfer");
      libusb_fill_bulk_transfer(s.x, _h, 0x84, s.buf.data(), static_cast<int>(s.buf.size()), cb, &s, 0);
      submit(i);
    }
  }
  // Next completed transfer (head of the queue); false on timeout.
  struct Done {
    const uint8_t *data;
    int len;
    int status;
  };
  bool next(Done &d, unsigned timeout_ms) {
    const size_t i = _fifo.front();
    const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (_slots[i].busy) {
      // past the deadline, reap once more before giving up (the thread may have slept through it)
      const bool late = std::chrono::steady_clock::now() >= dl;
      timeval tv{0, late ? 0 : 2000};
      const int rc = libusb_handle_events_timeout_completed(_ctx, &tv, nullptr);
      if (rc < 0 && rc != LIBUSB_ERROR_INTERRUPTED)
        throw std::runtime_error(std::string("libusb events: ") + libusb_error_name(rc));
      if (late && _slots[i].busy)
        return false;
    }
    d = {_slots[i].buf.data(), _slots[i].x->actual_length, _slots[i].x->status};
    return true;
  }
  void release() { // resubmit the head transfer at the tail
    const size_t i = _fifo.front();
    _fifo.pop_front();
    submit(i);
  }
  void drain(unsigned ms) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    Done d;
    while (std::chrono::steady_clock::now() < end)
      if (next(d, 1))
        release();
  }
  void stop() {
    for (auto &s : _slots)
      if (s.x && s.busy)
        libusb_cancel_transfer(s.x);
    for (int n = 0; n < 500; n++) {
      bool any = false;
      for (auto &s : _slots)
        any |= s.x && s.busy;
      if (!any)
        break;
      timeval tv{0, 2000};
      libusb_handle_events_timeout_completed(_ctx, &tv, nullptr);
    }
    for (auto &s : _slots) {
      if (!s.x)
        continue;
      if (s.busy)
        orphan_transfer(s.x, std::move(s.buf));
      else
        libusb_free_transfer(s.x);
      s.x = nullptr;
      s.busy = false;
    }
    _fifo.clear();
  }

private:
  struct Slot {
    libusb_transfer *x = nullptr;
    std::vector<uint8_t> buf;
    bool busy = false;
  };
  static void LIBUSB_CALL cb(libusb_transfer *x) { static_cast<Slot *>(x->user_data)->busy = false; }
  void submit(size_t i) {
    _slots[i].busy = true;
    if (const int rc = libusb_submit_transfer(_slots[i].x)) {
      _slots[i].busy = false;
      throw std::runtime_error(std::string("bulk-IN submit: ") + libusb_error_name(rc));
    }
    _fifo.push_back(i);
  }

  libusb_device_handle *_h;
  libusb_context *_ctx;
  std::vector<Slot> _slots;
  std::deque<size_t> _fifo;
};
#endif
} // namespace wsdr
