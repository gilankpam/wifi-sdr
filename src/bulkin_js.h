// SPDX-License-Identifier: GPL-2.0-only
// InOrderReader for WebAssembly builds that define WSDR_JS_BULKIN (bulkin88xx.h picks it): the stream's bulk-IN
// reads are kept queued from JavaScript instead of through libusb.
//
// libusb's Emscripten backend runs every transfer through the module's main thread: the stream thread's submit
// is proxied there and its completion comes back through libusb's event wait. That round trip per 16 KiB packet
// left the chip without a queued read often enough to miss its bank deadline. MEASURED (kepyur, Chrome 147,
// Linux, 8812BU at USB 3, 20 MS/s): libusb's reader 90.8 % of the banks and 11 restarts in 20 s; this reader
// 100 % and 0 restarts in 20 s, 0 restarts in 30 s of video decoding.
//
// The host page's module supplies two functions, called on the module's main thread:
//   Module.wsdrBulkStart(ctl, done, len, status, buf, ring, depth, slot_bytes)
//       keep `depth` transferIn(4, slot_bytes) in flight on the opened device; write each completion, in
//       completion order, to ring slot k = n++ % ring: data at buf + k * slot_bytes, HEAP32[len/4 + k] = bytes,
//       HEAP32[status/4 + k] = 0 ok / 1 error, then Atomics.store + Atomics.notify HEAP32[done/4 + k] = 1, and
//       submit the next transferIn. A transferIn that fails flags its slot with status 1 and stops.
//   Module.wsdrBulkStop(ctl)
//       ignore what is in flight from now on, abort it (WebUSB cannot cancel: release + claim interface 0),
//       wait until none is pending, then Atomics.store + Atomics.notify HEAP32[ctl/4] = 1.
// This side only waits on the slots' atomics: no per-transfer call crosses threads.
#pragma once

#include <emscripten.h>
#include <emscripten/atomic.h>
#include <emscripten/threading.h>

#include <chrono>
#include <cstdint>
#include <vector>

namespace wsdr {

class InOrderReader {
public:
  static constexpr int kLen = 16384;
  InOrderReader(libusb_device_handle *, libusb_context *, int depth)
      : _depth(depth < 64 ? 64 : depth), _ring(_depth * 2), _done(_ring), _len(_ring), _status(_ring),
        _buf(static_cast<size_t>(_ring) * kLen) {}
  ~InOrderReader() { stop(); }

  void start() {
    stop();
    for (auto &d : _done)
      d = 0;
    _head = 0;
    _ctl[0] = 0;
    _running = true;
    MAIN_THREAD_ASYNC_EM_ASM({ Module.wsdrBulkStart($0, $1, $2, $3, $4, $5, $6, $7); }, _ctl, _done.data(),
                             _len.data(), _status.data(), _buf.data(), _ring, _depth, kLen);
  }

  struct Done {
    const uint8_t *data;
    int len;
    int status;
  };
  // Next completed transfer (head of the ring); false on timeout.
  bool next(Done &d, unsigned timeout_ms) {
    const int i = static_cast<int>(_head % static_cast<uint64_t>(_ring));
    const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (__atomic_load_n(&_done[i], __ATOMIC_ACQUIRE) == 0) {
      const auto left = dl - std::chrono::steady_clock::now();
      if (left <= std::chrono::nanoseconds(0))
        return false;
      emscripten_atomic_wait_u32(&_done[i], 0, std::chrono::duration_cast<std::chrono::nanoseconds>(left).count());
    }
    d = {_buf.data() + static_cast<size_t>(i) * kLen, _len[i], _status[i]};
    return true;
  }
  void release() {
    __atomic_store_n(&_done[_head % static_cast<uint64_t>(_ring)], 0, __ATOMIC_RELEASE);
    _head++;
  }
  void drain(unsigned ms) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    Done d;
    while (std::chrono::steady_clock::now() < end)
      if (next(d, 1))
        release();
  }
  void stop() {
    if (!_running)
      return;
    _running = false;
    MAIN_THREAD_ASYNC_EM_ASM({ Module.wsdrBulkStop($0); }, _ctl);
    const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    while (__atomic_load_n(&_ctl[0], __ATOMIC_ACQUIRE) == 0 && std::chrono::steady_clock::now() < dl)
      emscripten_atomic_wait_u32(&_ctl[0], 0, 5'000'000);
  }

private:
  int _depth, _ring;
  std::vector<uint32_t> _done;
  std::vector<int32_t> _len, _status;
  std::vector<uint8_t> _buf;
  alignas(8) uint32_t _ctl[2] = {1, 0}; // [0]: JavaScript idle
  uint64_t _head = 0;
  bool _running = false;
};

} // namespace wsdr
