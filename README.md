# wifi-sdr

Realtek 88x2 USB Wi-Fi adapters as raw-IQ receivers.

The RTL8822B and RTL8822C baseband has a logic-analyser (LA) tap that writes 12-bit ADC samples into a
ring in the chip's TX FIFO. wifi-sdr keeps that ring free-running and loads a small MIPS16e2 program onto
the chip's embedded CPU (RAM only). The program ships every 64 KiB ring bank to the host over the USB
bulk-IN (C2H) path. On the host, the banks are reassembled, checked for gaps and staleness, and
offered as an SDR stream. Chip bring-up (firmware, RF, tuning) comes from
[devourer](https://github.com/OpenIPC/devourer), a userspace libusb driver.

Status: **experimental, receive only.** Every number here was measured on two bench units on one host.
The evidence is under [`evidence/`](evidence/), and the details are in [FINDINGS.md](FINDINGS.md).

## What works

| Adapter | Chip | Link | Continuous IQ | Gapped (spectrum only) | Paths |
|---|---|---|---|---|---|
| RTL8812BU `0bda:b812` | RTL8822B (Jaguar2) | USB 2 | 2.5, 5 MS/s | up to 80 MS/s | A + B² |
| RTL8812BU `0bda:b812` | RTL8822B (Jaguar2) | USB 3 | 2.5 – 40 MS/s | 80 MS/s | A + B |
| RTL8812CU `0bda:c812` | RTL8822C (Jaguar3) | USB 2 only¹ | 2.5, 5 MS/s | up to 40 MS/s | A + B, coherent |
| RTL8812EU `0bda:a81a` | RTL8822E (Jaguar3) | USB 2 | 2.5, 5 MS/s | up to 40 MS/s | A + B |

¹ The efuse on this 8812CU board disables USB 3.
The 8812EU runs the 8822C code unchanged (same firmware entry stub, LA ring, C2H map and streamer; 20 s
runs at 5 MS/s with zero faults, `evidence/phase6/`). Its board has no 2.4 GHz front end: the chip tunes
there, but streams the ADC floor. Use it on 5 GHz.
² Same chip setting as on USB 3; the two-path measurements were made on USB 3.

OEM-branded 8812BU: the **TP-Link Archer T3U Plus** (`2357:0138`) and **Archer T4U v3** (`2357:0115`, tested in
[kepyur](https://github.com/gilankpam/kepyur)) are RTL8812BUs under TP-Link's vendor id and work as the 8812BU rows
above; the **Archer T3U** (`2357:012d`) carries the same chip but is untested. Select them as `--pid 0xb812` (the
alias table in `src/session.h`), not by the id that `lsusb` shows; each needs its own line in `tools/70-wsdr.rules`.

Not supported: the **RTL8812AU** (`0bda:8812`, RTL8812A, Jaguar1) and the RTL8821AU have no LA block
(`0x7c0` never latches the arm bits; devourer reports `no_la_block`, `external/devourer/docs/la-capture.md`),
and their on-chip CPU is not the MIPS core the streamer is written for. They still work under devourer as
packet receivers or as a `txdemo` test source. Of the Jaguar1 parts only the RTL8814AU has an LA block
(64 KB TXFF window); it is not ported. The MT7612U has no raw-IQ path either (`notes/mt7612u-sdr.md`).

- **Continuous** means every sample with zero faults over 60 s runs. A separate offline check
  (`tools/stream_check.py`) confirms this: beacons sit on the 102.4 ms grid, with no slips or duplicate
  banks.
- **Gapped** means each 8192-sample bank is a complete capture, with ~1.5 ms gaps between banks. FFT
  within a bank for wide-span spectrum and waterfall views. Do not use it for continuous recording.
- **Bandwidth:** ~18.4 MHz (−3 dB) at 20 MHz channel width. At 40 MS/s the radio switches to 40 MHz
  width (on the 8822B, also the vendor BW40 RX DFIR), so the full ±20 MHz is real. On the 8822B, 80 MS/s
  (~77 MHz) exceeds the ~391 MB/s C2H ceiling, so it runs gapped only (below).
- **Gain:** fixed-gain mode with OFDM carrier sense off and IGI pinned gives ~0.95 dB per step over
  ~45 dB on the 8822B and ~1.0 dB per step on the 8822C. Alternatively, the chip's own AGC can run.
  On the 8822B a gain change takes effect while the stream runs (about 6 banks, no restart); on the
  8822C it restarts the stream (~150 ms).
- **Tuning:** 2.4 GHz ch 1–14 and 5 GHz ch 16–233 on the 5 MHz synthesizer grid, including the ch 65–99
  gap between the UNII-2 groups (see FINDINGS). The residual (≤ 2.5 MHz) is shifted out digitally.
- **Two paths on both adapters:** paths A and B come from the same LA word (the 8812BU streams LA
  dma_type 4 for that, at no extra USB cost). B leads A by ~146 ns on the 8812BU and ~370–400 ns on
  the 8812CU, which the stream does not compensate. On the 8812BU the paths are coherent at 40 MS/s,
  at 20 MS/s at 20 MHz width, and gapped at 40 and 80 MS/s; its lower rates select samples without a
  filter, so there A and B are two separate receivers.
- **80 MS/s on the 8812BU, gapped:** ~77 MHz (−6 dB) per bank at 80 MHz width. About a third of the
  time is captured over USB 3 (each 102.4 µs bank, then ~0.2 ms to ship it and re-arm), 6 % over USB 2
  (~1.5 ms). Too fast to stream; for wide spectrum views and
  sweeps (`RxSettings{.rate_sel = 0, .allow_gapped = true}`; `evidence/gapped-80msps.txt`).
- **USB 3 depends on the host port:** one of this host's xHCI controllers (AMD `1022:15b9`) wedges
  large C2H packets. Another (`1022:15c0`) runs flawlessly. The 8812BU comes up at USB 2 after a cold
  plug. `wsdr::request_usb3()` switches it to USB 3, as rtw88 does at probe; `request_usb2()` switches it
  back without a replug (`build/wsdr_usbmode --bus N --port P --to 2|3`).

TX is out of scope. The chips have no arbitrary-IQ transmit path (see `notes/tx-feasibility.md`).

## Front ends

- **SoapySDR module `wsdr`**: works with gqrx, SDR++ and GNU Radio, for both adapters. It uses true
  5 GHz frequencies and outputs CS16 or CF32 at native LA rates with no resampling. It exposes two
  channels (paths A and B) on both adapters.
- **`wsdr_tcp`**: an rtl_tcp server for the 8812BU on USB 2. It resamples to any client rate.
- **Recorders**: `wsdr_probe stream --out` (8812BU) and `wsdr_cu stream --out` (8812CU) write
  `PREFIX.cs16` (int16 I,Q, 12-bit << 4) plus `PREFIX.json`.
- **`wsdrcore` library / `wsdr::Receiver`** ([src/receiver.h](src/receiver.h)): the app-facing API. It
  opens, tunes, streams, restarts on faults and hands each bank to a sink.
  [wsdr-studio](https://github.com/snokvist/wsdr-studio), the desktop app, consumes it as a CMake
  subproject.

## Building

Requirements:
- CMake ≥ 3.21 and a C++20 compiler.
- libusb-1.0 headers.
- `binutils-mipsel-linux-gnu` and `xxd`, which assemble the on-chip programs.
- Optional: `libsoapysdr-dev` for the SoapySDR module.
- Optional: python3 with numpy and matplotlib for `tools/`.

```sh
git submodule update --init external/devourer   # pinned devourer; not --recursive (its submodules are reference trees)
cmake -S . -B build && cmake --build build -j
```

This builds `wsdr_probe`, `wsdr_cu`, `wsdr_tcp`, `wsdr_rxtest`, `wsdr_usbmode` and `iqdsp_test`. To use a devourer
tree other than the submodule, pass `-DDEVOURER_DIR=/path/to/devourer`.

The SoapySDR module is opt-in, because it rebuilds the core as position-independent code:

```sh
cmake -S . -B build-soapy -DWSDR_SOAPY=ON && cmake --build build-soapy -j --target wsdrSupport
SOAPY_SDR_PLUGIN_PATH=$PWD/build-soapy SoapySDRUtil --find=driver=wsdr
```

## Running

**1. Keep the kernel drivers off the adapters.** rtw88 probes these dongles on every enumeration and
loads its own firmware. The guard adds a temporary modprobe blacklist and unloads the modules. It
lists only the per-chip USB modules, so the host's own Wi-Fi is unaffected:

```sh
sudo tools/rtw88_guard.sh on      # then replug the adapter
sudo tools/rtw88_guard.sh off     # restore the normal drivers when done
```

**2. USB access.** Either run the tools as root, or install the udev rule so that SoapySDR clients
work without root:

```sh
sudo cp tools/70-wsdr.rules /etc/udev/rules.d/ && sudo rm -f /etc/udev/rules.d/99-wsdr.rules && sudo udevadm control --reload   # then replug
```

(`99-wsdr.rules` is the rule's old name: numbered 99 it ran after `73-seat-late.rules`, where `uaccess`
is turned into an ACL, so only `plugdev` members could open the adapters.)

**3. Pick a front end.**

```sh
# gqrx / SDR++ via SoapySDR (pid=0xc812 selects the 8812CU, pid=0xa81a the 8812EU; the 8812BU is the default)
SOAPY_SDR_PLUGIN_PATH=$PWD/build-soapy gqrx          # device string: soapy=0,driver=wsdr

# rtl_tcp: rtl_tcp sends frequency as a u32, so use an offset and set gqrx's LNB LO to 5000 MHz
sudo build/wsdr_tcp --pid 0xb812 --freq-offset 5e9   # gqrx device: rtl_tcp=127.0.0.1:1234, tune 5180 MHz

# Record 60 s and check continuity offline
sudo build/wsdr_probe stream --pid 0xb812 --rate 5 --seconds 60 --out captures/s5m
python3 tools/stream_check.py captures/s5m

# 8812BU: both paths at 40 MS/s (written from a separate thread; then the two-path checks)
sudo build/wsdr_probe stream --pid 0xb812 --channel 36 --bw 40 --rate 1 --dfir 40 --dma 4 --path ab --seconds 3 --out captures/ab
python3 tools/bu_twopath.py captures/ab; python3 tools/bu_ab_delay.py captures/ab

# 8812CU (8812EU: --pid 0xa81a): both paths, or gapped 40 MS/s at 40 MHz width
sudo build/wsdr_cu stream --pid 0xc812 --rate 4 --seconds 10 --path ab --out captures/cu
sudo build/wsdr_cu stream --pid 0xc812 --rate 1 --gapped --bw 40 --seconds 5

# Exercise the Receiver API (status JSON once a second; optional retune or sweep)
sudo build/wsdr_rxtest --pid 0xb812 --freq 5190e6 --rate 1 --seconds 10 --retune 5230e6
```

`--rate N` selects fs = 80 MHz >> N: 5 = 2.5, 4 = 5, 3 = 10, 2 = 20, 1 = 40 and 0 = 80 MS/s (gapped
only). `--channel` is the primary 20 MHz channel, so the centre is ch+2 at 40 MHz width and ch+6 at
80 MHz (from any primary, measured). Each tool's source header documents its subcommands and
options: [src/wsdr_probe.cpp](src/wsdr_probe.cpp), [src/wsdr_cu.cpp](src/wsdr_cu.cpp),
[src/wsdr_tcp.cpp](src/wsdr_tcp.cpp) and [src/soapy/SoapyWsdr.cpp](src/soapy/SoapyWsdr.cpp).

Live views: `tools/cu_live.py` shows the 8812CU's A/B spectrum, waterfall and coherence via SoapySDR.
`tools/soapy_check.py` is a headless check of the Soapy module.

## Using the library

`wsdrcore` builds as a subproject. When wifi-sdr is not the top-level project, the tools, tests and
Soapy module are skipped.

```cmake
add_subdirectory(external/wifi-sdr)
target_link_libraries(myapp PRIVATE wsdrcore iqdsp)
```

```cpp
wsdr::Receiver rx(std::make_shared<Logger>());
rx.open(wsdr::kPid8812bu, {.freq = 5190e6, .rate_sel = 1});   // blocking, ~5 s bring-up
rx.start([](const uint8_t *bank, size_t n, const wsdr::BankMeta &m) {
  // Runs on the libusb event thread: copy and return. Unpack (la_i/la_q), DC-remove and
  // shift by m.shift_hz on your own thread; reset DSP state when m.gen changes.
});
rx.set(new_settings);   // applied between stream runs; a same-step retune does not restart
```

## Repository map

| Path | Contents |
|---|---|
| `src/` | The host core: `Session` (devourer bring-up), `la88xx` (LA ring), `cpu88xx`/`cpu8822c` (on-chip loaders), `c2h88xx`, `stream88xx`/`stream_cu` (streamers), `receiver`, `usbmode`, `iqdsp`. Also the CLI tools. |
| `src/fw/` | The on-chip MIPS16e2 programs: marker, C2H bench, LA timing and the streamers, for each chip. |
| `src/soapy/` | The SoapySDR module. |
| `src/mt76sdr/` | MT7612U probe. That chip has no raw-IQ path (`notes/mt7612u-sdr.md`). |
| `tools/` | Analysis and check scripts, the rtw88 guard, the udev rule, and vendor-firmware extraction and RE helpers. |
| `notes/` | Design and research notes: wide tap, 8822C custom code, firmware disassembly, TX feasibility, and more. |
| `evidence/` | Small derived artifacts (JSON, CSV, PNG, logs) behind every claim. Raw captures go to `captures/` (not in git). |
| `external/devourer` | The pinned devourer submodule. |

## Documents

- [FINDINGS.md](FINDINGS.md): the measured facts, including the sample layout, the firmware and CPU,
  the C2H transport, streaming and gapped mode. Start here for "why".
- [REPORT.md](REPORT.md): a phase-by-phase progress report, with reproduction steps and licensing.
- [STATUS.md](STATUS.md) and [HANDOVER.md](HANDOVER.md): the current state, the bench hardware and
  ports, and next steps.
- [LOG.md](LOG.md): the chronology. [NEEDS.md](NEEDS.md): open hardware and decision requests.
- [HUMAN-INVESTIGATION.md](HUMAN-INVESTIGATION.md): a guided map of the workspace.

## Safety

- The tools only receive. On-chip code lives in RAM and is gone after a power cycle.
- The tools never write to efuse or flash, and never change the VID/PID. The registers they touch are
  snapshotted and restored.
- If an adapter hangs, a physical replug recovers it.

## License

The code in `src/` and `tools/` is GPL-2.0-only, matching devourer, which it links. Vendor firmware and
driver files are extracted locally into `vendor/` by `tools/extract_vendor_fw.sh` and are never
committed. ac54_sdr has no license, so it served only as documentation of the
technique: no code was copied from it.
