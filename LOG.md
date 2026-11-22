# LOG — engineering journal (append-only)

Times are local (CET, +01:00).

## 2026-10-25

**06:36** — Workspace created: `~/wifi-sdr/` (git, `main`). `vendor/`, `ref/`, `captures/` gitignored.

**06:36** — Recon.
- `lsusb`: one Realtek adapter, `0bda:b812` "RTL88x2bu [AC1200 Techkey]", bus 2 dev 2, sysfs
  `/sys/devices/pci0000:00/0000:00:08.1/0000:65:00.3/usb2/2-1` (AMD xHCI root port), **5000M (USB 3.0)**,
  bcdUSB 3.00, serial `123456`, MaxPower 504 mA. Interface 0, 5 EPs: bulk IN 0x84, bulk OUT 0x05/0x06/0x08,
  interrupt IN 0x87 (64 B). Bulk wMaxPacketSize 1024.
- Other USB: Cypress keyboard/hub, SteelSeries mouse, MediaTek BT (`0e8d:0616`, btusb).
- PCIe: MT7922 Wi-Fi (`wlp2s0`, **rfkill soft-blocked** — left alone), 2× RTL8125 2.5GbE.
- **Host uplink = `enp3s0`** (192.168.2.242/24, default via 192.168.2.2). Protected: never touched.
- `uhubctl`: "No compatible devices detected" → no VBUS switching available.
- Installed already: uhubctl, usbreset, usb_modeswitch, cmake, gcc, python3 (numpy 2.3.5, matplotlib 3.10.7,
  scipy 1.16.3). DKMS: rtl88x2cu 5.15.8-52, rtl88x2eu 5.15.0.1, rtl8821cu (out-of-tree vendor drivers).
- `/etc/modprobe.d/blacklist-rtw88.conf` already blacklists `rtw88_8822cu` (pre-existing, owner's).
  `rtw88_8822bu` was bound to the 8812BU (`wlx40a5ef515661`, down).
- Kernel 7.0.0-38-generic, Ubuntu 26.04.1.

**06:37** — Cloned refs into `ref/`: djObsidian/ac54_sdr (HEAD 2b41f98; **no LICENSE file**),
djObsidian/wifi-sdr, plaingca/rtl8922-sdr (depth 1), OpenIPC/devourer (HEAD 0bf1957, #496).
OpenIPC master is 37 commits ahead of the local snokvist clone (fc66aa9, #440).

**06:45** — Joakim (mid-session message): OK to bring the snokvist clone up to upstream head and work in it.
`/home/snokvist/dev/snokvist/devourer`: added remote `upstream` = OpenIPC/devourer, `git merge --ff-only
upstream/master` → 0bf1957. Not pushed. Submodules inited: reference/rtl88x2bu, rtl88x2cu, rtl88x2eu.
Built Release (`cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`) — clean.
From here "devourer" = that tree (`$DVR`).

**06:50** — `tools/rtw88_guard.sh on`: temp blacklist `/etc/modprobe.d/zz-wifi-sdr-temp-blacklist.conf`
(rtw88_8822bu, rtw88_8822cu, rtw88_8821cu) + `modprobe -r`. Adapter now driverless; enp3s0 still up.
**Must run `sudo tools/rtw88_guard.sh off` at the end of the work** to restore.

**06:52** — `doctor --pid 0xb812 --channel 6` → SUSPECT (0x40): init OK, EFUSE 4/4 stable (ID 0x8129),
FW ready, but RX 0 ok / 12 CRC. Ch 6 turned out to be RF-quiet here, not a deaf radio (see below).

**06:55** — rxdemo channel survey (9–12 s dwells, `DEVOURER_PID=0xb812`): frames seen —
ch1 1811, ch6 0, ch11 3, ch36 1011, ch44 646, ch100 0, ch149 74 (max RSSI 95), ch157 0.

**06:58** — `doctor --pid 0xb812 --channel 1` → **HEALTHY** (exit 0): rx_ok 1899, rx_crc 636, EFUSE stable,
fw_ready. Chip: 8822B cut 3 (UMC) 2T2R, SYS_CFG1=0x0c493d37, EFUSE rfe_type=0x03.
Evidence: `evidence/phase0/doctor-8812bu-ch{6,1}.{jsonl,stderr}`, `evidence/phase0/rxdemo-8812bu-ch*.jsonl`.

**06:58** — Test signal: no second adapter, no reference SDR → ambient APs (ch1, ch36) for now. NEEDS N1–N5 filed.

**07:00–08:40** — Phase 1 (LA baseline), both chips. All captures via devourer `rxdemo` +
`DEVOURER_LA_CAPTURE`, decoded with new `tools/la_analyze.py`.
- Joakim: OK to use Fable-level agents for firmware probes/software if Opus-level stalls (noted).
- Joakim plugged in the RTL8812CU: `0bda:c812`, bus 5 port 1 (`…/0000:67:00.3/usb5/5-1`), **480M,
  bcdUSB 2.00**, bound to DKMS `rtl88x2cu`. Added `88x2cu` to `tools/rtw88_guard.sh` (temp blacklist,
  restored on `off`); owner's `/etc/modprobe.d/88x2cu-waybeam.conf` untouched.
- 8812BU ch1 manual (4701 samples, readback 324 ms) live noise. ch1 `crcok`: 16104/16384 words stuck at
  0x3d5c57 → CCK freezes the bus (ac54 concurs). ch36 `crcok`/`cca`: live; L-LTF hits 4/4 correct
  orientation. 40M at BW20: genuine 40 Msps. BW40 (ch38 centre): passband ±11 MHz centred, no L-LTF at
  any shift in 7 captures → open.
- 8812CU doctor ch36 HEALTHY. LA readback 12.4 s / 16K samples (USB 2). Paths A+B both decode L-LTFs
  (9/9 correct); B leads A by ≈7.4 samples @20 Msps; per-tone B/A phase stable 0.4–4.3° rms → coherent.
- Off-centre test, ch37 @40 Msps: ch36 AP found only at −5 MHz on both chips/paths.
  Figures `evidence/phase1-{bu,cu}-offcentre.png`.
- Subagent reports: ac54 analysis → `notes/ac54-analysis.md`. Firmware-image characterization (subagent;
  scripts `tools/rtkfw88xx.py`, `tools/extract_vendor_fw.sh`, outputs in gitignored `vendor/`) condensed
  into FINDINGS §3: Linux `rtw8822b_fw.bin` reproduces ac54's vendor-imem byte-for-byte; 8822C has
  IMEM@0x30000 + EMEM, no vectors. Subagent also cloned `ref/88x2bu-20210702` and inited devourer's
  reference/rtl88x2bu + rtl88x2cu submodules in `ref/devourer`.
- No packages installed so far. SDR++/GNU Radio/SoapySDR are not installed.

**~09:00–10:10** — Phase 3 groundwork (8812BU).
- Joakim: SDR++ visualization question → answered (snapshot-mode feed early, real one after streaming).
- Joakim: suggests firmware disassembly / custom firmware for hidden SDR blocks → background subagent
  started on static RE of the 8822B/8822C NIC firmware (→ `notes/fw-disasm.md`).
- Installed `binutils-mipsel-linux-gnu` (2.45.90) for MIPS16e2 assembly. gas encodings match the forms
  ac54 verified on silicon (e.g. extended LUI `f030 6820`).
- New host code (`src/`, GPL-2.0 because it links devourer): `rtlreg` (vendor-req 0x05 regs, SS 504-B
  EP0 cap, OCP SRAM window, DDMA), `session` (devourer CreateRadio + InitWrite), `la88xx` (free-running
  LA ring, snapshot/restore), `cpu88xx` (prologue splice from devourer's FW array, FNV-checked),
  `wsdr_probe` (ring / snap / marker). CMake pulls devourer via add_subdirectory(EXCLUDE_FROM_ALL).
- `wsdr_probe ring`: free-running ring (0x7c0 = 0xCB, never triggered) is **LIVE** after devourer
  bring-up (≈1 % words identical between reads; frozen would be ≥95 %). 128 KiB EP0 read = 12.5 ms
  (10.4 MB/s) on USB 3. dbg 0x0 and 0x880 both live; dbg 0 tags bits [39:32]… with 0x6c, 0x880 → 0.
- InitWrite leaves IGI = 0x40 → ring RMS ≈ 4 LSB (deaf-looking). IGI 0x20 → RMS ≈ 135.
- With CCA on, bits [31:24] of the LA word are non-zero only during frames (0x40/0x80/0x84/0xc5/0xc7/
  0xc8/0xd2…) → BB RX-state flags (lead, undecoded).
- 0x808[29] = 0 (OFDM block off) FREEZES the LA writer (6/6 reads identical). Negative result.
- **0x838[1] = 1 (OFDM RX CCA off, from phydm_stop_ic_trx/phydm_psd) + CCK off + pinned IGI → fixed
  gain**, writer stays live; metadata bits stay 0. `wsdr_probe snap` (contiguous manual-trigger
  snapshots, DVLA output): CCA-on frame shows AGC spike + step-down; CCA-off frame flat at +22 dB,
  no clipping. Evidence `evidence/phase3-snap-{cca,nocca}-frame.png`.

**~10:10–10:30** — 8812BU, continued.
- IGI sweep with CCA off (`evidence/phase3-igi-sweep-bu-ch36.txt`): noise-floor RMS 0x1c→198, 0x20→134,
  0x28→54, 0x30→22, 0x38→9.3, 0x40→4.0 → **≈0.95 dB per IGI step**, ~45 dB usable range.
- `wsdr_probe marker`: custom MIPS16 code runs on the 8822B CPU behind the vendor 0x230 prologue
  (RAM only). 4.2 M loop iterations/s; REG 0x568 advanced 200,256 µs vs 200,232 µs host wall → a true
  µs counter. ISA probe: MIPS16e2 LUI works; ORI and XORI execute as plain LI; ANDI 0xffff executes as
  LUI 0xffff → only LUI of the e2 LI-group exists. Constants must be LUI+ADDIU (`LI32` macro).
- 10:30 — my response was stopped by a safety classifier while writing the next on-chip program
  (a C2H transport benchmark). The truncated file was deleted; that piece is paused pending Joakim.

## 2026-10-25 11:40 — pause
- Declined to write a hand-off brief for other agents to do the stopped on-chip transfer work.
- Refactor: snapshot capture moved into `La8822b::snap()`; `wsdr_probe snap` uses it; builds clean.
- Started a snapshot-mode rtl_tcp server (`src/wsdr_tcp.cpp`); the response was stopped by a safety
  classifier mid-write. Partial file deleted, not retried. Work paused for Joakim's direction.
- Firmware-RE subagent completed; output at `notes/fw-disasm.md` (unreviewed, uncommitted).

## 2026-10-25 12:05 — firmware-RE study cross-check
- Checked `notes/fw-disasm.md` against the vendor sources: halmac register names, 8822B PSD engine
  (0x910/0xf44), 8822C IQK read-out (0x1b38) hold. Correction: the 8822C (JGR3) PSD branch assigns no
  registers in this vendor tree. Summary in FINDINGS §3; the note is committed with its [V]/[I] tags.

## 2026-10-25 12:20 — coord channel; visualization paused
- Added coord/ (one-writer message files + mkdir locks) and tools/coord.sh for the parallel opencode session.
- Started an offline visualization tool (tools/viz_capture.py); the response was stopped by a safety classifier mid-write. Partial file deleted, not retried. Claude session paused.

## 2026-10-25 08:30 — C2H large-packet result (8812BU)
- Host C2H works on RX SRAM and both TX-bank aliases (8/8 each). On-chip 16 KiB latency: 38–46 µs
  (p50 41), one 1024/1024 run at 387 MB/s — not reproducible.
- Sustained 16 KiB C2H wedges randomly after 3–1563 requests (10 of 11 1600-request runs failed;
  two failure signatures, trigger stuck or packet lost). 256 B/1024 B were 2000/2000; 4/8 KiB flaky;
  16 KiB unusable. CPU-driven and host-driven, single URB, unique payloads, varying addresses, TX
  source, RXDMA modes, LPM, FWFF off, agg threshold, CR, thermal off, retries: all negative.
  Forcing 0x284 BIT18 (RW_RELEASE_EN) blocks C2H entirely.
- New files: `src/fw/c2hbench8822b.S`, `src/c2h88xx.{h,cpp}`, `wsdr_probe c2h|c2hbench`; evidence
  `evidence/phase3-c2h-{bench,tail}.*`, `evidence/phase3-c2h-wedge-matrix.txt`.
- Handoff/research brief: `notes/c2h-wedge-handoff.md`. Recommended order: snapshot front end first,
  C2H investigation in parallel; USB2 replug test is the top experiment.

## 2026-10-25 09:55 — E1: C2H over USB2 is reliable (8812BU)
- Replugged the 8812BU with the rtw88 guard ON → enumerates at 480M (1-1); devourer does no USB3 switch.
- 16 KiB C2H, RX source: 1600/1600 ×3 and a 20000/20000 soak, 0 timeouts. TX0/TX1 aliases 1600/1600 ×2
  each. 4 KiB and 8 KiB 2000/2000. On-chip c2hbench 1024/1024 ×3: 347–365 µs/packet, 44.7 MB/s
  (USB2 wire limit). Same code/binary that wedged 10 of 11 runs on USB3.
- Caveat (open): on USB2 the engine is wire-paced ~9× slower (365 vs 41 µs/packet), so "USB3 link"
  and "back-to-back request rate" are not yet separated. The matrix has no `--gap-us` evidence; the
  host-driven depth-1 USB3 runs (~host-RTT paced) still wedged, which leans towards USB3.
- Budget: LA ring is 8 B/sample, so 5 MS/s = 40 MB/s (~90 % of measured USB2; bank 1638 µs vs
  ~1.47 ms ship time) and 2.5 MS/s = 20 MB/s (comfortable). A USB2 continuous mode is unblocked.
- Evidence: `evidence/phase3-c2h-usb2-{16k,bench,sources}.txt`.

## 2026-10-25 10:30 — continuous streaming over USB2 (8812BU)
- New: `src/fw/latime8822b.S` + `wsdr_probe latime` (CPU-timed LA arm→trigger), `src/fw/stream8822b.S`
  + `src/stream88xx.{h,cpp}` + `wsdr_probe stream` (bank-scheduled C2H streamer, in-order bulk reader,
  per-bank framing/deadline/staleness checks, `.cs16` + JSON recorder), `tools/stream_check.py`
  (offline continuity: beacon timing, duplicate banks, boundary steps).
- latime: rate_sel 5 = exactly 2.5 MS/s, writer starts within ~1 µs of the arm, no drift over 5 s.
- Results: 60 s at 2.5 MS/s and 60 s at 5 MS/s, zero faults; ship 1475 µs max per bank (45 % / 91 %
  of the budget). Offline: 455 beacons on exact 102.4 ms multiples, 0 bank slips.
- Found: CPU stores into the bank the ADC is filling are corrupted (bytes 8..15 = copy of 0..7);
  shipping 1 ms early reproduces it. Frozen-writer control (`--ofdm 0`) flags 303/305 banks stale.
- Captures in `captures/phase3/` (not tracked); evidence `evidence/phase3-stream-*`.

## 2026-10-25 10:45 — rtl_tcp front end (wsdr_tcp)
- Committed the C2H probes (fb15901, opencode's work) and the USB2 streamer (dd87516).
- New `wsdr_tcp`: rtl_tcp server on `Stream8822b` (R820T header and gain table; u8 IQ; DC removed;
  NCO for the off-grid residual; arbitrary-rate polyphase resampler from 2.5/5 MS/s; gain → IGI in
  fixed-gain mode, auto → chip AGC; every setting change restarts the stream; drops root after open).
  `src/iqdsp.{h,cpp}` + `iqdsp_test` (tone/alias/DC-bias checks, all pass).
- rtl_tcp carries frequency as a u32: 5 GHz arrives wrapped mod 2^32. The server unwraps
  0.78–1.97 GHz, and `--freq-offset` pairs with a client LNB LO (gqrx clamps to the R820T range:
  LNB LO 5000 MHz + `--freq-offset 5e9`).
- Verified: test client 30 s at 2.400007 MS/s, 254 beacons on the 102.4 ms grid, 0 slips; retune
  ch36→40→36 and gain changes work (~150 ms gap each); gr-osmosdr (gqrx backend) headless and gqrx
  GUI live on ch36, no drops while the client reads. Fixed a u8 truncation bias (−0.5 LSB DC line).
- Evidence: `evidence/phase3-tcp-*`, `evidence/phase3-iqdsp-test.txt`.

## 2026-10-25 11:05 — USB3 test: the wedge is the USB3 path, not the request rate
- Switched the 8812BU to USB3 with a short rtw88 probe (guard off → modprobe → re-enumerated at
  5000M in ~1 s → guard on; NM Wi-Fi radio was already disabled, the wlx interface stayed DOWN).
- 16 KiB C2H on USB3: baseline wedges at 17/174; paced with `--gap-us 1000` (≈ the USB2 cadence that
  is reliable) still wedges at 102/150/15, `--gap-us 300` at 381/0 → **not rate-dependent**.
- RX DMA burst size (0x290[5:4], never varied before): 512 B (0x1e) 4/14, 64 B (0x2e) 107/49 → negative.
- Streamer on USB3: a bank ships in 181 µs (would fit 20 MS/s at 47 % of the 389.6 µs budget) but
  wedges after 5–26 banks at both 2.5 and 20 MS/s.
- Small packets are not a workaround: 1 KiB wedged at 5751/20000, 2 KiB at 243/1600 (on-chip
  4–7 µs per packet when it runs). Per-packet hazard that grows with size; no host-side xHCI errors.
- Evidence: `evidence/phase3-c2h-usb3-{pacing,small}.txt`, `evidence/phase3-stream-usb3.txt`.

## 2026-10-25 11:40 — USB3 wedge: not devourer, not libusb — the RX DMA / bulk-IN path locks
- Register diff on USB3 (`wsdr_probe regs`, rtw88 debugfs mac_* in monitor mode so IPS keeps the MAC on):
  differences in RX aggregation (0x010c bits 0/2, 0x0280 = 0x100), FIFO page boundaries, 0xFE58
  (USB HRPWM), a few SYS bits. rtw88's RX-agg values applied to the C2H test (`c2h --set`): still
  wedges (9/488/23).
- Decisive: C2H with **rtw88 owning the chip** (its own init, kernel URBs; requests through debugfs
  write_reg, packets counted with usbmon; `tools/rtw88_c2h_test.py`) wedges too: 287, 43, then 0.
  After the wedge rtw88's own Wi-Fi RX is dead (0 packets/3 s vs 517/2 s fresh) until a MAC re-init.
- So: large C2H packets on USB3 lock the chip's RX DMA → USB3 bulk-IN path, independent of driver.
  Remaining separators: another xHCI (this host has four AMD controllers; the 8812BU is on 65:00.3,
  the 8812CU's port is on 67:00.3), another board (N6), a hub in between.
- Evidence: `evidence/usb3-regdiff/`.

## 2026-10-25 12:05 — SoapySDR module "wsdr" (Phase 4)
- `src/soapy/SoapyWsdr.cpp` + `cmake/soapy.cmake` (opt-in `-DWSDR_SOAPY=ON`, `build-soapy/libwsdrSupport.so`),
  written by a Claude fork, tested here: RX path A, CS16/CF32, 2.5/5 MS/s, frequency as a double
  (2.40–2.50, 5.080–6.165 GHz; no u32 limit, no LNB trick), gain element "IGI" 0–49.4 dB + AGC mode.
- Hardware (8812BU moved to xHCI 67:00.3, USB2): `tools/soapy_check.py` 20 s at 2,500,381 S/s, 0
  overflows; retune 5180→5200→5180 + gain change while streaming, no errors (~0.1 s per restart);
  close/re-open clean. Offline: 192 beacons on the 102.4 ms grid, 0 slips. gqrx via
  `soapy=0,driver=wsdr` live at 5180.000 MHz (`evidence/phase4-soapy-gqrx-ch36.png`).
- python3-soapysdr 0.8.1 quirk: `SoapySDR.Device(dict(...))` fails with "no match"; use the string form.
- USB2 streaming also holds on the second controller (10 s, 3052 banks, 0 faults).

## 2026-10-25 12:40 — USB3 fixed by the port: 20 MS/s and 40 MS/s for 60 s (Phase 3 done)
- Moved the 8812BU to the port on xHCI 0000:67:00.3 (AMD 1022:15c0; the failing one is 65:00.3,
  1022:15b9), switched it to USB3 there with a short rtw88 probe.
- C2H on the new controller: 16 KiB 1600/1600 ×3 (155 MB/s host-driven), c2hbench 1600/1600 ×3 at
  391 MB/s (39–46 µs), 50000/50000 soak. The wedge was the xHCI/port, not chip, board or code.
- Streamer on USB3: synthetic 20 MS/s byte-exact; **live 20 MS/s 60 s, 146,484 banks, 0 faults**
  (ship 182 µs of a 389.6 µs budget); **live 40 MS/s 60 s ×2 with 256 posted URBs, 0 faults**
  (182 of 202.8 µs). With 64 URBs one 40 MS/s run hit a single 560 µs bank after 39 s (host buffer
  only ~3 ms) — the deeper queue removed it in two runs.
- Offline (20 MS/s, 10 s): 93 beacons, max residual 20 µs, 0 slips, no duplicates; boundary steps
  equal control positions (paired log-ratio −0.086 vs −0.087).
- Evidence: `evidence/phase3-c2h-usb3-xhci2.txt`, `evidence/phase3-stream-usb3-20m*`.

## 2026-10-25 13:10 — channel width and the LA tap; SoapySDR at 10/20/40 MS/s on USB3
- LA writer rate is 80 MHz >> rate_sel at 20/40/80 MHz channel width (latime), and rate_sel 0
  duplicates ~48 % of samples at every width → the LA tap is 40 MS/s at most.
- Devourer's width switch takes effect (0x8ac low bits 0/5/6), but the sampled noise floor is ~±10 MHz
  (−3 dB ≈ 18.4 MHz) at every width with both known taps (dbg 0x880, 0x0): the tap sits after a
  ~20 MHz filter. 40 MS/s is an oversampled 20 MHz channel, not a 40 MHz span. 80 MHz is out (tap
  rate), 160 MHz is out (chip). A wider tap would need another debug-port selector (research).
- SoapySDR module: 2.5/5 MS/s on USB2, plus 10/20/40 MS/s on USB3; bandwidth reported as
  min(rate, 18.4 MHz); 256 URBs at ≥ 20 MS/s; block-copy ring. Python readStream on USB3: 10/20/40 MS/s
  clean for 10/5/10 s at 10,000,844 / 20,002,360 / 40,000,111 S/s, 0 overflows; retune/gain/re-open
  clean at every rate. Offline (20 MS/s): 47 beacons, 0 slips.
- `tools/stream_check.py`: a slip is now a persistent step against the TBTT grid (a deferred beacon is
  one outlier); reports AP clock drift (−16…−21 ppm). Earlier evidence re-checked: 0 slips.

## 2026-10-25 14:10 — wide tap found (RX DFIR); 40 MHz streams at 40 MS/s; custom code on the 8822C
- Research agent (Opus): the ~20 MHz limit is the **RX DFIR**, which devourer programs with the BW20
  set at every width. With the vendor `phydm_rxdfirpar_by_bw_8822b` values (`LaRingCfg::dfir`,
  `--dfir 40|80`): BW40 rate_sel 1 → ±20 MHz floor; BW80 rate_sel 0 → genuine 80 MS/s (adjacent
  duplicates 0.0 vs 0.47), floor ±37 MHz. dma_type 11 = raw ADC, int8, both paths, 160 MS/s/path,
  pre-DFIR (snapshot only). 0x8fc selector does not change the sampled data. Off-centre L-LTF checks
  confirm the wide band is real. `--channel` is the primary 20 MHz channel: BW40 centre = ch+2,
  BW80 centre = ch+6. Notes: `notes/wide-tap.md`; evidence `evidence/phase3-widetap-*`.
- Verified independently here (adj_dup / −6 dB floor) and **streamed 40 MS/s at BW40 with the vendor
  DFIR for 10 s over USB3: 48,828 banks, 0 faults, floor ±20.0 MHz around 5190 MHz.**
- 8812CU: the fork built `wsdr_cu` + the 8822C loader; I found the 8822C hangs on 4 KiB EP0 window
  reads (needs a replug to recover) and IMEM does not read back over DDMA. With 4-byte transfers and no
  IMEM read-back, **the marker runs on the 8822C** (magic, 5.33 M loops/s, exact µs counter, same ISA).
  C2H: trigger clears but no bulk-IN packet yet. `notes/cu-custom-code.md`, `evidence/phase5-cu-*`.
- `Reg::set_cap()` (block-transfer size) and a 3×w32 fallback in `Reg::ddma` for small caps.

## 2026-10-25 14:40 — SoapySDR: full 40 MHz at 40 MS/s
- Soapy module: at 40 MS/s it runs 40 MHz channel width + vendor BW40 DFIR (bandwidth 40 MHz;
  setBandwidth <= 25 MHz selects the 20 MHz path); centre → primary channel (centre − 10 MHz).
  Below 40 MS/s it stays at 20 MHz width. Verified: 40,003,839 S/s, 0 overflows, −6 dB floor
  −20.0…+20.0 MHz; retune/gain/re-open clean; gqrx live at 5190 MHz (`evidence/phase4-soapy-gqrx-40m-bw40.png`).
  A narrow spur near 5198 MHz (also seen by the wide-tap agent) is open.

## 2026-10-25 15:30 — 8822C C2H partly working; devourer DFIR PR opened
- 8812CU C2H: staging and the engine work (0x298 walks the packet); with RX filtered nothing reaches
  bulk-IN; with all frames accepted and the CPU halted 15/64 1 KiB C2H packets arrive. Jaguar3 RX DMA
  seems to flush only under aggregation; with aggregation on the request is not accepted. Details and
  next steps: `notes/cu-custom-code.md`.
- devourer: branch `pr/jaguar2-rx-dfir-by-bw` (8822B RX DFIR by width, vendor values), verified:
  patched devourer alone gives ±19.9 MHz (BW40) / ±38 MHz + genuine 80 MS/s (BW80) on the LA ring; rxdemo
  20 MHz frame counts unchanged within noise; ctest 83/83. **PR: OpenIPC/devourer#497.** The local
  devourer clone is back on master (0bf1957); wifi-sdr keeps `--dfir` / the Soapy BW40 path until it
  merges. Evidence: `evidence/devourer-dfir-ab.txt`.

## 2026-10-25 16:30 — 8822C C2H works; address map found
- The missing 8822C C2H packets were devourer's Jaguar3 coex/phydm runtime thread, which keeps a
  bulk-IN read posted when no RX loop runs (usbmon showed every packet on the wire). With the thread
  off (devourer branch `pr/jaguar3-runtime-thread-optional`, `tuning.j3_runtime_thread`; `Session`
  sets it via SFINAE): host C2H 1600/1600 at 16 KiB; CPU bench 1024/1024 ×3 at 44.7 MB/s.
- C2H→TX map on the 8822C (`wsdr_cu c2hmap`): each bank splits at offset 0x6000; table in
  `notes/cu-custom-code.md`. Next: 8822C LA ring config + streamer with a 5-chunk layout.

## 2026-10-25 12:45 — 8822C streamer works (USB 2, both paths)
- `La8822c` (Jaguar3 LA dialect) gives a live free-running ring; `latime8822c.S` confirms fs = 80 MHz
  >> rate. `stream8822c.S` ships each bank as five C2H packets around the 0x6000 segment split.
- Synthetic 5 s exact; live 60 s at 5 MS/s and 60 s at 2.5 MS/s, zero faults; offline beacon grid
  clean, 0 slips. Dual-path recording: B leads A by 401 ns. One 26 ms-late bank with depth 32 and two
  output files → default depth 256.
- Opened OpenIPC/devourer#498 (optional Jaguar3 runtime thread). MT7612U research agent running.

## 2026-10-25 13:10 — MT7612U research (Opus agent, static RE; no capture yet)
- MCU is Andes NDS32; firmware download has no signature check, and the stock firmware's in-band
  commands 8–12 (burst/random read/write) reach any MCU-bus address → RAM-only custom code looks
  feasible without a modified image [I].
- No IQ ring like Realtek's LA mode. Best candidate: the PBF "ADC dump" capture (0x0440 / 0x0400[14],
  SMM 0x4000 / PKT 0x8000), compiled out for RLT MACs in the vendor driver but likely present [I/?].
- The adapter wedged and dropped off xHCI 67:00.4 during the first devourer bring-up (verified: port
  8-1 does not enumerate). Needs a physical replug, preferably on another port. `notes/mt7612u-sdr.md`.

## 2026-10-25 13:30 — 8812CU fixed gain + SoapySDR; #498 review fixed
- 8822C fixed gain: OFDM CCA off 0x1D58[11:3] = 0x1FF + CCK 0x1A14[9:8] = 3 + IGI 0x1D70; ~1.0 dB per
  IGI step; CCA on compresses frames 6–8 dB at the same IGI, CCA off does not.
- Soapy module drives both adapters (`pid=0xc812` picks the 8812CU; 8812BU stays the default) and
  gives the 8812CU two coherent channels. 20 s two-channel 5 MS/s, retune/gain/re-open, no overflows.
- #498 qodo review: StartRxLoop's phydm worker still ran DIG with the runtime thread off (reproduced:
  IGI=0x1e). Fix 4afa386: skip_dig honoured on Jaguar3; docs scoped. Replied and resolved.
  `evidence/devourer-j3-skipdig-ab.txt`.

## 2026-10-25 14:10 — MT7612U: the PBF ADC capture does not exist on the RLT MAC
- Live test through mt76x2u debugfs (monitor mode ch36, registers restored): the vendor
  Set_ADCDump_Proc sequence captures nothing; 0x440 reads back 0x3f3f3f3f after writing all ones
  (four 6-bit fields, no start bit); the RLT header has no CAP_MODE. No raw-IQ path known on the
  MT7612U. Mtk79ICAP (MT7921AU, RBIST/ICAP, 80 MS/s) is the MediaTek route.
- 8812CU live two-path viewer `tools/cu_live.py` shown on the desktop.

## 2026-10-25 14:45 — core as a library for wsdr-studio
- devourer pinned as submodule `external/devourer` (fork 4afa386); the J3 runtime switch is now
  mandatory at compile time (no silent SFINAE fallback).
- CMake subproject-safe (3.21, CURRENT_SOURCE/BINARY_DIR, `wsdr_fw`); `wsdrcore` streams both chips
  (8822C pieces moved in); tools/Soapy only when top-level. Verified with an empty consumer project.
- `Session::open` throws `OpenError` (not found / permission / busy). Streamers build `C2hRxPath`
  inside the try (an unplug becomes a stream error).
- `Receiver` (src/receiver.*): app-facing open/start/set/status with raw-bank sink, lost/error states.
  `wsdr_rxtest`: 8812BU 40 MS/s 10 s with a ch36→44 retune, 48253 banks, 0 restarts, sink 1.1 µs/bank.
- 8812CU unplugged at ~13:57 (clean USB disconnect, not during a test); its regression is pending.


## 2026-11-22 — 8812BU RX-DMA stall under fast retunes: in-place re-open, Receiver sweep
- Field report (now-studio's 5 GHz scanner: 14 centres, 40 MS/s wide, a `set` every ~350 ms): ~12
  stalls/h. Mid-run (banks 629/846/1256), "no packet within timeout"; after that every start arms
  (DMEM state 1, t_arm advancing, r7c0 0x800000cb) but no packet reaches bulk-IN. CPU reload + LA
  configure never recovers (15 starts); a full re-open does. Same signature as the 65:00.3 C2H wedge
  (LOG 2026-10-25 11:40), here on the good controller 67:00.3 at a far lower rate.
- `Receiver` now re-opens the adapter in place (full bring-up, same bus/port) at the 2nd and 4th
  consecutive fault, Error only at the 5th. Measured: 7 re-opens in 10 min, all recovered, ~2.3 s each.
- `Receiver::sweep(centres, dwell_s, gap_s)`: one run per centre, BankMeta step/pass. 5 min each,
  same 14 centres, 160 ms dwell: gap 0 → 223 ms/step, 5 stalls in 1369 runs (0.37 %); gap 30 ms →
  245 ms/step (p90 246), 2 stalls in 1208 runs (0.17 %, the set loop's ~0.11 %). Short runs, so
  suggestive only.
- The "no first packet in 20 ms, starting again" retries show the stall's exact state (state 1,
  r7c0 800000cb) and cluster right before stalls (3 of 6, then 2 of 2 episodes; ~1 retry in 250 runs).
  They look like an early symptom of the RX path failing, not a cause. Open: what degrades it.
