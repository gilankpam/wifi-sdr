# FINDINGS

Register maps, protocol notes, firmware behaviour and quirks, written for someone who has not read the
journal. Each claim cites its evidence (a file in `evidence/`, a capture in `captures/` — regenerable,
not in git — or a source file:line). Verified vs inferred is marked.

## Conventions
- "devourer" = OpenIPC/devourer at 0bf1957 (local clone `~/dev/snokvist/devourer`, `$DVR`).
- "ac54" = djObsidian/ac54_sdr at 2b41f98 (`ref/ac54_sdr`). **No LICENSE** → used as documentation of a
  technique only; no code copied (NEEDS N4). Full read-out with file:line citations:
  [`notes/ac54-analysis.md`](notes/ac54-analysis.md).
- LA = phydm "logic analyzer" / ADC-sampling mode. Sample word = one u64 in the TX-FIFO ring.

---

## 1. Bench units

| | RTL8812BU `0bda:b812` | RTL8812CU `0bda:c812` |
|---|---|---|
| die / cut | 8822B cut 3 (UMC), SYS_CFG1 0x0c493d37 | 8822C cut 5 (UMC), SYS_CFG1 0x0c495d37 |
| chains | 2T2R | 2T2R |
| EFUSE | id 0x8129, rfe_type 3 (stable 4/4 reads) | id 0x8129, rfe_type 3 (stable 4/4) |
| USB as found | **SuperSpeed 5000M**, bcdUSB 3.00, bulk MPS 1024 | **High-Speed 480M**, bcdUSB 2.00, bulk MPS 512 |
| vendor FW boot (devourer DLFW) | OK, 0x80 = 0xC078 | OK, 0x80 = 0xC078 |
| doctor | HEALTHY ch1 (1899 ok / 636 crc) | HEALTHY ch36 (2676 ok / 0 crc) |

Evidence: `evidence/phase0/doctor-8812{bu,cu}-*.{jsonl,stderr}`.

- The 8812BU was already in USB 3 mode because `rtw88_8822bu` performs the USB2→USB3 mode switch at probe
  and it persists until power loss. A cold-plugged unit enumerates USB 2 first (ac54 `tools/usb3.c`;
  rtw88 `rtw_usb_switch_mode_new`). **Not a permanent change** — a register write, lost on power-off.
- The 8812CU came up on a USB 2 root port with bcdUSB 2.00. **This board cannot switch to USB 3**
  (2026-10-25): in the 8812BU's USB 3 port (xHCI 0000:67:00.3, SS bus 6) it enumerates at 480M, and
  `rtw88_8822cu` (kernel 7.0, `switch_usb_mode=Y`, `debug_mask` on) logs "Switching to USB 3 mode
  disabled by chip's efuse". The board does not appear to be wired for USB 3 either. So this 8812CU
  is USB 2 only: continuous streaming tops out at 5 MS/s (both paths, 40 MB/s); wider spans would
  need gapped capture (burst at 40 MS/s, ship over USB 2, ~12 % duty).
- USB-2 ceiling: ~35–40 MB/s of bulk payload in practice → 20 Msps of 16-bit IQ (80 MB/s) cannot fit;
  12-bit packed at 20 Msps (60 MB/s) cannot either. Without USB 3 the 8812CU tops out near 10 Msps (24-bit)
  — to be measured, not assumed.

## 2. LA-mode sample path, measured on these units (Phase 1)

Tool: devourer `rxdemo` with `DEVOURER_LA_CAPTURE` (src/LaCapture.cpp); analysis `tools/la_analyze.py`
(burst detection, PSD, L-LTF correlation for x and conj(x), CFO), `tools/phase1_figure.py`.

### Layout (verified on both chips)
- `dma0` + dbg port `0x880`: **one complex sample per u64**, 12-bit two's complement,
  **path A: I = [11:0], Q = [23:12]**. 8822C additionally carries **path B: Q = [43:32], I = [55:44]**.
  8822B high dword is all zero at this setting.
- Orientation: with this layout, ambient-frame L-LTFs correlate against the reference L-LTF and **not**
  against its mirror. 8812BU: 4/4 detections correct (peak 0.69–0.78 vs ≤0.44 conjugated). 8812CU: 9/9
  correct on both paths (0.78–0.97 vs ≤0.46). The L-LTF tone set is asymmetric in k, so a spectrally
  inverted capture would correlate as conj(x). Evidence: `evidence/phase1-*-cca-20M*.json`.
- Frequency placement: tuned to ch37 (5185 MHz) at 40 Msps, the ch36 AP (5180) is detected **only** after a
  −5 MHz shift, never at 0 or +5 — both chips, both 8822C paths. Figures:
  `evidence/phase1-bu-offcentre.png`, `evidence/phase1-cu-offcentre.png`.
- Rates: at BW20, the `40M` LA rate gives **genuine 40 Msps** (adjacent-duplicate fraction ≤1.5 %; L-LTF
  found with the 40 Msps reference; same CFO as at 20 Msps). ac54 reports the BW20 ADC is 40 Msps and that
  lower LA rates are sample *selection* (no anti-alias filter at that tap: a +11 MHz tone folds to
  −8.96 MHz at 20 Msps) — consistent with the ~±11 MHz passband seen here at 40 Msps.
- CFO of the ch36 AP vs each unit: 8812BU +77…+95 kHz, 8812CU +28…+48 kHz (≈15 and ≈7 ppm at 5180 MHz;
  partly the AP's own offset).

### 8822C two-path coherence (verified)
- Path B **leads** path A by a fixed **≈7.4 samples at 20 Msps (≈370 ns)** in every capture (integer
  part 7 or 8, per-tone phase-slope residual −0.35 / +0.6 respectively). Not physical (≈110 m of path);
  a capture-pipeline skew → **delay B by ~7.4 samples** before any coherent combining.
- After aligning, the per-tone ratio B/A is stable between the two L-LTF symbols to **0.4–4.3° rms**
  (9 frames, 5 captures): the chains share an LO and the capture is phase-coherent.
- Observation (unexplained): at 40 Msps path B's out-of-band floor (|f| > 12 MHz) is ~20 dB higher than
  path A's — path B looks less filtered. Open.

### Quirks
- **CCK freezes the 8822B sample bus.** At ch1 (2.4 GHz) a `crcok`-triggered capture held 16104 of 16384
  words at one constant (0x3d5c57) followed by ~14 µs of live data. On 5 GHz (no CCK) the ring is live
  throughout (max repeated word 112/16384). ac54 independently saw "whole banks of constant IQ" with CCK on
  and streams with `0x808[28]` (CCK block enable) cleared (`ac54-analysis.md` §4). → the streamer must clear
  `0x808[28]`. Not yet tested on the 8822C.
- **Post-AGC bus.** Captured frames sit only a few dB above the idle noise: the gain is switched per frame
  (visible as AGC steps / few-µs holds in |x|). An SDR needs a frozen gain; ac54 lists this as unsolved.
- **BW40 tap unexplained.** With `DEVOURER_BW=40` (RF at ch38 / 5190 MHz, `rf18=0x10926`) the 40M tap still
  shows a ±11 MHz passband centred at 0 and no L-LTF at 0/±10 MHz shifts (7 captures). Either 0x880 is a
  post-DFE primary-channel signal in BW40, or the filter/ADC config differs. Open; streaming targets BW20.
- Readback cost over EP0 (two reads per sample): 16384 samples in **1.22 s on USB 3** (8812BU) vs
  **12.4 s on USB 2** (8812CU). devourer's "~1 s per 1K" figure is a USB-2 number for a different host.
- Manual trigger fires immediately after arm → a manual capture is a random 0.2–0.8 ms window; at
  ≈100–200 ambient frames/s it usually holds no frame. `cca` (frame start) + `t<us>` post-trigger
  reliably holds a preamble.

## 3. Firmware images and the 88xx embedded CPU

Extracted locally by `tools/extract_vendor_fw.sh` / `tools/rtkfw88xx.py` into gitignored `vendor/`
(hashes in `vendor/SHA256SUMS`). Never committed.

- **ac54's "vendor IMEM" is not from the Windows driver.** `ac54/tools/fw_inspect.py:7-16` cuts it from
  `array_mp_8822b_fw_nic` in morrownr/88x2bu-20210702 (d31ffa82) `hal/rtl8822b/hal8822b_fw.c`. That array
  is byte-identical to `/lib/firmware/rtw88/rtw8822b_fw.bin` and devourer `hal/hal8822b_fw.c`
  (8822B NIC v30.20.0, 2020-05-14, image sha256 a72da690…). `rtkfw88xx.py info
  /lib/firmware/rtw88/rtw8822b_fw.bin.zst --ac54 DIR` reproduces ac54's `vendor-imem.bin` exactly
  (149952 B, sha256 f09acd6f6d047ef31bd4c25c950afd5a9d8527331385e752918b1f2f8de17d3f).
- Only **IMEM[0:0x230]** is used (sha256 7a2b616d4c1312dd48dccddb897ed148d5012aa36faa538581922cbcf1ba059d).
  It is: `0x000 jal 0x80000200`; an exception stub at 0x080 jumping to 0x80000329; at 0x200 SP =
  0x8020C000 (DMEM), CP0 Status = 0xff00 (IE=0), CP0 $20 clear 0x800 / set 0x400; fall-through to 0x230.
  Identical in every 8822B build examined (Linux v30.20 NIC/AP/WoW, Windows v28.5 NIC). Hazard: the
  exception vector lands inside whatever code is appended at 0x230 (+0xF8).
- **Header (88xx "3081" format, LE):** sig u16 @0x00, version @0x04, subversion @0x06, build date
  @0x10–0x15, mem_usage @0x18, dmem_addr/size @0x20/0x24, imem_size @0x30, emem_size/addr @0x34/0x38,
  imem_addr @0x3C. Body = DMEM, IMEM, [EMEM], each + 8-byte trailer (u16 XOR of LE halfwords + 6 zero
  bytes). Load address = header address with bit 31 cleared. **No signature field; no secure boot on the
  host download path** (halmac `chk_fw_size` demands the exact size).
- **CPU:** little-endian MIPS executing **MIPS16 / MIPS16e2** (8822B IMEM: 0 MIPS32 `jr ra`/`eret`; 3213
  in-image MIPS16 JALs, 0 JALX). Opcode 10111 is repurposed as CP0 moves. Inferred: Lexra/RLX-class core
  (R3000-style vector at 0x80000080). ac54 found only MIPS16e2 code executes after the vendor prologue.
- **Address map:** OCP/DMA view — 8822B IMEM 0x0, DMEM 0x200000, TX buffer 0x18780000; CPU kseg0 =
  0x80000000 + OCP; uncached kseg1 MAC registers 0xB86x_xxxx (0xB8640568 = REG 0x568), RX SRAM 0xB8700000,
  TX SRAM 0xB8780000.
- **A started CPU can stall at its first MAC-register access until the host reads the MAC block**
  (2026-10-25, issue #2; 8812BU on USB 3, `wsdr_rxtest --sweep` with 3 cores of host load). Now and then
  (47 starts in 1088; ~1 in 20 in wsdr-studio's sweep) stream8822b.S did not arm the LA (0x7C0) after
  `Cpu8822b::start`, then armed at the host's next 0x0568 read, ~20 ms later, to the µs (`t_arm` = the
  host's reading). Host reads of 0x0003 or 0x0080 (system block) did not release it. Polling 0x7C0 from
  right after the start: 0 stalls in ~3000 starts, armed by the first read 97 % of the time. Mechanism
  unknown. Untested on the 8822C, whose streamer starts its CPU the same way.
- **8822C differs:** IMEM at **0x30000**, EMEM at **0x100000** (holds most code), DMEM 0x200000; IROM field
  0x10000; IMEM starts directly with SP/CP0 init (SP 0x80210000, Status 0xfe00) and has **no reset or
  exception vectors** → inferred mask ROM owns reset/exceptions. ac54's "keep vendor prologue, append code"
  does not map 1:1; the 0x09 clock-sync toggle is 8822B-only. Linux images: rtw88 rtw8822c_fw.bin v9.9.15;
  DKMS 88x2cu / devourer hal8822c_fw.c v9.17.0.
- **Download protocol (halmac_fw_88xx.c, identical for B and C):** CPU off (0x03 b2, 0x1D b0) → pltfm reset
  (0x1082 b0; +0x09 b6 on 8822B/8821C) → 0x80 b0 FWDL_EN → each section in 8192-B chunks as reserved page,
  DDMA ch0 (0x1200 SA = 0x18780000+48, 0x1204 DA, 0x1208 = OWN b31 | CHKSUM_EN b29 | len [| CONT b24]) →
  host writes DW_OK/CHKSUM_OK into 0x80 → FW_DW_RDY, CPU on, poll 0x80 == 0xC078 (failure: 0x10FC ==
  0xFAAAAA00 "illegal key").

### Static firmware RE (`notes/fw-disasm.md`): what was cross-checked
A subagent's read-only study of the NIC images; it tags claims [V]/[I]. Checked here against the
vendor sources (`~/dev/snokvist/devourer/reference/rtl88x2cu/hal/`) on 2026-10-25:
- **Register names hold:** `REG_C2H_PKT` 0x0294, `REG_FREERUN_CNT` 0x0568 (the µs timer measured in §4),
  `REG_WMAC_FWPKT_CR` 0x0601 at the cited `halmac_reg2.h` lines.
- **Hardware PSD engine, 8822B (11AC_2 branch):** `phydm_psd.c:461-462` sets control 0x910 and report
  0xf44; a tone index goes in 0x910's low bits, BIT(22) pulses the trigger, it settles
  `10 << (fft_points >> 7)` µs, and the report converts to dB **plus the current IGI**. That gives one
  tone per trigger, magnitude only, no phase: a spectrum-occupancy sensor, not an IQ source.
  **Correction to the study:** on the **8822C** (JGR3 branch) this vendor tree assigns no PSD
  registers. Its 0x1e8c/0x2d90 assignment sits under `#if 0`, so "0x910 on 8822C" is wrong.
- **IQK read-out, 8822C (`halrf_iqk_8822c.c`):** writing 0x40000000 to BB 0x1b38 triggers (:464, :601).
  The result reads as x = 0x1b38[31:20] and y = 0x1b38[19:8] (:439-440): scalar IQ-mismatch estimates,
  not a sample buffer. They could post-correct LA captures offline. The 8822B equivalent is not checked.
- **Stock firmware and gain:** none of the LA, dbg-port or IGI registers appears as a full 32-bit
  constant in either image's literal pool. Per the study's own caveat that is suggestive, not proof,
  since base+offset addressing escapes the scan. It is consistent with §4: the 8822B fixed-gain
  recipe is host-side register writes. The study's §7(b) lists AGC freezing as open; on our 8822B it
  is solved (§4); on the 8822C it is still untested.
- **Not cross-checked:** H2C table locations and index map, call-graph leaf identities, the 8822C entry
  stub, and the §7 leads. Treat them as the study's own claims.

## 4. Continuous streaming data path (ac54, 8822B) — Phase 2

Source: ac54 (`notes/ac54-analysis.md` §2–§8). Status: **transport reproduced on our 8812BU; the
large-packet path is not yet sustainable (see the C2H subsection below).**

```
 ADC (BW20: 40 Msps) ─LA block (free-running, never triggered)─► 128 KiB ring in TX-FIFO SRAM
   0x20000..0x40000 (OCP 0x18780000+off), 8 B/sample, two 64 KiB banks (bank period at 20 Msps = 409.6 µs)
        │  embedded MIPS16e2 program (vendor 0x230 prologue + custom code at IMEM 0x230)
        │  waits on free-running µs counter REG 0x568 (CPU 0xB8640568) for "bank complete"
        ▼  per bank: saves 224-B prefix, writes 24-B RX descriptors in-place, issues 4 overlapping
        │  C2H requests (16352 B each, 24 B overlap, first at bank+200) + 1 prefix/trailer packet
        │  via REG 0x294 (poll bit 16); TX-SRAM→C2H address alias is non-trivial (banks at 0x16000/0x6000)
        ▼
   USB bulk-IN EP 0x84: each transfer = 24-B RX desc (len 14b in [0..1], OWN 0x80 @3, C2H 0x10 @11) + payload
        ▼
   host: ≥32 queued async 64 KiB transfers; checks header, length, overlap bytes, trailer seq ==
   bank, deadline ordering, full 64 KiB coverage, one-ring-lag freshness (≥95 % identical = frozen)
        ▼
   output: interleaved LE int16 I,Q (12-bit × 16)
```

Key preconditions (ac54): MAC RX **must stay enabled** (0x100 |= 0x8a) — disabling it freezes the ADC
writer (their earlier "live" streams were frozen-ring false positives); frames filtered instead
(0x6a0/0x6a2/0x6a4 = 0, 0x608 |= 0x80000, low 16 cleared, RXDMA 0x290 = 14, 0x10c &= ~4); CCK off
(0x808[28] = 0); LA: 0x7c0 = 0 stop, 0x8f8 &= ~0x3c00000, 0x8b4 |= 0x80, 0x95c [11:8] dma_type, [7:5]
rate (fs = 80 MHz >> rate), [4:0] = 3; dbg port 0x8fc = 0; arm 0x7c0 = 0xcb. CPU load: halt → clock-sync
reset (0x1082/0x09) → 0x1080 |= 0x10100 → 0x80 = (old & 0x3800) | 1 → stage image in TXFF at 0x1000 →
DDMA to IMEM 0 → read back and compare → 0x80 = (old & 0x3800) | 0x4078, 0x1d |= 1, 0x03 |= 4.
EP0 reads on SuperSpeed are capped at 504 B per transfer.

Their measured results: 20 Msps for 60 s with 0 faults (single unit, quiet host); 40 Msps only in bursts
(best 0.66 s) — the limit is C2H latency outliers (normal 38–45 µs, worst 369 µs) against a 204.8 µs
opposite-bank window, not USB average throughput. On-chip packing tops out at 3.8 Msps; DDMA at ~15 MB/s.

### Measured on our 8812BU (Phase 3 groundwork)
- Free-running ring (0x7c0 = 0xCB, never triggered) is **live** after devourer `InitWrite` bring-up, with
  MAC RX on and all frames filtered (≈1 % of words identical between two reads; frozen ≥ 95 %). Both dbg
  port 0x0 and 0x880 are live (0x0 tags the upper word bits with a constant; 0x880 leaves them 0) —
  resolves unknown 1. A 128 KiB ring read over EP0 takes 12.5 ms (10.4 MB/s) on USB 3.
- `InitWrite` leaves IGI (0xc50/0xe50 [6:0]) at 0x40 → ≈4 LSB RMS. Pinning IGI is the gain control:
  **≈0.95 dB per step**, noise-floor RMS 198 @0x1c … 9.3 @0x38 (`evidence/phase3-igi-sweep-bu-ch36.txt`).
- **Fixed-gain mode (resolves unknown 3 for the 8822B):** OFDM RX CCA off (`0x838[1] = 1`, the 11AC
  branch of phydm `phydm_stop_ic_trx`, used by phydm's own PSD routine) + CCK off (`0x808[28] = 0`) +
  pinned IGI. No packet detection → no AGC steps; the LA writer stays live; the [31:24] state bits stay 0.
  Contiguous snapshots: CCA on → frame shows the AGC spike then a gain step-down; CCA off → frame flat at
  +22 dB over noise, no clipping at IGI 0x20 (`evidence/phase3-snap-{cca,nocca}-frame.png`).
- Negative: `0x808[29] = 0` (OFDM block off) **freezes** the LA writer.
- With CCA on, word bits [31:24] are non-zero only during frames (0x40, 0x80, 0x83–0x86, 0xc5, 0xc7, 0xc8,
  0xd2) — BB receive-state flags; undecoded lead.
- **Embedded CPU:** custom code at IMEM 0x230 runs (`wsdr_probe marker`): ≈4.2 M simple loop iterations/s;
  REG 0x568 (CPU 0xB8640568) is a µs counter (200,256 vs 200,232 µs host wall). Only **LUI** of the
  MIPS16e2 extended-LI group is implemented — ORI/XORI behave as LI, ANDI 0xffff as LUI 0xffff — so 32-bit
  constants must be LUI + ADDIU. After our code starts, 0x80 reads 0x4078 (no vendor ready bit 15).

### C2H large-packet transport on our 8812BU (Phase 3, 2026-10-25)
- **Working:** host-driven C2H and the on-chip bench. RX SRAM and both TX-bank aliases (TX0 0x16000,
  TX1 0x6000) deliver 8/8 packets; on-chip 16 KiB latency is 38–46 µs (min/p50/p99/max =
  38/41/45/46, mean 41.4) — inside the 204.8 µs 40 Msps bank window. One 1024-request run completed
  1024/1024 at 387 MB/s; it is **not reproducible**.
- **Blocked:** sustained 16 KiB C2H wedges after a random 3–1563 requests. Ten of eleven 1600-request
  runs failed (720, 0, 166, 813, 394, 56, 369, 428, 405, 323 completed); two signatures: the 0x294
  BIT16 trigger stays set with no packet, or the trigger clears and the packet never arrives. Retry,
  abort, restage, gap and descriptor checks do not recover; only a fresh bring-up does.
  `r288` overflow (BIT7) appears only after retries; `r280=0x0a03`, `r10c=0xf5a0`, `r284=0`.
- **Size dependence:** 256 B and 1024 B were 2000/2000 (two runs each); 4 KiB and 8 KiB were flaky
  (one 2000 pass, one early fail each); 16 KiB is unusable.
- **Ruled out (all negative):** CPU-driven vs host-driven, single vs 16 outstanding URBs, unique
  payload per request, rotating 16 KiB addresses, TX source, MAC RX on/off, CR force, RXDMA modes
  0/2/12/14, USB LPM clear, agg-threshold low byte, FWFF off, thermal thread off, RW_RELEASE_EN=0.
  Forcing Rw release (0x284 BIT18) sets RXDMA_IDLE and blocks C2H entirely (0 completions).
- **Handoff / next experiments:** `notes/c2h-wedge-handoff.md` (USB2 replug test first, then a
  devourer-vs-rtw88 register diff, a second host controller/board, and a wedge recovery without
  re-open). Matrix: `evidence/phase3-c2h-wedge-matrix.txt`.
- **USB2 (2026-10-25, E1):** after a replug with the rtw88 guard on, the 8812BU enumerates at 480M and
  the same 16 KiB C2H is reliable: 1600/1600 ×3, a 20000/20000 soak, TX0/TX1 1600/1600 ×2, 4/8 KiB
  2000/2000, c2hbench 1024/1024 ×3 at 347–365 µs/packet = 44.7 MB/s (wire-limited). Open: on USB2 the
  engine is paced ~9× slower, so "USB3 link" vs "back-to-back rate" is not yet separated (the matrix
  has no `--gap-us` run). Evidence: `evidence/phase3-c2h-usb2-*.txt`.
- **USB3, rate separated (2026-10-25):** paced to the USB2 cadence (`--gap-us 1000`) 16 KiB C2H still
  wedges on USB3 (102/150/15), so the failure belongs to the USB3 path, not to back-to-back requests.
  RX DMA burst size 512/64 B (0x290 = 0x1e/0x2e) is also negative. Smaller packets only lower the
  per-packet hazard (1 KiB: 5751/20000; 2 KiB: 243/1600). The streamer on USB3 ships a bank in 181 µs
  (enough for 20 MS/s) but wedges after 5–26 banks. No xHCI errors are logged. Remaining leads:
  rtw88-initialized chip without devourer init (E3), another host controller (E2), another board (E6).
- **USB3, driver ruled out:** with rtw88 owning the chip (its init, its kernel URBs; requests via debugfs,
  usbmon counting) 16 KiB C2H wedges at 287 and 43 requests, and rtw88's normal RX dies with it until
  the MAC is re-initialized. Applying rtw88's RX aggregation registers (0x10c = 0xf5a5, 0x280 = 0x100)
  to our path does not help either. The fault is in the chip's RX DMA → USB3 bulk-IN path when it
  carries C2H packets, on this board / this AMD xHCI (1022:15b9). Next separators: another xHCI
  controller, another 8822BU board, a USB3 hub in between. `evidence/usb3-regdiff/`.
- **USB3 resolved — it is the host controller/port.** On this host's other xHCI (0000:67:00.3, AMD
  1022:15c0) the same adapter, code and init run 16 KiB C2H 1600/1600 ×3, a 50k soak, c2hbench at
  391 MB/s, and the streamer at **20 MS/s for 60 s** and **40 MS/s for 60 s** (×2) with zero faults.
  The failing controller is 0000:65:00.3 (AMD 1022:15b9). At 40 MS/s the host needs a deep URB queue
  (256 × 16 KiB ≈ 13 ms); with 64 one bank once took 560 µs (> 202.8 µs budget) after 39 s.
  `evidence/phase3-c2h-usb3-xhci2.txt`, `evidence/phase3-stream-usb3-20m*`.
- **Channel width vs the LA tap:** the LA writer runs at 80 MHz >> rate_sel at 20/40/80 MHz width, and
  rate_sel 0 duplicates ~48 % of samples at every width (true ceiling 40 MS/s). Width switching works
  (0x8ac = …0/…5/…6), but the sampled noise floor stays ±10 MHz (−3 dB ≈ 18.4 MHz) at every width for
  dbg 0x880 and 0x0, so the tap is after a ~20 MHz filter. `evidence/phase3-bw-*.txt`.
- **Correction — the ~20 MHz limit is the RX DFIR, not the tap.** Devourer writes the BW20 DFIR set at
  every width. With the vendor values (0x948[29:28] / 0x94c[29:28] / 0xc20[31]=0xe20[31] = BW40 1/0/0,
  BW80 2/1/0; `LaRingCfg::dfir`): BW40 at 40 MS/s gives a ±20 MHz floor (streamed 10 s over USB3, 0
  faults); BW80 at rate_sel 0 is a genuine 80 MS/s tap (adjacent duplicates 0.0), floor ±37 MHz
  (snapshot only: 640 MB/s exceeds the ~391 MB/s C2H ceiling). 0xc20[31] = 1 forces the narrow,
  doubled path whatever 0x948/0x94c hold. dma_type 11 is the raw ADC (int8, both paths, 160 MS/s per
  path, pre-DFIR). `--channel` is the primary channel (centre ch+2 at BW40, ch+6 at BW80).
  `notes/wide-tap.md`, `evidence/phase3-widetap-*`.

### Continuous streaming over USB2 (Phase 3, 2026-10-25)
- **Working:** `wsdr_probe stream` + `src/fw/stream8822b.S` stream the LA ring continuously. 60 s at
  **2.5 MS/s** (rate_sel 5, 20 MB/s) and 60 s at **5 MS/s** (rate_sel 4, 40 MB/s): 18311 / 36621 banks,
  zero strays, zero stale banks, every bank's framing and trailer sequence checked; on-chip shipping
  start is 0–0.8 µs after the bank is due; a bank ships in 1454–1475 µs (4 × 16376 B + 168 B trailer,
  USB2 wire time) against a 3256.8 / 1618.4 µs budget (45 % / 91 %). `evidence/phase3-stream-*`.
- **Offline continuity** (`tools/stream_check.py`, 60 s 2.5 MS/s capture, sha256 in evidence): 455 beacons
  of the ch36 AP, all intervals exact multiples of 102.4 ms (median residual 0 samples, max 320 µs
  CSMA jitter), 0 bank slips; no duplicate banks; sample steps at bank/chunk boundaries equal those at
  control offsets (paired log-ratio −0.07 ± 0.004 everywhere). The ~1.7 % "identical to one ring
  earlier" rate is lag-independent: it is the narrow noise-floor value distribution, not stale data.
- **Rate selectors:** 0x95c[7:5] = 4, 5, 6 all give live, non-duplicated data at BW20. `latime` (CPU
  arms, waits D µs on REG 0x568, triggers) measures rate 5 as exactly 2.5 MS/s; the writer starts at
  TXFF 0x20000 within ~1 µs of the arm write; after 5 s (12.5 M samples) the predicted ring position is
  exact (no ADC/REG 0x568 drift). So an open-loop bank schedule from the µs counter is exact.
- **Hazard (measured):** CPU 32-bit stores into the ring bank the ADC is *currently filling* are
  corrupted even far from the write pointer: a descriptor's bytes 8..15 came back as a copy of bytes
  0..7 (consistent with a 64-bit read-modify-write conflict). Shipping 1 ms early reproduces it every
  time (framing fault, run stops). CPU writes into a bank are therefore only safe while the ADC is in
  the other bank; the schedule keeps descriptor and sentinel writes inside that window (at 5 MS/s with
  ~160 µs to spare).
- **Integrity controls:** a frozen LA writer (`--ofdm 0`) is flagged stale on 303/305 banks (descriptor
  + last-sample sentinel survive a revolution); the synthetic mode (host-filled banks, LA stopped)
  matches the fill pattern byte for byte. 10 Msps (80 MB/s) cannot fit USB2.
- **rtl_tcp front end (`wsdr_tcp`):** the protocol's 32-bit frequency word cannot express 5 GHz; gr-osmosdr
  sends 5180 MHz as 885 032 704 (mod 2^32), and gqrx clamps tuning to the advertised R820T range. Both
  are handled (unwrap of 0.78–1.97 GHz; `--freq-offset` + client LNB LO). gr-osmosdr centres u8 at 127.4,
  which leaves a small DC line at this radio's low noise floor (gqrx "DC remove" clears it). Retune or
  gain change costs ~150 ms of samples (stream restart + devourer retune).

### Continuous streaming on the 8822C (Phase 5, 2026-10-25)
- **Working:** `wsdr_cu stream` + `src/fw/stream8822c.S` (`Stream8822c`, `src/stream_cu.*`) on the
  8812CU over USB 2. 60 s at **5 MS/s** (36,621 banks) and 60 s at **2.5 MS/s** (18,311 banks), zero
  strays, zero stale banks, framing and trailer sequence checked on every bank; shipping starts
  ≤ 0.8 µs after the bank is due and takes 1481–1484 µs per bank (USB 2 wire time) against
  1618.4 / 3256.8 µs. A synthetic run (host-filled banks) matched the pattern exactly.
  `evidence/phase5-cu-stream.txt`, `-60s.jsonl`.
- **Layout:** the 8822C's C2H window splits each bank at 0x6000 (map in `notes/cu-custom-code.md`),
  so a bank ships as five packets that never cross the split: [128, 12352), [12352, 0x6000),
  [0x6018, 0x9568), [0x9568, 0xCAB8), [0xCAB8, 0x10000), plus a trailer carrying the 128 B prefix
  and the 24 B at 0x6000 (both overwritten by descriptors).
- **LA ring, Jaguar3 dialect (`La8822c`):** 0x1CE4 ([5:0] dma, [10:8] rate, [17:13] trig sel 3),
  0x1CF4[23], 0x1EB4[23], 0x1C3C[19:8] = dbg port 0x880, 0x7CC[30] = 0, arm 0x7C0 = 0xCB. Live with
  the vendor CPU running or halted (0 % identical words between reads). **Rate:** CPU-timed
  (`latime8822c.S`) fs = 80 MHz >> rate for rate 1–5 (ratio 1.0003–1.0012, i.e. a few samples of arm
  latency); the ring starts at TXFF 0x20000 at the arm write, as on the 8822B.
- **Offline** (`tools/stream_check.py`, now 8822C-layout aware): 567 / 552 beacons of the ch36 AP on
  the 102.4 ms grid (median residual 0 samples), 0 bank slips, 0 duplicate banks; steps at bank,
  chunk and prefix boundaries equal the overall mean. sha256 in `evidence/phase5-cu-stream-60s.sha256`.
- **Two paths for free:** `--path ab` writes both. B leads A by **2.0 samples = 401 ns** at 5 MS/s
  (Phase 1: ≈370 ns); single-tap coherence 0.58 in frame bursts vs 0.02 in noise
  (`tools/cu_twopath.py`, `evidence/phase5-cu-stream-5m-ab-twopath.json`).
- **Host queue depth matters:** with 32 queued transfers and two output files one bank ended 26 ms
  late (a host write stall back-pressures the chip's C2H engine); 256 (the new default, ~100 ms at
  5 MS/s) ran clean.
- **Fixed gain (8822C):** Jaguar3 phydm_stop_ic_trx / phydm_dis_cck_trx: OFDM RX CCA off
  `0x1D58[11:3] = 0x1FF`, CCK RxIQ weighting `0x1A14[9:8] = 3`, IGI pinned in `0x1D70[6:0]/[14:8]`
  (`--no-cca --igi N`). IGI sweep at ch36 (`evidence/phase5-cu-igi-sweep-ch36.txt`): ≈1.0 dB per step
  on the noise floor over 0x20–0x3C (floor 98.6 → 3.9 LSB RMS); maximum gain for IGI ≤ 0x1C; above
  0x3C the thermal floor sinks under the ADC's ~3 LSB floor, while frame peaks keep falling ≈1.1 dB per
  step to 0x50. CCA on vs off at the same IGI: CCA on, the AGC compresses frame peaks 6–8 dB (p99.9
  1126 vs 2855 at 0x24), so CCA off holds the gain.
- **SoapySDR on the 8812CU** (`driver=wsdr,pid=0xc812`): 2.5 / 5 MS/s, gain 0–52 dB (1 dB), two
  coherent RX channels (0 = path A, 1 = path B). 20 s two-channel at 5 MS/s with no overflows, a live
  retune and gain change, and a re-open; offline 190 beacons on the grid, 0 slips
  (`evidence/phase5-cu-soapy.txt`).

### Gapped wide spans on the 8822C over USB 2 (2026-10-25)
- **Working:** `stream8822c.S` gapped mode (DMEM +0x20 = 1): arm the LA (0x7C0 = 0xCB), wait one bank
  time + guard, stop it (0x7C0 = 0), ship bank 0 (five packets + trailer, ~1480 µs), repeat. Every
  re-arm restarts the writer at bank 0: **zero stale banks** in every run (the descriptors and the
  sentinel are always overwritten). `wsdr_cu stream --gapped [--bw 40]`.
- Measured (8812CU, USB 2, ch36, 3-5 s each): 5 MS/s 310 banks/s (51 % of the time sampled); 20 MS/s
  508 banks/s (21 %); 40 MS/s 555 banks/s (11 %). Signal level matches continuous mode (RMS 203 vs 207).
- **The 8822C's RX filter follows the channel width** (unlike the 8822B, where devourer always writes
  the BW20 DFIR): 40 MS/s at BW20 is flat to ±9 MHz and −40 dB beyond ±12 MHz; at BW40 (centre ch38)
  the floor is flat from −17 to +17 MHz, rolling off at ±19 MHz, with no adjacent-duplicate samples.
  A real carrier at +7.7 MHz (5197.7 MHz, the one the 8812BU also shows) stands 31-32 dB above the floor
  on both paths. 20 MS/s at BW20: flat to ±8 MHz, −6 dB at ±9 MHz.
- **8812BU on USB 2, same mode** (`stream8822b.S`, DMEM +0x24; `wsdr_probe stream --gapped`): zero stale
  banks; 20 MS/s 516 banks/s (21 %), 40 MS/s (BW40 + vendor DFIR) 579 banks/s (12 %), 80 MS/s (BW80 +
  vendor DFIR) ~610 banks/s (6 %, `evidence/gapped-80msps.txt` §5); no centre notch on
  the 8822B (a DC spike instead, removed by the consumer's DC block); continuous 5 MS/s unchanged.
- Use: spectrum/waterfall over wide spans (FFT within one 8192-sample bank); not for continuous IQ.
- **The 8822B can notch its centre too** (2026-10-25, seen in wsdr-studio, not yet measured): 8812BU at
  USB 3, 40 MS/s continuous, 2.4 GHz (centre 2422 MHz): a deep, ~1 MHz wide dip at the centre. At 5 GHz
  (centre 5190) none was seen. Open: band/AGC dependence; if confirmed, offset tuning applies to the BU too.
  `Receiver` offers it only with `RxSettings::allow_gapped` and marks `BankMeta::gapped`.

### 5 GHz between the UNII-2 groups, ch 65..99 (2026-10-27)
- **Both chips tune and receive there (verified, one unit each, 40 MHz width).** Centres 5330..5490
  MHz: zero stale banks, the same band shape as in-table spans, the dongle-internal 40 MHz comb on
  the same bin from two tunings, and real carriers (the 5346 and 5494.72 MHz pixel-clock harmonics)
  where the earlier survey put them. `evidence/gap-ch65-99.txt`, plots `evidence/gap-*.png`.
- **8822B: the result depended on the previous channel, until devourer `5458e5a`.**
  devourer's vendor-derived per-channel values (AGC table `0x958`, CFO fc `0x860`, VCO band RF `0xBE`)
  had no entry for 65..99 (and RF 0xBE none for centrals 50/114/130/163), so such a channel kept the
  previous channel's values. Measured on one 8812BU (bus 6), same unit before and after: 5350 and 5450
  MHz read 3.6 / 4.5 dB apart depending on whether they were reached from 5180 or from 5510/5745
  (floor and fixed carriers alike, so a gain offset); with the fix 0.4 / 0.7 dB, inside the in-table
  control's 0.8 dB. The AGC table was what mattered: a stale RF 0xBE alone (5255/5815) showed nothing
  resolvable (below ~1.5 dB), so the RF 0xBE values devourer now writes there are untested either
  way. A +3 dB fresh-bring-up floor step at ch 82..98 also went away, but those sweeps only resolve
  ~2 dB.
- **8822C: no gap in devourer's tables** (contiguous `rf18_for` / `select_agc_tables` / `sco_for`);
  the fresh-bring-up floor is smooth across the gap (61.8..63.0), and retune history does not matter
  (0.1..0.4 dB spread by previous channel at 5 MS/s and 20 MHz width, the same as the in-table
  control).
- Not tested: the 8822B at 20 MHz and either chip at 80 MHz width in the gap, absolute sensitivity,
  the 8821C half of the fix.

### 8822B dual-path tap, dma_type 4 (2026-10-27)
- **One LA word carries both RX paths (verified, one 8812BU, 40 MS/s at BW40).** dma 4: **path A
  I/Q = [11:0]/[23:12], path B I/Q = [35:24]/[47:36]**, 12-bit each, post-DFIR, genuine 40 MS/s.
  Each half carries one ADC's fingerprint: [23:0] matches dma 0 (DC −17.1+6.3j vs −17.0+6.5j, the same
  floor and filter edge), [47:24] matches dma 1, which is therefore path B alone.
- **Two distinct antennas, coherent.** Over 3 s streamed, L-LTFs found on each path on its own: ch 36
  A 199 / B 232, 168 on both within 2 µs; ch 40 A 68 / B 145, 62 on both; 0 at the centre control and
  0 in conj(x) on either path. A/B coherence on 341 frames median 0.72, against 0.41 on noise
  stretches of the same lengths. The A−B level varies per frame (+1.1..+8.9 dB, p10..p90), so it is not
  one path copied: each antenna catches frames the other misses (ch 40: B twice A).
- **Same USB cost as one path.** 10 s each at 40 MS/s: dma 0 and dma 4 both 318.85 MB/s, 0 stale,
  0.8 µs worst lateness of a 184.8 µs budget. Recording needs two things at 40 MS/s (measured
  2026-10-27): 256 posted URBs (~51 banks, ~10.5 ms; with 32, ~1.3 ms, 30-s runs failed at 8.0 and
  13.9 s, by a missed deadline or a 168-byte packet in the bank stream), and no file writes on the USB thread (a
  write can sleep in dirty-page throttling: path A to disk missed the deadline in 4 of 4 3-s runs).
  `wsdr_probe stream` now copies each bank into a ring and writes from its own thread: both paths,
  3 × 30 s and 1 × 60 s, 0 faults.
- **All rates.** 80 MS/s (BW80): snapshots show L-LTFs on both paths at all four 20 MHz centres,
  none at the controls, filter edge ±37 MHz on both. 80 MS/s cannot stream (640 MB/s), but it runs
  **gapped**: 3 s, 9701 banks, 0 stale, 212 MB/s; each 102.4 µs bank is whole and ships in ~182 µs,
  so ~33 % of the time is captured. 20 MS/s at BW20: 3 s streamed, L-LTFs A 177 / B 127, 123 on both.
  Streaming at 20/10/5/2.5 MS/s and gapped at 40 MS/s: clean, 0 stale.
- **Path B leads path A by ~146 ns** (5.9 samples at 40 MS/s), from L-LTF channel estimates on frames
  found on both paths: −148 ns (ch 36 half, 82 frames) and −144 ns (ch 40 half, 24 frames), IQR about
  ±1 ns across transmitters. Delay B by that before combining (the 8822C needs ~370–400 ns).
- **Coherent where the filter sets the band.** Frame coherence median (p10) against noise stretches of
  the same lengths, median (p90), 6.4 µs segments: 40 MS/s at BW40 0.72 (0.33) vs 0.41 (0.45) and, in a
  second run, 0.64 (0.48) vs 0.44 (0.67); 20 MS/s at BW20 0.92 (0.83) vs 0.36 (0.44); gapped 40 MS/s
  0.64 (0.56) vs 0.38 (0.57); gapped 80 MS/s 0.77 (0.45) vs 0.41 (0.58) (each gapped bank is
  contiguous). The medians separate everywhere; at 40 and 80 MS/s the two distributions overlap.
  Below the filter's band the tap only selects words (no anti-alias filter): at 5 MS/s A and B are
  weakly coherent (0.18 vs 0.13 in noise), two valid receivers, not a coherent pair; 10 and 2.5 MS/s
  (and gapped at those rates) are the same mechanism, not measured.
- Counterparts: one unit; dma 5 matched dma 4 at 40 MS/s (layout, filters, skew); which
  connector is path A is unknown (soldered antennas). `evidence/dual-path-tap-dma4.txt`,
  `tools/bu_twopath.py`, `tools/bu_ab_delay.py`.

### Gain changes while streaming (8822B, 2026-11-20)
One unit (T4U v3, 2357:0115), USB 3, CCA off, ch 130 / 40 MHz. Levels are the noise floor (VTX off).
- **Register writes misbehave while the streamer runs.** A write completes without error, but about
  4 times in 5 the chip executes it with the data of the last control-IN transfer instead of the data
  sent: `m32(0xC50)` (read, then write) put the old IGI back in every one of 14 changes; a `w8` after a
  read of 0x838 (0x87746343) left 0x43 in 0xC50; two blind `w8`s landed in 2 of 5 changes. Reads were
  right every time. Before `cpu.start()` writes are reliable.
- **What works:** read a byte that already holds the wanted value, write, read back. `Stream8822b`
  keeps a table (byte i = i) at RX SRAM 0x18701800 with the debug window on that page, so register
  `0x8800 + v` reads `v`. Per IGI register (0xC50, 0xE50): table read, `w8`, `r8`. 1006 of 1006 changes
  at 50 a second took the minimum number of transfers, no fault, no stale bank.
- **One transfer per bank.** Host register traffic slows the CPU's shipping: `end_max_us` 182.8 with
  none, 184.0 with one transfer per bank (20 s, 250 transfers a second), against the 184.8 budget at
  40 MS/s. A write + read-back loop run inside one bank (about 5 transfers per bank) ended a bank
  186.2 us after due, once in 27 changes. Without the table, write + read back until it holds, one
  transfer per bank: no fault in 1002 changes, but a median 19 and up to 171 transfers per change.
- **Timing:** the level steps within one bank, about 3 banks after the change starts.
- **`Receiver::set`** with only `gain_db` changed no longer restarts an 8812BU's continuous fixed-gain
  stream (`StreamCfg::live_igi`). `wsdr_rxtest --gain 40 --gain-hop 30:100`: 191 changes in 20 s at
  40 MS/s, 0 restarts, quietest bank 16.1 / 6.2 LSB rms in every stretch; the same at 20 and 2.5 MS/s
  (58 and 34 changes). If the read-back fails 16 times the run ends with a fault and the restart sets
  the gain the old way (checked by pointing the table read at the wrong page: 4 restarts in 12 changes,
  stream up again each time).
- With an HDZero VTX at 3 m (5658 MHz): 8 s with a 6 dB step every second (IGI 0x43 <-> 0x3D); the
  kepyur-hd decoder found 479 frames, never lost lock, constellation error -20.4 to -20.7 dB in 100 ms
  windows across a step, as without one.
- Open: the 8822C (its stream still restarts), gapped mode (restarts), steps over 6 dB under a signal.
- `wsdr_probe stream --igi-hop N --hop-ms MS --hop-mode 0..7` is the experiment (7 = the method above).

### Unknowns (Phase 2 exit list)
1. Does dbg port 0x8fc = 0 give a live bus on *our* unit? devourer reports dbg_port 0 as dead and uses
   0x880 (`docs/la-capture.md`). Conflicting; test both. **Resolved (8822B):** both are live.
2. Does the free-running ring stay fresh with devourer's bring-up (DIG/watchdog threads touching BB regs)?
   **Resolved (8822B):** live, ≈1 % identical between reads. Long-run disturbance is not yet measured.
3. Gain: no manual AGC freeze path known (ac54 open item; devourer has IGI control — test pinning it).
   **Resolved (8822B):** the fixed-gain recipe above. The 8822C is untested.
4. The TX-SRAM→C2H address alias (bank bases 0x16000/0x6000) — empirical on one ASUS unit; confirm here.
   **Resolved (8822B):** both aliases deliver on our cut; the remaining blocker is the 16 KiB wedge.
5. Linux usbfs tail latency vs WinUSB RAW_IO — whether 20 Msps holds and where 40 Msps breaks.
   **Open:** the 16 KiB wedge is the immediate blocker; a USB2 run and a host-stack comparison are the
   next tests (handoff note). The measured on-chip latency is inside the 40 Msps window, so tail
   latency is not the current limiter.
6. Whether the MIPS16e2-only / 0x230 splice behaviour holds on our cut (same die/cut 3 as ac54: likely).
   **Partly resolved:** code spliced at 0x230 runs on our unit; the implemented MIPS16e2 subset is
   characterized above.
7. 8822C: custom-code entry without vectors (ROM), EMEM layout, C2H register/addresses, LA ring at
   0x20000..0x40000 (devourer: same window), the 7.4-sample A/B skew, USB-2 bandwidth.
8. BW40/80 taps (80/160 Msps?) and dma_type 4–15 formats (dual-path packing) — undecoded.
   **Partly resolved (8822B):** dma 4 is A + B, 12-bit each, in one word (above); BW40/80 taps in
   `notes/wide-tap.md`. dma 6–10 are still inferred only.
