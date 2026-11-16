#!/usr/bin/env python3
"""Cabled-bench TX supervisor for wsdr_txprobe.

Validates lab/test-setup.conf and lab/allowed_channels.txt, takes the shared
8812bu coordination lock, starts the RX capture FIRST, then runs the TX probe
under a hard timeout, and always resets the TX adapter afterwards (including on
crash or Ctrl-C).  Saves every command and its output under tx_research/logs/
and the capture plus a spectrum/envelope summary under tx_research/captures/.

Examples:
  ./tx_research/tx_run.py cw --channel 36 --gain 0 --seconds 3 --tag cw-g0
  ./tx_research/tx_run.py ook --channel 36 --gain 4 --seconds 3 \
      --pattern 1010 --rate-hz 1000 --tag ook-lo-1k
  ./tx_research/tx_run.py cont --channel 36 --seconds 2 --tag cont
  ./tx_research/tx_run.py raw --channel 36 --seconds 2 \
      --bb 0x914=0x70000:0x10000 --tag rawsinglecarrier
  ./tx_research/tx_run.py cw --channel 36 --dry-run
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import NoReturn

ROOT = Path(__file__).resolve().parent.parent
PROBE = ROOT / "build-opencode" / "wsdr_probe"
TXPROBE = ROOT / "build-opencode" / "wsdr_txprobe"
CONF = ROOT / "lab" / "test-setup.conf"
ALLOWED = ROOT / "lab" / "allowed_channels.txt"
CAPS = ROOT / "tx_research" / "captures"
LOGS = ROOT / "tx_research" / "logs"
REGS = ROOT / "tx_research" / "regs"
COORD = ROOT / "tools" / "coord.sh"

TX_PID = 0xB812
RX_BUS, RX_PORT = 6, "1"


def die(msg) -> NoReturn:
    print(f"ABORT: {msg}", file=sys.stderr)
    sys.exit(2)


def parse_conf(path):
    cfg = {}
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        k, v = line.split("=", 1)
        cfg[k.strip()] = v.strip()
    return cfg


def parse_adapter(s):
    m = re.search(r"Bus\s+0*(\d+)\.Port\s+([0-9.]+)", s)
    if not m:
        raise ValueError(f"cannot parse tx_adapter: {s}")
    port = ".".join(str(int(p)) for p in m.group(2).split(".") if p)
    return int(m.group(1)), port


def load_lab():
    if not CONF.exists():
        die(f"{CONF} missing")
    cfg = parse_conf(CONF)
    if cfg.get("cabled", "").lower() not in ("yes", "1", "true"):
        die("lab/test-setup.conf is not marked cabled=yes")
    try:
        atten = int(cfg.get("attenuation_db", "0"))
    except ValueError:
        die("attenuation_db is not an integer")
    if atten <= 0:
        die("attenuation_db must be > 0")
    if not cfg.get("tx_paths", "").strip():
        die("tx_paths is empty")
    try:
        tx_bus, tx_port = parse_adapter(cfg.get("tx_adapter", ""))
    except ValueError as e:
        die(str(e))
    chans = set()
    for raw in ALLOWED.read_text().splitlines():
        for tok in raw.split("#", 1)[0].replace(",", " ").split():
            if tok.isdigit():
                chans.add(int(tok))
    if not chans:
        die(f"{ALLOWED} has no channel numbers")
    for bus, port, what in ((tx_bus, tx_port, "TX"), (RX_BUS, RX_PORT, "RX")):
        dev = Path(f"/sys/bus/usb/devices/{bus}-{port}")
        if not dev.exists():
            die(f"{what} adapter {bus}-{port} not present in sysfs")
        try:
            vid = (dev / "idVendor").read_text().strip()
            pid = (dev / "idProduct").read_text().strip()
        except OSError as e:
            die(f"cannot read {dev}: {e}")
        # 2357:0138 = TP-Link Archer T3U Plus, an OEM 8812BU (src/session.h kUsbAliases)
        if f"{vid}:{pid}" not in ("0bda:b812", "2357:0138"):
            die(f"{what} adapter {bus}-{port} is {vid}:{pid}, expected an 8812BU (0bda:b812)")
    return {"tx_bus": tx_bus, "tx_port": tx_port, "atten": atten,
            "tx_paths": cfg["tx_paths"], "channels": chans, "cfg": cfg}


def coord_lock(tag):
    r = subprocess.run([str(COORD), "lock", "8812bu", "opencode", f"tx research {tag}"],
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        die(f"coordination lock busy: {r.stdout.strip()} {r.stderr.strip()}")
    return True


def coord_unlock():
    subprocess.run([str(COORD), "unlock", "8812bu", "opencode"], cwd=ROOT,
                   capture_output=True, text=True)


def reset_tx(bus, port, log):
    cmd = [str(TXPROBE), "reset", "--pid", hex(TX_PID), "--bus", str(bus), "--port", port]
    log.write(f"$ {' '.join(cmd)}\n")
    log.flush()
    try:
        r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=15)
        log.write(r.stdout + r.stderr)
        log.flush()
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        log.write("reset: TIMEOUT\n")
        log.flush()
        return False


def _spectrum(x, fs, nperseg=65536, max_segments=40):
    import numpy as np

    if len(x) < nperseg:
        return None
    nseg = min(max_segments, len(x) // nperseg)
    win = np.hanning(nperseg).astype(np.float32)
    psd = np.zeros(nperseg, dtype=np.float64)
    for k in range(nseg):
        seg = x[k * nperseg:(k + 1) * nperseg]
        seg = (seg - seg.mean()) * win
        psd += np.abs(np.fft.fftshift(np.fft.fft(seg))) ** 2
    psd /= max(nseg, 1)
    freqs = np.fft.fftshift(np.fft.fftfreq(nperseg, 1.0 / fs))
    pdb = 10 * np.log10(psd + 1e-12)
    # Ignore the exact DC bins: RX DC offset lives there.
    dc = np.abs(freqs) <= 3 * fs / nperseg
    idx = int(np.argmax(np.where(dc, -1e30, pdb)))
    floor = float(np.median(pdb[~dc]))
    res = {"nperseg": nperseg, "segments": int(nseg),
           "peak_hz": float(freqs[idx]), "peak_db": float(pdb[idx]),
           "floor_db": floor, "snr_db": float(pdb[idx] - floor),
           "dc_db": float(pdb[dc].max())}
    lin = np.maximum(psd - 10 ** (floor / 10), 0)
    if lin.sum() > 0:
        cum = np.cumsum(lin) / lin.sum()
        lo = int(np.searchsorted(cum, 0.005))
        hi = int(np.searchsorted(cum, 0.995))
        res["bw99_hz"] = float(freqs[hi] - freqs[lo])
    return res


def _longest_run(mask):
    import numpy as np

    best = (0, 0)
    start = None
    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        if start is not None and (not v or i == len(mask) - 1):
            end = i if not v else i + 1
            if end - start > best[1] - best[0]:
                best = (start, end)
            start = None
    return best


def analyze(cap_prefix, mode, tx_seconds, summary_path, pattern="", rate_hz=0.0,
            analyze_start=0.0):
    import numpy as np

    meta_path = Path(str(cap_prefix) + ".json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    fs = float(meta.get("sample_rate", 2.5e6))
    raw = np.fromfile(str(cap_prefix) + ".cs16", dtype=np.int16)
    if raw.size < 4096:
        return {"ok": False, "error": "capture too short", "samples": int(raw.size // 2)}
    raw = raw[: (raw.size // 2) * 2].reshape(-1, 2)
    # Keep the analysis bounded; low-rate captures use the whole run so the
    # pre/post-TX baseline is present, high-rate ones are capped at 2 s.
    max_seconds = 6.0 if fs <= 5e6 else 2.0
    if analyze_start > 0:
        raw = raw[int(analyze_start * fs):]
    raw = raw[: int(min(len(raw), fs * max_seconds))]
    x = (raw[:, 0].astype(np.float32) + 1j * raw[:, 1].astype(np.float32)) / 16.0

    out = {"ok": True, "format": "cs16", "fs": fs, "channel": meta.get("channel"),
           "samples": int(len(x)), "seconds": len(x) / fs, "tx_seconds": tx_seconds,
           "analysis_start_s": analyze_start}
    out["spectrum"] = _spectrum(x, fs)

    # Slice-by-slice amplitude and peak frequency, as inspectable evidence.
    sl = max(1, int(fs * 0.1))
    ns = (len(x) // sl) * sl
    slices = []
    for k in range(0, ns - sl + 1, sl):
        seg = x[k:k + sl]
        sp = _spectrum(seg, fs, nperseg=min(16384, sl), max_segments=1)
        slices.append({"t_s": round(analyze_start + k / fs, 3),
                       "rms": round(float(np.sqrt(np.mean(np.abs(seg) ** 2))), 2),
                       "peak_hz": round(sp["peak_hz"], 1) if sp else None,
                       "peak_db": round(sp["peak_db"], 2) if sp else None})
    out["time_slices"] = slices

    # Envelope after removing the complex mean (kills the RX DC offset).
    y = np.abs(x - x.mean())
    win_s = min(1e-4, 0.2 / rate_hz) if mode == "ook" and rate_hz > 0 else 0.005
    nwin = max(1, int(fs * win_s))
    n = (len(y) // nwin) * nwin
    env = y[:n].reshape(-1, nwin).mean(axis=1)
    env_t = analyze_start + np.arange(len(env)) * (nwin / fs)
    with open(str(cap_prefix) + "-envelope.csv", "w") as f:
        f.write("t_s,rms\n")
        for t, v in zip(env_t, env):
            f.write(f"{t:.6f},{v:.3f}\n")

    base = float(np.percentile(env, 10)) if env.size else 0.0
    top = float(np.percentile(env, 99)) if env.size else 0.0
    if top <= base * 1.5:
        out["burst"] = {"detected": False, "baseline_rms": round(base, 2),
                        "peak_rms": round(top, 2)}
        return out
    thr = max(base * 1.5, (base + top) / 2)
    on = env > thr
    if mode == "ook":
        # Find the keyed interval with a coarse envelope, then measure the
        # toggling rate inside it (not over the whole capture, which includes
        # the pre-roll and the post-TX baseline).
        cwin = max(1, int(fs * 0.01))
        cn = (len(y) // cwin) * cwin
        coarse = y[:cn].reshape(-1, cwin).mean(axis=1)
        cbase = float(np.percentile(coarse, 10)) if coarse.size else 0.0
        ctop = float(np.percentile(coarse, 99)) if coarse.size else 0.0
        if ctop > cbase * 1.5:
            ca, cb = _longest_run(coarse > max(cbase * 1.5, (cbase + ctop) / 2))
            tx_start, tx_end = ca * cwin / fs, cb * cwin / fs
        else:
            tx_start, tx_end = 0.0, len(y) / fs
        i0, i1 = int(tx_start / win_s), int(tx_end / win_s)
        i1 = min(i1, len(on))
        seg = on[i0:i1]
        trans = int(np.count_nonzero(seg[1:] != seg[:-1])) if seg.size else 0
        duration = max((i1 - i0) * win_s, 1e-9)
        toggles_per_pattern = sum(pattern[i] != pattern[(i + 1) % len(pattern)]
                                  for i in range(len(pattern))) if pattern else 0
        expected = rate_hz * toggles_per_pattern / len(pattern) if pattern else 0.0
        lo, hi = np.percentile(env, [5, 95])
        on_levels = env[i0:i1][seg]
        off_levels = env[i0:i1][~seg]
        burst_x = x[int(tx_start * fs):int(tx_end * fs)]
        out["envelope"] = {
            "window_samples": nwin,
            "low": round(float(lo), 2), "high": round(float(hi), 2),
            "ratio": round(float(hi / max(lo, 1e-9)), 2),
            "tx_start_s": round(float(tx_start), 3),
            "tx_end_s": round(float(tx_end), 3),
            "tx_duration_s": round(float(duration), 4),
            "on_fraction_in_burst": round(float(seg.mean()), 4) if seg.size else 0.0,
            "transitions_in_burst": trans,
            "measured_toggle_hz": round(float(trans / duration), 1),
            "expected_toggle_hz": round(float(expected), 1),
            "on_rms_median": round(float(np.median(on_levels)), 2) if on_levels.size else 0.0,
            "off_rms_median": round(float(np.median(off_levels)), 2) if off_levels.size else 0.0,
            "burst_spectrum": _spectrum(burst_x, fs),
        }
        return out

    a, b = _longest_run(on)
    if b <= a:
        out["burst"] = {"detected": False, "baseline_rms": round(base, 2),
                        "peak_rms": round(top, 2)}
        return out
    on0, on1 = a * nwin, b * nwin
    on_x = x[on0:on1]
    off_x = np.concatenate([x[:on0], x[on1:]])
    rms_on = float(np.sqrt(np.mean(np.abs(on_x) ** 2)))
    rms_off = float(np.sqrt(np.mean(np.abs(off_x) ** 2))) if off_x.size else 0.0
    on_spec = _spectrum(on_x, fs)
    off_spec = _spectrum(off_x, fs)
    peak_positions = [s["peak_hz"] for s in slices
                      if s["peak_hz"] is not None and on0 / fs <= s["t_s"] < on1 / fs]
    out["burst"] = {
        "detected": True,
        "start_s": round(float(on0 / fs), 4),
        "end_s": round(float(on1 / fs), 4),
        "duration_s": round(float((on1 - on0) / fs), 4),
        "baseline_rms": round(rms_off, 2),
        "on_rms": round(rms_on, 2),
        "delta_db": round(float(20 * np.log10(max(rms_on, 1e-9) / max(rms_off, 1e-9))), 2),
        "on_fraction": round(float(on.mean()), 4),
        "on_spectrum": on_spec,
        "off_spectrum": off_spec,
        "peak_drift_hz": round(float(max(peak_positions) - min(peak_positions)), 1)
        if peak_positions else None,
    }
    return out


def main():
    ap = argparse.ArgumentParser(description="cabled-bench TX supervisor")
    ap.add_argument("mode", choices=["cw", "cont", "ook", "raw", "tone", "fw914sweep", "fwfsk", "fsk"])
    ap.add_argument("--channel", type=int, default=36)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--gain", type=int, default=0)
    ap.add_argument("--pattern", default="1010")
    ap.add_argument("--rate-hz", type=float, default=1000.0)
    ap.add_argument("--ook-src", choices=["lo", "gain"], default="lo")
    ap.add_argument("--tone-index", type=int, default=0x08)
    ap.add_argument("--tone-path", type=int, choices=[0, 1], default=0)
    ap.add_argument("--tone-repeat-ms", type=float, default=0.0)
    ap.add_argument("--tone-full", action="store_true")
    ap.add_argument("--fsk-f1", type=int, default=8)
    ap.add_argument("--fsk-f2", type=int, default=16)
    ap.add_argument("--fsk-dwell-us", type=int, default=1000)
    ap.add_argument("--fsk-cycles", type=int, default=0)
    ap.add_argument("--fsk-amp", type=int, default=0)
    ap.add_argument("--bb", action="append", default=[])
    ap.add_argument("--rf", action="append", default=[])
    ap.add_argument("--tx-extra", default="", help="extra wsdr_txprobe args (shell-split)")
    ap.add_argument("--rx-rate", type=int, default=None,
                    help="LA rate selector; default 5 for cw/raw/ook, 1 for cont")
    ap.add_argument("--rx-bw", type=int, default=None)
    ap.add_argument("--rx-dfir", type=int, default=None)
    ap.add_argument("--rx-igi", type=int, default=None)
    ap.add_argument("--tx-power", type=int, default=None)
    ap.add_argument("--preroll", type=float, default=1.0,
                    help="seconds of RX baseline captured before TX is armed")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-analyze", action="store_true")
    ap.add_argument("--analyze-start", type=float, default=0.0,
                    help="skip this many seconds before analysing the capture")
    args = ap.parse_args()

    lab = load_lab()
    if args.channel not in lab["channels"]:
        die(f"channel {args.channel} not in allowed channels {sorted(lab['channels'])}")
    if not (0.1 <= args.seconds <= 10.0):
        die("--seconds must be within 0.1..10")
    tag = args.tag or time.strftime("%Y%m%d-%H%M%S") + f"-{args.mode}-ch{args.channel}"
    CAPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)
    REGS.mkdir(parents=True, exist_ok=True)
    cap = CAPS / f"{tag}-rx"
    reglog = REGS / f"{tag}.jsonl"
    runlog = LOGS / f"{tag}.log"
    summary_path = CAPS / f"{tag}-summary.json"

    if args.rx_rate is None:
        args.rx_rate = 1 if args.mode == "cont" else (2 if args.mode in ("tone", "fw914sweep", "fwfsk", "fsk") else 5)
    if args.rx_bw is None:
        args.rx_bw = 40 if args.mode in ("cont", "tone", "fw914sweep", "fwfsk", "fsk") else 20
    if args.rx_dfir is None:
        args.rx_dfir = 40 if args.mode in ("cont", "tone", "fw914sweep", "fwfsk", "fsk") else 20

    ready_file = LOGS / f"{tag}.ready"
    tx_cmd = [str(TXPROBE), args.mode, "--pid", hex(TX_PID),
              "--bus", str(lab["tx_bus"]), "--port", lab["tx_port"],
              "--channel", str(args.channel), "--seconds", str(args.seconds),
              "--log", str(reglog), "--wait-stdin", "--ready-file", str(ready_file)]
    if args.mode in ("cw", "ook"):
        tx_cmd += ["--gain", str(args.gain)]
    if args.mode == "ook":
        tx_cmd += ["--pattern", args.pattern, "--rate-hz", str(args.rate_hz),
                   "--ook-src", args.ook_src]
    if args.mode == "tone":
        tx_cmd += ["--tone-index", str(args.tone_index),
                   "--tone-path", str(args.tone_path),
                   "--tone-repeat-ms", str(args.tone_repeat_ms)]
        if args.tone_full:
            tx_cmd += ["--tone-full"]
    if args.mode == "fwfsk":
        tx_cmd += ["--fsk-f1", str(args.fsk_f1), "--fsk-f2", str(args.fsk_f2),
                   "--fsk-dwell-us", str(args.fsk_dwell_us),
                   "--fsk-cycles", str(args.fsk_cycles),
                   "--fsk-amp", str(args.fsk_amp)]
    if args.mode == "fsk":
        tx_cmd += ["--fsk-f1", str(args.fsk_f1), "--fsk-f2", str(args.fsk_f2),
                   "--rate-hz", str(args.rate_hz)]
    for op in args.bb:
        tx_cmd += ["--bb", op]
    for op in args.rf:
        tx_cmd += ["--rf", op]
    if args.tx_power is not None:
        tx_cmd += ["--tx-power", str(args.tx_power)]
    if args.tx_extra:
        tx_cmd += args.tx_extra.split()

    rx_seconds = args.seconds + args.preroll + 2.0
    rx_cmd = [str(PROBE), "stream", "--pid", hex(TX_PID),
              "--bus", str(RX_BUS), "--port", RX_PORT,
              "--channel", str(args.channel), "--bw", str(args.rx_bw),
              "--dfir", str(args.rx_dfir), "--rate", str(args.rx_rate),
              "--seconds", str(rx_seconds), "--out", str(cap)]
    if args.rx_igi is not None:
        rx_cmd += ["--igi", str(args.rx_igi)]

    print(f"bench: TX {lab['tx_bus']}-{lab['tx_port']} -> {lab['atten']} dB -> "
          f"RX {RX_BUS}-{RX_PORT} ({lab['tx_paths']}); channel {args.channel}")
    print(f"RX : {' '.join(rx_cmd)}")
    print(f"TX : {' '.join(tx_cmd)}")
    if args.dry_run:
        print("dry run: no lock, no capture, no TX")
        return 0
    for p in (PROBE, TXPROBE):
        if not p.exists():
            die(f"{p} missing (build it first)")

    if not coord_lock(tag):
        return 2
    rx = None
    tx = None
    rc = 1
    try:
        with open(runlog, "w") as log:
            log.write(f"tag: {tag}\nbench: TX {lab['tx_bus']}-{lab['tx_port']} "
                      f"atten={lab['atten']} paths={lab['tx_paths']} channel={args.channel}\n")
            ready_file.unlink(missing_ok=True)
            log.write(f"$ {' '.join(tx_cmd)}\n")
            log.flush()
            tx = subprocess.Popen(tx_cmd, cwd=ROOT, stdin=subprocess.PIPE,
                                  stdout=log, stderr=subprocess.STDOUT)
            # Bring-up happens before RX starts and before any TX is keyed.
            deadline = time.time() + 90.0
            while time.time() < deadline:
                if tx.poll() is not None:
                    die(f"TX bring-up exited early (rc={tx.returncode}); see {runlog}")
                if ready_file.exists():
                    break
                time.sleep(0.2)
            if not ready_file.exists():
                die(f"TX bring-up did not become ready; see {runlog}")
            log.write("# TX brought up and waiting; starting RX\n")
            log.flush()
            log.write(f"$ {' '.join(rx_cmd)}\n")
            log.flush()
            rx = subprocess.Popen(rx_cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            # The receiver's own bring-up takes seconds; wait until it is
            # actually writing samples before any TX is enabled.
            cs16 = Path(str(cap) + ".cs16")
            deadline = time.time() + 30.0
            while time.time() < deadline:
                if rx.poll() is not None:
                    die(f"RX capture exited before TX (rc={rx.returncode}); see {runlog}")
                if cs16.exists() and cs16.stat().st_size >= 1_000_000:
                    break
                time.sleep(0.1)
            if not cs16.exists() or cs16.stat().st_size < 1_000_000:
                rx.terminate()
                die(f"RX capture not producing samples; refusing to TX (see {runlog})")
            log.write(f"# RX running, {cs16.stat().st_size} bytes; "
                      f"collecting {args.preroll:.1f}s baseline\n")
            log.flush()
            time.sleep(max(0.0, args.preroll))
            log.write("# arming TX\n")
            log.flush()
            # Release the probe only after the receiver has produced samples.
            assert tx.stdin is not None
            tx.stdin.write(b"\n")
            tx.stdin.flush()
            try:
                tx.wait(timeout=args.seconds + 8)
            except subprocess.TimeoutExpired:
                log.write("TX: TIMEOUT, killing\n")
                log.flush()
                tx.kill()
                tx.wait()
            log.write(f"# TX rc={tx.returncode}\n")
            log.flush()
            try:
                rx.wait(timeout=rx_seconds + 10)
            except subprocess.TimeoutExpired:
                log.write("RX: TIMEOUT, killing\n")
                log.flush()
                rx.kill()
                rx.wait()
            log.write(f"# RX rc={rx.returncode}\n")
            log.flush()
    except SystemExit:
        raise
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        for p in (tx, rx):
            if p is not None and p.poll() is None:
                p.terminate()
    finally:
        # Stop any probe first, then reset the adapter: never leave it keyed,
        # even after a crash or Ctrl-C.
        for p in (tx, rx):
            if p is not None and p.poll() is None:
                p.kill()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    pass
        with open(runlog, "a") as log:
            ok = reset_tx(lab["tx_bus"], lab["tx_port"], log)
            log.write(f"# TX reset {'ok' if ok else 'FAILED'}\n")
        coord_unlock()

    if tx is not None and tx.returncode == 0 and not args.no_analyze:
        try:
            summary = analyze(cap, args.mode, args.seconds, summary_path,
                              args.pattern, args.rate_hz,
                              analyze_start=args.analyze_start)
        except Exception as e:  # analysis must never affect the safety path
            summary = {"ok": False, "error": f"analysis failed: {e}"}
        summary["tag"] = tag
        summary["mode"] = args.mode
        summary["channel"] = args.channel
        summary["rx_command"] = " ".join(rx_cmd)
        summary["tx_command"] = " ".join(tx_cmd)
        summary_path.write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2))
        rc = 0 if summary.get("ok") else 1
    else:
        print(f"TX failed or analysis disabled; see {runlog}", file=sys.stderr)
        rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
