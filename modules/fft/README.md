# FFT Module — On-Board Vibration Analysis & Dynamic Notch Filtering

## Overview

The FFT module performs **on-board** 256-point FFT vibration analysis on all three
gyro axes, detects dominant vibration peaks, and feeds them to the dynamic notch
filter module in real time. This enables adaptive motor vibration rejection without
ESC RPM telemetry.

An optional spectrum log stream sends compressed frequency data to a Python tool
for real-time spectrogram visualization.

## Architecture

```
ICM-42688P (1 kHz gyro)
    │
    ▼  SENSOR_IMU1_GYRO_UPDATE (1 kHz, 3 floats in deg/s)
 fft.c
    ├─ Collect in 256-sample ring buffer (per axis) [gyro ISR, ~1 µs]
    ├─ SCHEDULER_10HZ sets g_fft_pending flag [ISR, negligible]
    ├─ LOOP (main thread) runs 256-point Hanning-windowed FFT
    │   (cycles X → Y → Z, so ~3.3 Hz per axis)
    ├─ Normalize: power[i] = (re² + im²) / (N/2)²
    ├─ Detect top 2 peaks (50-200 Hz, SNR ≥ 8×, min sep 20 Hz)
    ├─ EMA smooth frequencies (α=0.3) for consecutive in-range peaks
    ├─ Reset to 0 when peak is out of range or not detected
    └─ publish(FFT_PEAKS_UPDATE)  →  notch_filter module
         │
         ▼  (optional, when Python tool requests)
      publish(SEND_LOG)  →  dblink  →  UART  →  Python tool
```

### ISR Safety

All `SCHEDULER_*` callbacks run inside a TIM ISR at priority 0 — they block
the 1 kHz control loop. A 256-point FFT is too expensive to run there.

The FFT module splits work into two parts:
- **ISR** (`SCHEDULER_10HZ`): sets a `volatile` flag (~1 instruction, negligible)
- **Main thread** (`LOOP`): checks the flag, runs the actual FFT computation

The 1 kHz timer ISR can preempt the FFT at any time, so the control loop
runs uninterrupted regardless of FFT computation time.

### Signal Flow to Dynamic Notch Filter

```
fft.c ──(FFT_PEAKS_UPDATE)──→ notch_filter.c
                                ├─ Subscribe SENSOR_IMU1_GYRO_UPDATE
                                ├─ Update notch center frequencies from peaks
                                ├─ Apply cascaded biquad notch filters (2 per axis)
                                └─ publish(SENSOR_IMU1_GYRO_FILTERED_UPDATE)
                                     │
                                     ▼
                                attitude_estimation.c (uses filtered gyro)
```

## FFT Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| FFT size | 256 samples | Radix-2 Cooley-Tukey DIT |
| Sample rate | 1000 Hz (`GYRO_FREQ`) | Defined in `macro.h` |
| Frequency resolution | ~3.91 Hz/bin | 1000/256 |
| Window | Hanning | Pre-computed at init |
| Analysis range | 50–400 Hz | Body motion below, motor vibration above |
| Spectrum display range | 0–400 Hz | 103 bins streamed to host |
| Update rate | 10 Hz total | Cycles X→Y→Z (~3.3 Hz/axis) when no axis selected; 10 Hz on selected axis when streaming spectrum/dual |

## Peak Detection

| Parameter | Value | Notes |
|-----------|-------|-------|
| Valid range | 50–400 Hz | Peaks outside this range keep the previous value (lock-and-hold) |
| SNR threshold | 5× mean power (tunable) | Peak must be 5× above average |
| Min separation | 40 Hz | Between two peaks on same axis |
| EMA alpha | 0.15 (tunable) | Smooths consecutive in-range peaks |
| Out-of-range | Reset to 0 | Notch filter disengages immediately |
| Re-acquisition | Snap to value | First valid peak after reset initializes instantly |

The FFT module always runs peak detection (no log class needed). It publishes
`fft_peaks_t` on `FFT_PEAKS_UPDATE` at ~3.3 Hz per axis. The notch filter module
subscribes and dynamically adjusts its center frequencies.

## Log Streaming (Optional)

When a Python tool requests a spectrum log class, the FFT module streams
compressed frequency data for visualization. Two modes:

### Peaks Only (LOG_CLASS_FFT_PEAKS, 0x17)

Streams all smoothed peaks across all 3 axes at 10 Hz:

```
Payload: 6 × float32 = 24 bytes
  [X_peak1] [X_peak2] [Y_peak1] [Y_peak2] [Z_peak1] [Z_peak2]
```

### Spectrum + Peaks (LOG_CLASS_FFT_SPECTRUM_X/Y/Z, 0x18–0x1A)

Streams compressed spectrum bins plus peak frequencies in a **single combined
frame** for the selected axis, at 10 Hz:

```
Payload: 1 + 103 + 8 = 112 bytes
  [axis (1 byte)] [103 × uint8 dB-scaled bins] [peak1 (float)] [peak2 (float)]
```

- **dB scaling**: Absolute, floor = -30 dB, range = 60 dB. Power is normalized
  (÷ (N/2)²) before `10·log10`. Mapped to 0–255 uint8.
- **Combined frame**: Peaks are appended in the same frame (not sent separately)
  to avoid UART buffer corruption — `HAL_UART_Transmit_IT` is non-blocking and
  dblink uses a single shared output buffer.

### Dual (Raw + Filtered) Spectrum (LOG_CLASS_FFT_SPECTRUM_DUAL_X/Y/Z, 0x14–0x16)

For verifying notch filter effectiveness. Runs the 256-point FFT **twice** per
tick on the selected axis — once on the raw gyro ring, once on the post-notch
ring (`SENSOR_IMU1_GYRO_FILTERED_UPDATE`) — and emits one combined frame at
10 Hz:

```
Payload: 1 + 103 + 103 + 8 + 8 = 223 bytes
  [axis (1)]
  [raw_bins (103 × uint8)]
  [filt_bins (103 × uint8)]
  [raw_peak1 (float)] [raw_peak2 (float)]   ← EMA-smoothed (locked)
  [filt_peak1 (float)] [filt_peak2 (float)] ← raw, un-smoothed
```

Filtered peaks are **not EMA-smoothed**: a working notch should make them drop
out / become weak, in which case `fft_find_peaks` returns 0 — useful as a
visual confirmation that the notch is biting.

### Bandwidth Budget (38400 baud ≈ 3840 B/s payload)

| Stream | DB-frame size | Rate | Throughput |
|--------|---------------|------|------------|
| Peaks only | 24 + 8 = 32 B | 10 Hz | 320 B/s (~8%) |
| Spectrum + peaks | 112 + 8 = 120 B | 10 Hz | 1200 B/s (~31%) |
| Dual (raw + filt) | 223 + 8 = 231 B | 10 Hz | 2310 B/s (~60%) |

(8-byte DB frame overhead: 2 header + 1 ID + 1 class + 2 length + 2 checksum)

## Python Tools

### fft_spectrum_view.py — Real-Time Spectrogram (raw)

Scrolling spectrogram with dynamic notch peak overlay traces:

```bash
python3 flight-controller/tools/fft_spectrum_view.py
```

- Select axis (X/Y/Z button) to begin spectrum streaming
- Spectrogram shows frequency content over time (color = magnitude)
- P1/P2 colored traces show detected peak frequencies

### fft_spectrum_dual_view.py — Raw vs Post-Notch Side-by-Side

Two stacked spectrograms (raw on top, filtered on bottom) sharing the same time
and frequency axes — the easiest way to verify the dynamic notch is killing the
motor harmonics:

```bash
python3 flight-controller/tools/fft_spectrum_dual_view.py
```

- Bright bands in the raw plot should disappear (or strongly attenuate) in the
  filtered plot at the same frequency
- P1/P2 readout on the filtered side reads `---` when the notch has driven the
  peak below the SNR threshold (= notch is working)

#### Reading the Dual View — Notch Health Check

| Bottom (filtered) plot looks like… | Diagnosis | Action |
|------------------------------------|-----------|--------|
| Bright bands **gone** at same freq as raw, filt P1/P2 read `---` | Notch is biting cleanly | Done |
| Bright bands **dimmed but still visible** (raw mag ~150 → filt mag ~120) | Notch is too narrow OR center frequency is lagging the real peak | Lower notch Q (wider bandwidth), and/or raise `fft_freq_alpha` (faster peak tracking) |
| Bright bands **unchanged**, filt P1/P2 still report the same peaks | Notch barely engaged — peak SNR threshold rejecting valid peaks, or notch coefficients not being updated | Lower `fft_peak_snr` (e.g. 5 → 3), confirm `notch_filter` is consuming `FFT_PEAKS_UPDATE` and recomputing biquad coefficients per update |
| Filtered plot is **noisier than raw** with wild peak hopping | Notch is moving so much it's injecting transients; smoothing alpha probably set too high or peaks are flickering above/below threshold | Stabilize: lower `fft_freq_alpha` slightly, raise SNR threshold so weak peaks don't flip the notch on/off |

The filtered P1/P2 lines are deliberately **not EMA-smoothed** — they show the
raw output of `fft_find_peaks()` on the post-notch signal. A working notch will
either drive the peak below the SNR threshold (→ `0` / `---`) or push the
"loudest" peak to a different, weaker frequency. Either is a good sign.

Tune these via `tuning_board.py` (no firmware rebuild needed). After every
parameter change, click "Upload Defaults" → reboot, otherwise stale flash
values can override the runtime params (see `docs/TUNING_PARAM_ZERO_BUG.md`).

| Display Feature | Description |
|-----------------|-------------|
| Spectrogram | Scrolling waterfall, ~30s history, inferno colormap |
| P1/P2 traces | Dynamic notch peak frequencies overlaid on spectrogram |
| Peak readout | Current P1/P2 frequencies displayed as text |

#### What to Look For

| Pattern | Meaning |
|---------|---------|
| Horizontal bright line | Constant vibration frequency (e.g., idle RPM) |
| Rising diagonal line | Frequency increasing with throttle/RPM |
| Bright spot at specific time | Resonance at a particular throttle setting |
| Broadband vertical stripe | Impulse event (bump, landing, tap) |
| Diffuse high-frequency glow | Bearing noise or loose hardware |

## Files

| File | Purpose |
|------|---------|
| `modules/fft/fft.h` | Module header — exposes `fft_setup()` |
| `modules/fft/fft.c` | 256-point FFT, peak detection, spectrum streaming |
| `modules/notch_filter/notch_filter.c` | Dynamic notch filter (subscribes FFT_PEAKS_UPDATE) |
| `tools/fft_spectrum_view.py` | Real-time spectrogram + peak overlay tool (single axis, raw) |
| `tools/fft_spectrum_dual_view.py` | Raw + post-notch spectrograms stacked — notch effectiveness viewer |
| `base/foundation/messages.h` | `fft_peaks_t`, `LOG_CLASS_FFT_*` definitions |
