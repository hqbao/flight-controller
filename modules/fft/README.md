# FFT Module — Gyro Vibration Analysis

## Overview

The FFT module streams gyro data from the flight controller to a host machine
for real-time frequency spectrum analysis. This helps identify motor vibrations,
frame resonance, prop imbalance, and PID oscillation — without any on-board FFT
computation (planned for a future version).

## Architecture

```
ICM-42688P (4 kHz ODR)
    │
    ▼  SCHEDULER_1KHZ
 imu.c (read sensor, calibrate, publish)
    │
    ▼  SENSOR_IMU1_GYRO_UPDATE (1 kHz, 3 floats in deg/s)
 fft.c
    ├─ Decimate 1 kHz → 250 Hz (every 4th sample)
    ├─ Scale float → int16 (×10, 0.1°/s resolution)
    ├─ Batch 50 samples into buffer
    └─ publish(SEND_LOG) when batch full
         │
         ▼  SEND_LOG
      logger.c (wrap in DB frame, send UART)
         │
         ▼  UART 9600 baud
      Python tool (parse, accumulate, FFT, display)
```

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **int16 instead of float** | 2 bytes vs 4 bytes per sample — halves bandwidth |
| **×10 scaling** | 0.1°/s resolution, ±3276°/s range — more than enough |
| **Batch 50 samples** | Amortizes 8-byte DB frame overhead (94% payload efficiency) |
| **Decimate to 250 Hz** | 125 Hz Nyquist covers frame resonance and PID oscillation |
| **Gyro Z axis only** | Most sensitive to prop/motor vibration on flat quad frame |

### Bandwidth Budget (9600 baud = 960 B/s)

| Parameter | Value |
|-----------|-------|
| Gyro effective rate | 250 Hz (1 kHz ÷ 4) |
| Samples per batch | 50 × int16 = 100 bytes |
| DB frame overhead | 8 bytes (header 6 + checksum 2) |
| Frame size | 108 bytes |
| Frame rate | 250 ÷ 50 = **5 fps** |
| Throughput | 5 × 108 = **540 B/s (56% utilization)** |

## Runtime Activation

The FFT stream is activated at runtime via the Python tool — **no firmware
recompilation needed**. The tool sends a `DB_CMD_LOG_CLASS` command with
`LOG_CLASS_IMU_GYRO` (0x05) over UART. The logger broadcasts this via
`NOTIFY_LOG_CLASS`, and the FFT module activates its gyro collection.

Only one log class is active at a time. Starting FFT streaming automatically
stops any other active log stream (attitude, compass, position, etc.).

## Wire Format

Each DB frame payload contains 50 × int16 (100 bytes, little-endian):

```
DB Frame:
  'd' 'b' [0x00] [0x05] [0x64 0x00] [int16 × 50] [checksum LE]
  ─────── ────── ────── ────────── ─────────────── ─────────────
  header   ID    class   len=100    payload         checksum

Each int16 = gyro Z in 0.1°/s units.
To convert: deg_per_sec = int16_value / 10.0
```

## Python Tools

### fft_view.py — FFT Spectrum Viewer

Real-time dual-panel display:
- **Top**: Time-domain gyro Z signal (°/s)
- **Bottom**: Hanning-windowed FFT magnitude spectrum (0–125 Hz)

```bash
python3 flight-controller/tools/fft_view.py
```

| Parameter | Value |
|-----------|-------|
| FFT size | 512 samples (~2 seconds) |
| Window | Hanning |
| Frequency resolution | 250 / 512 ≈ 0.49 Hz |
| Update rate | ~7 Hz (150ms interval) |

### fft_spectrogram.py — Waterfall Spectrogram

Scrolling spectrogram showing frequency content over time:
- **Top**: Time-domain signal (latest window)
- **Bottom**: Color-mapped spectrogram (X=time, Y=frequency, Color=magnitude)

```bash
python3 flight-controller/tools/fft_spectrogram.py
```

| Parameter | Value |
|-----------|-------|
| FFT size | 256 samples (~1 second) |
| Hop size | 50 samples (200ms per column) |
| Display | ~30 seconds of history |
| Colormap | Inferno |

#### What to Look For

| Pattern | Meaning |
|---------|---------|
| Horizontal bright line | Constant vibration frequency (e.g., idle RPM) |
| Rising diagonal line | Frequency increasing with throttle/RPM |
| Bright spot at specific time | Resonance at a particular throttle setting |
| Broadband vertical stripe | Impulse event (bump, landing, tap) |
| Diffuse high-frequency glow | Bearing noise or loose hardware |

## Usage

1. **Flash firmware** (FFT module is always compiled in, just inactive):
   ```bash
   cd flight-controller/base/boards/h7v1
   ./build-flash.sh
   ```

2. **Run a viewer**:
   ```bash
   python3 flight-controller/tools/fft_view.py
   # or
   python3 flight-controller/tools/fft_spectrogram.py
   ```

3. **Click "Start Log"** in the GUI to begin streaming.

4. **Spin up motors** and observe the spectrum/spectrogram.

5. **Click "Stop Log"** when done (or close the window).

## Files

| File | Purpose |
|------|---------|
| `modules/fft/fft.h` | Module header — exposes `fft_setup()` |
| `modules/fft/fft.c` | Gyro collection, decimation, batching, SEND_LOG |
| `tools/fft_view.py` | FFT spectrum viewer (time + frequency domain) |
| `tools/fft_spectrogram.py` | Scrolling waterfall spectrogram |
| `base/foundation/messages.h` | `LOG_CLASS_IMU_GYRO` (0x05) definition |

## Future Enhancements

- **On-board FFT**: Compute FFT on STM32 and send frequency bins instead of
  raw samples — reduces bandwidth and enables on-board notch filter tuning.
- **Multi-axis**: Stream all 3 gyro axes (would require higher baud rate or
  reduced sample rate).
- **Auto notch filter**: Detect dominant vibration peaks and automatically
  configure notch filters in the attitude estimation pipeline.
