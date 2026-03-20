# HardFault: Unaligned FPU Access on Cortex-M7

## Summary

A `HardFault` occurred on STM32H743 (Cortex-M7) when uploading magnetometer calibration data via UART. The root cause was a `float*` pointer cast on an unaligned byte buffer, triggering a `VLDR` (FPU load) instruction that requires 4-byte alignment.

**Impact**: Gyro and accel calibration appeared to work, masking the bug until compass calibration was tested.

## Symptom

- `calibration_mag.c` crashed with HardFault on every calibration upload
- `calibration_gyro.c` and `calibration_accel.c` worked fine with seemingly identical code
- The crash was **not** a logic error — the calibration math was correct

## Root Cause

### The struct layout

```c
typedef struct {
    uint8_t byte;          // offset 0 (1 byte)
    uint8_t buffer[128];   // offset 1 (UNALIGNED!)
    ...
} uart_rx_t;
```

`buffer` started at **offset 1** in the struct. When the UART parser dispatched `buffer` to PubSub subscribers, the `data` pointer was **not 4-byte aligned**.

### The dangerous cast

In `calibration_mag.c`:
```c
static void on_db_message(uint8_t *data, size_t size) {
    float *values = (float *)&data[4];  // data is at offset 1 → data[4] = offset 5
    double offset_x = (double)values[0]; // VLDR + VCVT → HardFault!
}
```

`&data[4]` pointed to offset 5 from the struct base — **not 4-byte aligned**. On Cortex-M7 with hardware FPU (`-mfpu=fpv5-d16`), the compiler generates `VLDR` to load the float into an FPU register. `VLDR` **requires 4-byte alignment** and raises a `UsageFault` (escalated to `HardFault`) on unaligned access.

### Why gyro/accel survived

In `calibration_gyro.c` and `calibration_accel.c`, the code used:
```c
float *values = (float *)&data[4];
float bias_x = values[0]; // float = float → compiler uses LDR (integer load)
```

`LDR` on Cortex-M7 supports unaligned access (it performs two bus cycles transparently). No FPU register involved → no crash.

Only `calibration_mag.c` did `double = (double)values[0]`, which forced the compiler to use `VLDR` (FPU load) + `VCVT` (float→double), triggering the alignment fault.

## Fix

### 1. Root cause fix — struct alignment (`platform_uart.c`)

```c
typedef struct {
    uint8_t byte;
    uint8_t _pad[3];       // ← aligns buffer to offset 4
    uint8_t buffer[128];   // now at 4-byte aligned offset
    ...
} uart_rx_t;
```

### 2. Defense-in-depth — memcpy in all calibration modules

```c
float values[12];
memcpy(values, &data[4], 12 * sizeof(float));  // always safe
double offset_x = (double)values[0];            // values[] is stack-aligned
```

Even though the struct fix resolves alignment, `memcpy` is the correct C idiom for extracting typed values from byte buffers. It avoids both alignment issues and strict aliasing violations.

## Rule: Never Cast Byte Buffer Pointers to Multi-Byte Types

```c
// ❌ DANGEROUS — alignment-dependent, strict aliasing violation
float *values = (float *)&data[4];

// ✅ SAFE — works regardless of alignment
float values[N];
memcpy(values, &data[4], N * sizeof(float));
```

This applies to **all** casts from `uint8_t*` (UART buffers, DMA buffers, network packets) to `float*`, `int32_t*`, `uint16_t*`, etc.

## Debugging Notes

- The crash address in the faulting `VLDR` instruction is the **only** clue — the data itself looks correct
- `SCB->CFSR` shows `UNALIGNED` bit set in `UsageFault` status register
- On Cortex-M7, `CCR.UNALIGN_TRP` can be set to trap all unaligned accesses (useful for finding these bugs proactively)
- GDB: `x/i $pc` at HardFault shows the faulting `VLDR` instruction

## Files Changed

| File | Change |
|------|--------|
| `base/boards/h7v1/platform/platform_uart.c` | Added `_pad[3]` to `uart_rx_t`; `memcpy` for `payload_size` |
| `modules/calibration/calibration_mag.c` | `float*` cast → `memcpy` |
| `modules/calibration/calibration_accel.c` | `float*` cast → `memcpy` |
| `modules/calibration/calibration_gyro.c` | `float*` cast → `memcpy` |
