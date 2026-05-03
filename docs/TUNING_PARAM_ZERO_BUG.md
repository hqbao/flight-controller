# Tuning Param Zero Bug — Attitude `v_true` Frozen at (0, 0, -1)

## Summary

When the flight controller boots with stored tuning parameters of value `0.0` for must-be-positive fields (e.g., `att_accel_smooth`, `att_f3_beta`, `att_f3_zeta`), the attitude estimator silently breaks. The accelerometer fusion stops updating, leaving `v_true` (measured gravity) frozen at its init value `(0, 0, -1)`. The drone appears level no matter how it's tilted.

## Symptom

- In CubeIDE Live Expressions: `g_f11.v_true = (0, 0, -1)` constant — even when the drone is rotated.
- `g_f11.a` (raw accel) DOES update with motion → so accel data is reaching the fusion.
- Quaternion `g_f11.q` and `v_pred` update fine from gyro integration.
- `attitude_view.py` shows the blue "true gravity" vector frozen.
- Other flight controllers (with valid tuning flash) behave normally with the same firmware.

## Root Cause

`g_f11.k0`, `g_f11.beta`, `g_f11.zeta` all become `0.0` at runtime, overwriting the compiled defaults (`4.0`, `0.001`, `0.0001`).

With `k0 = 0`, the accel low-pass filter inside `fusion3_update` becomes a no-op:

```c
// robotkit/fusion3.c
vector3d_follow(&f->a_smooth, &f->a, f->k0 * dt);  // 0 * dt = 0 → no update
vector3d_normalize(&f->v_true, &f->a_smooth);       // stuck at init (0, 0, -accel_scale)
```

`a_smooth` never moves from its init value, so `v_true = normalize(a_smooth) = (0, 0, -1)` forever — regardless of what the accelerometer actually measures. The Madgwick correction step also becomes useless because `beta = 0` and `zeta = 0`.

## How the Zeros Get In

The boot-time tuning load chain in `modules/config/config_tuning.c`:

1. `config_tuning_request_load()` reads each `PARAM_ID_*` from flash storage.
2. `config_tuning_on_result()` writes whatever is in flash into the in-memory `g_tuning` struct. **Only NaN/Inf are rejected — `0.0` is accepted as valid.**
3. After all params are loaded, `config_tuning_publish()` fires `TUNING_READY` with the (zeroed) struct.
4. `attitude_estimation.c::on_tuning_ready` overwrites the compiled defaults with the flash values.

Plausible reasons the flash slot contains `0.0`:

- A previous firmware version had different/missing tuning params, leaving uninitialized flash slots that read back as 0.
- An earlier `tuning_board.py` upload sent `0` for those fields.
- The `tuning_params_t` struct layout changed between firmware revisions, so old saved values land in the wrong offsets.

## Fix (User-Facing)

Open [tuning_board.py](../tools/tuning_board.py), connect to the FC, and click **"Upload Defaults"**. This re-writes every `PARAM_ID_*` slot with the correct compiled value. On next boot the loader pulls valid numbers and `on_tuning_ready` no longer poisons the fusion.

## Diagnostic Procedure

1. Connect ST-Link, start CubeIDE debugger.
2. In Live Expressions, watch `g_f11.k0`, `g_f11.beta`, `g_f11.zeta`.
3. If any are `0` (or NaN) → flash tuning is corrupted. Run "Upload Defaults" from `tuning_board.py`.
4. The same symptom can affect other modules whose `on_tuning_ready` overrides defaults from `tuning_params_t` (attitude_control PID gains, position_estimation gains, etc.). Check their fusion/gain state if behavior is wrong.

## Latent Bug (Not Yet Fixed)

`config_tuning_on_result` should reject `0.0` for must-be-positive params (or each consumer's `on_tuning_ready` should guard). Otherwise a single bad upload, partial flash, or future struct-layout change can silently brick attitude estimation again.

Until that guard is added: **after any firmware upgrade that touches `tuning_params_t` layout, run "Upload Defaults" before flying**.
