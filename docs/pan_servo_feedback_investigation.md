# Pan Servo Live Feedback Investigation

## Context

This document captures the findings from an investigation into whether the ESP32 firmware on the Waveshare UGV Rover exposes live pan servo position feedback over serial JSON. It is intended to provide enough context to continue debugging in a new chat session.

### Hardware

| Component | Detail |
|---|---|
| Robot | Waveshare UGV Rover |
| Pan servo | ST3215 Serial Bus Servo (half-duplex TTL, 1 Mbps) |
| Microcontroller | ESP32 (handles servo bus + JSON serial bridge) |
| Host | Raspberry Pi, connected to ESP32 via `/dev/ttyAMA0` at 115200 baud |

### Serial Protocol (T-codes)

| T-code | Direction | Description |
|---|---|---|
| `T=605` | Host → ESP32 | Set `InfoPrint` level (`cmd=1` enables debug serial output) |
| `T=900` | Host → ESP32 | Module init (`main=2, module=2` for UGV Rover with pan-tilt) |
| `T=130` | Host → ESP32 | Poll telemetry (triggers a T=1001 response) |
| `T=133` | Host → ESP32 | Pan/tilt command (`X`=pan degrees, `Y`=tilt degrees) |
| `T=1001` | ESP32 → Host | Base telemetry response; includes `pan` and `tilt` fields when gimbal module is active |
| `T=1005` | ESP32 → Host | Bus servo error — emitted when `FeedBack()` fails for a servo ID (requires `InfoPrint=1`) |

Example T=1001 response (observed — pan field is stuck):

```json
{"T":1001,"pan":-179.9560394,"tilt":0.0}
```

Example T=1005 error response (FeedBack failure):

```json
{"T":1005,"id":1,"status":0}
```

### ST3215 Encoder

- Resolution: 4096 steps per 360°
- Absolute position, supports live readback via SCServo `FeedBack()` + `ReadPos()`
- Hardware confirmed capable of live feedback (Waveshare wiki documents `Position`, `Load`, `Speed`, `Input Voltage` all readable)

### Correct SCServo Readback Pattern

```cpp
// Call FeedBack() to latch the servo's current state into the library cache
int result = sms_sts.FeedBack(servo_id);  // Returns 0 on success, -1 on failure

if (result == 0) {
    int16_t raw_pos = sms_sts.ReadPos(-1);  // -1 reads from cache populated by FeedBack()
    // raw_pos is in encoder steps (0–4095 for 360°)
    // Convert to degrees: angle = (raw_pos / 4096.0) * 360.0 - 180.0
} else {
    // Servo did not respond — handle error
}
```

---

## Investigation Findings

### What Was Tested

A probe script (`ugv-follower/tools/check_pan_tilt_feedback.py`) was written and run on the Raspberry Pi host. It:

1. Optionally sends `T=900` to initialise the ESP32 servo module
2. Sends `T=605 cmd=1` to enable `InfoPrint` (firmware debug output, including T=1005 errors)
3. Commands the pan servo to a target angle via `T=133`
4. Listens for `T=1001` responses and extracts the `pan` field
5. Also polls `T=130` explicitly and samples responses
6. Tracks any `T=1005` bus servo error packets throughout all phases

Test conditions covered:

- Pan commanded to **20°** (with T=900 init)
- Pan commanded to **60°** (with T=900 init)
- Pan commanded to **20°** (without T=900 init, using `--no-init` flag)

### Key Finding

**`T=1001.pan` always reports `-179.9560394` regardless of commanded angle.**

- The servo **physically moves** to the commanded angle (visually confirmed)
- The `pan` field **never changes** across any test condition
- The value `-179.9560394` is mathematically explained below

### Mathematical Explanation of the Stuck Value

The firmware computes pan angle using `panAngleCompute()`:

```cpp
float panAngleCompute(int inputPos) {
  return mapFloat((inputPos - 2047), 0, 4095, 0, 360);
}
```

With `gimbalFeedback[0].pos` zero-initialised (its C++ default):

$$\text{panAngleCompute}(0) = \frac{(0 - 2047) \times 360}{4095} = -179.9560\ldots$$

This is exactly the observed value. **`gimbalFeedback[0].pos` is never written** because it is only updated inside the `if(st.FeedBack(GIMBAL_PAN_ID) != -1)` guard in `getGimbalFeedback()`, and `FeedBack()` is consistently returning `-1`.

---

## Firmware Analysis (waveshareteam/ugv_base_ros)

The relevant firmware source is at: https://github.com/waveshareteam/ugv_base_ros

### `gimbal_module.h` — `getGimbalFeedback()`

The firmware **does** call `FeedBack()` before `ReadPos(-1)`. The pattern is correct in the repo:

```cpp
void getGimbalFeedback() {
  if(st.FeedBack(GIMBAL_PAN_ID) != -1) {
    gimbalFeedback[0].status = true;
    gimbalFeedback[0].pos = st.ReadPos(-1);
    // ... speed, load, voltage, current, temp, mode
  } else {
    servoFeedback[0].status = false;  // NOTE: bug — writes to servoFeedback, not gimbalFeedback
    if(InfoPrint == 1) {
      // emits {"T":1005,"id":GIMBAL_PAN_ID,"status":0}
    }
  }
  // same pattern for GIMBAL_TILT_ID → gimbalFeedback[1]
}
```

**Known firmware bug:** the `else` branch writes `servoFeedback[0].status = false` instead of `gimbalFeedback[0].status = false`. This means `gimbalFeedback[0].status` is never set to `false` on failure — but does not affect the `pos` field behaviour.

### `ugv_advance.h` — T=1001 pan/tilt population

The T=1001 `pan` and `tilt` fields are sourced from `gimbalFeedback[].pos` via `panAngleCompute()` / `tiltAngleCompute()`. These are **live-readback fields**, not software target variables. The firmware architecture is correct; the problem is that `FeedBack()` never succeeds.

### Conclusion

**`FeedBack()` is returning `-1` on every call.** The firmware code is structured correctly — it just never gets past the `if` guard because the servo does not respond on the bus. Failure point #1 (FeedBack not called) is ruled out. The root cause is one of the bus-level failures in failure points #2 or #5 below.

---

## Latest Confirmation (May 2026)

The lower-controller firmware mapping has now been cross-checked against `waveshareteam/ugv_base_general` (described by Waveshare as the lower computer program).

Confirmed definitions:

```cpp
#define CMD_BASE_FEEDBACK 130
#define FEEDBACK_BASE_INFO 1001
#define CMD_BUS_SERVO_ERROR 1005
```

Confirmed dispatch path:

```cpp
case CMD_BASE_FEEDBACK:
  baseInfoFeedback();
  break;
```

Confirmed gimbal telemetry path:

- `baseInfoFeedback()` sets `jsonInfoHttp["T"] = FEEDBACK_BASE_INFO`.
- In gimbal mode (`moduleType == 2`), it publishes:
  - `pan = panAngleCompute(gimbalFeedback[0].pos)`
  - `tilt = tiltAngleCompute(gimbalFeedback[1].pos)`

Confirmed feedback-read gate:

- `getGimbalFeedback()` updates `gimbalFeedback[]` only when `st.FeedBack(GIMBAL_*) != -1`.
- On failure and `InfoPrint == 1`, firmware emits `{"T":1005,"id":...,"status":0}`.

Confirmed expected gimbal IDs in this firmware family:

```cpp
#define GIMBAL_PAN_ID  2
#define GIMBAL_TILT_ID 1
```

### Implication

This conclusively rules out "wrong telemetry command" as the root cause:

- `T=130` is the request.
- `T=1001` is the response payload.

Your probe output (continuous `T=1005` with stuck `T=1001.pan`) is consistent with readback failure on the servo bus, not protocol misunderstanding.

---

## Probable Failure Points (Ranked)

### 1. ~~`FeedBack()` / `ReadPos()` not called per telemetry cycle~~ — RULED OUT

Firmware source confirmed that `getGimbalFeedback()` calls `st.FeedBack(GIMBAL_PAN_ID)` before every `ReadPos(-1)`. The call is present and the pattern is correct.

---

### 2. `FeedBack()` consistently returns `-1` — servo not responding on bus

**Most likely remaining cause.** `FeedBack()` sends a read request to the servo and waits for a response packet. If the servo never responds, `FeedBack()` times out and returns `-1`, leaving `gimbalFeedback[0].pos` at its zero-initialised default.

The T=1005 diagnostic path in the firmware (gated on `InfoPrint == 1`) will emit `{"T":1005,"id":<GIMBAL_PAN_ID>,"status":0}` on every failure. **Running the updated probe script will confirm this definitively** — a stream of T=1005 packets means `FeedBack()` is failing every cycle.

If T=1005 is observed:
- Check `GIMBAL_PAN_ID` constant in firmware matches the physical servo's programmed ID
- Check servo bus wiring (single-wire half-duplex, correct pin)
- Check servo power

Additional signal from latest run: repeated `T=1005` was observed for multiple IDs (`1`, `2`, and probe-visible IDs such as `11/12/14/15`). This broad failure pattern increases confidence that the problem is a generic readback-path issue (half-duplex RX direction / bus layer), not only a single wrong pan ID.

---

### 3. `pan` field sourced from target angle variable, not from servo register

**Now less likely** given firmware source confirms `panAngleCompute(gimbalFeedback[0].pos)` is used. Only relevant if the robot is running different firmware than the ugv_base_ros repo.

**How to confirm:** Command an angle, then physically block the servo. If `T=1001.pan` reports the commanded angle rather than the stuck position, the field comes from software state.

---

### 4. Servo operating in Motor Mode instead of Servo Mode

**Less likely but worth ruling out.** If the pan servo is in Motor Mode, `FeedBack()` may succeed but position data is meaningless.

**How to confirm:** Read Mode register (address `0x21`). Value `0` = Servo Mode, `1` = Motor Mode.

---

### 5. UART half-duplex direction control misconfigured

**Plausible.** ST3215 uses a single-wire half-duplex bus. The ESP32 must toggle a direction pin between TX and RX. If the pin is stuck in TX mode, `SyncWritePosEx` (write-only) works fine but `FeedBack()` (requires receiving a response packet) always times out and returns `-1`. This would perfectly explain the symptom: servo moves, but feedback always fails.

**How to confirm:** Scope the bus direction/DE pin during a `FeedBack()` call. It must transition from TX → RX after the command byte is sent. If it stays high throughout, readback is impossible regardless of servo health.

---

## Next Steps

### Immediate: run updated probe script

The probe script now sends `T=605 cmd=1` at startup and collects T=1005 packets throughout all phases. Run it and check the summary line:

```bash
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30
```

- **T=1005 packets seen** → `FeedBack()` is confirmed failing; note the `id` field to verify `GIMBAL_PAN_ID`
- **No T=1005 packets, pan still stuck** → robot firmware differs from ugv_base_ros; InfoPrint path may not be compiled in

### If FeedBack() failure confirmed (T=1005 seen)

1. **Check half-duplex direction pin first** — scope the bus direction/DE pin during `FeedBack()` (most likely cause when many IDs fail readback).
2. **Check servo power and bus wiring** — verify signal continuity, shared ground, and stable servo supply under motion.
3. **Verify servo IDs** — confirm physical gimbal IDs match firmware expectations (`PAN=2`, `TILT=1`) or update firmware constants accordingly.

### If no T=1005 and pan still stuck

1. Verify firmware version on the robot (check for version string or compare behaviour against ugv_base_ros `main` branch)
2. Consider reading servo Mode register to rule out Motor Mode (failure point #4)

---

## Probe Script

The investigation script is at:

```
ugv-follower/tools/check_pan_tilt_feedback.py
```

CLI entry point: `ugv-check-pan-tilt-feedback`

The script:
- Sends `T=605 cmd=1` (enables `InfoPrint` so firmware emits T=1005 on `FeedBack()` failure)
- Collects and counts T=1005 bus servo error packets in all phases
- Prints a diagnostic summary identifying the servo IDs that failed

Usage:

```bash
# With module init (default)
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30

# Skip T=900 init
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30 --no-init

# Longer capture
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30 --duration 15
```
