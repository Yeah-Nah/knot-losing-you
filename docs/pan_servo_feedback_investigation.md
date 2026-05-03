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
| `T=900` | Host → ESP32 | Module init |
| `T=133` | Host → ESP32 | Pan command (angle in degrees) |
| `T=130` | Host → ESP32 | Poll telemetry |
| `T=1001` | ESP32 → Host | Telemetry response; includes `pan` field |
| `T=1005` | ESP32 → Host | Acknowledgement |

Example T=1001 response:

```json
{"T":1001,"pan":-179.9560394,"tilt":0.0}
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
2. Commands the pan servo to a target angle via `T=133`
3. Listens for `T=1001` responses and extracts the `pan` field
4. Also polls `T=130` explicitly and samples responses

Test conditions covered:

- Pan commanded to **20°** (with T=900 init)
- Pan commanded to **60°** (with T=900 init)
- Pan commanded to **20°** (without T=900 init, using `--no-init` flag)

### Key Finding

**`T=1001.pan` always reports `-179.9560394` regardless of commanded angle.**

- The servo **physically moves** to the commanded angle (visually confirmed)
- The `pan` field **never changes** across any test condition
- The value `-179.9560394` = `-(4095/4096) * 180` — exactly one encoder step below the negative range boundary
- This is consistent with an **uninitialised encoder cache** — it is the default value the SCServo library holds before `FeedBack()` is ever successfully called

### Root Cause Conclusion

The ESP32 firmware is **not forwarding live SCServo position data** into the `T=1001.pan` JSON field. The field is either:

- Sourced from an internal software variable (target angle, not measured angle), or
- Sourced from a stale/uninitialised servo register cache

---

## Probable Failure Points (Ranked)

### 1. `FeedBack()` / `ReadPos()` not called per telemetry cycle

**Most likely cause.** The firmware probably never calls `sms_sts.FeedBack(id)` before publishing `T=1001`. The SCServo library does not auto-update its internal cache — `FeedBack()` must be called explicitly before each `ReadPos(-1)`.

**How to confirm:** Add `Serial.println(sms_sts.FeedBack(PAN_ID))` in the telemetry handler and observe whether it returns `0` or `-1`. If the call is absent entirely, the cache is never refreshed.

---

### 2. `FeedBack()` return value not checked — silent failure

**Likely co-cause.** If `FeedBack()` is called but its return value is not checked, a failed read (return `-1`) will leave `ReadPos(-1)` returning stale data. The firmware would silently publish an old or default value with no error indication on the JSON stream.

**How to confirm:** Instrument the firmware to log the FeedBack return code. A consistent `-1` would indicate the servo is not responding to readback requests on the bus.

---

### 3. `pan` field sourced from target angle variable, not from servo register

**Plausible.** The firmware may populate `pan` in `T=1001` from the same variable used to track the last-commanded angle (i.e., what angle was *sent* to the servo, not what the servo *reports*). This would explain why the field does not track physical position.

**How to confirm:** Command an angle, then physically block the servo from reaching it. If `T=1001.pan` still reports the commanded angle rather than the actual stuck position, the field is derived from software state, not hardware readback.

---

### 4. Servo operating in Motor Mode instead of Servo Mode

**Less likely but worth ruling out.** ST3215 supports both Servo Mode (absolute position, encoder active) and Motor Mode (continuous rotation, encoder not used for position). If the servo was inadvertently configured in Motor Mode, absolute position feedback would be unavailable or meaningless.

**How to confirm:** Read the servo's Mode register (address `0x21` / register 33). Value `0` = Servo Mode, `1` = Motor Mode. Can be done via a direct SCServo read from the ESP32 or a standalone Python tool using `pypots`/`scservo_sdk`.

---

### 5. UART half-duplex direction control misconfigured

**Least likely given that commands work, but possible.** ST3215 uses a single-wire half-duplex bus. The ESP32 must switch a direction pin (or use hardware auto-direction) between TX and RX. If the direction pin is stuck in TX mode, servo commands are transmitted correctly but readback bytes are never received — `FeedBack()` would time out and return `-1` every time.

**How to confirm:** Scope the bus direction pin (or UART DE pin if using RS485 transceiver) during a `FeedBack()` call. It should toggle from TX to RX after the command byte is sent. If it stays high (TX), readback is impossible.

---

## Recommended Next Steps

1. **Get ESP32 firmware source** — the Waveshare UGV firmware is on GitHub. Locate the `T=1001` telemetry handler and check how `pan` is populated.
2. **Search for `ReadPos` in firmware** — if absent, failure point #1 is confirmed.
3. **Add serial debug logging** to the ESP32 telemetry cycle: log `FeedBack()` return code and raw `ReadPos(-1)` value before JSON serialisation.
4. **Check servo Mode register** to rule out failure point #4.
5. **Scope the half-duplex bus** if `FeedBack()` consistently returns `-1` after firmware logging is added.

---

## Probe Script

The investigation script is at:

```
ugv-follower/tools/check_pan_tilt_feedback.py
```

CLI entry point: `ugv-check-pan-tilt-feedback`

Usage:

```bash
# With module init (default)
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30

# Skip T=900 init
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30 --no-init
```
