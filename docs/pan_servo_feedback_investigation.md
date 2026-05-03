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
int result = sms_sts.FeedBack(servo_id);  // Returns servo ID (>= 0) on success, -1 on failure

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

## Firmware Analysis (waveshareteam/ugv_base_general)

The relevant firmware source is at: https://github.com/waveshareteam/ugv_base_general

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

**`FeedBack()` is returning `-1` on every call.** The firmware code is structured correctly — it never gets past the `if` guard because `FeedBack()` always fails. The root cause is the empty `wFlushSCS()` function in `SCServo/SCSerial.cpp`, which causes the TX echo to be misread as the servo response. See the Root Cause Identified section below.

---

## Confirmation and Servo ID Scan (May 2026)

The lower-controller firmware mapping has been cross-checked against `waveshareteam/ugv_base_general` (described by Waveshare as the lower computer program).

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
- On failure and `InfoPrint == 1`, firmware emits `{"T":1005,"id":..., "status":0}`.

Confirmed expected gimbal IDs in this firmware family:

```cpp
#define GIMBAL_PAN_ID  2
#define GIMBAL_TILT_ID 1
```

### Servo ID Scan Results

A diagnostic script (`ugv-follower/tools/scan_servo_ids.py`, CLI: `ugv-scan-servo-ids`) was written and run. It observes T=1005 packets across a 20 s window and cross-references which servo groups fail.

**Results:**

- T=1005 failures seen only for IDs **1** and **2** (gimbal group) — 103 failures each.
- **No drive servo ID failures observed.**
- The UGV Rover uses DC motors with quadrature encoders for the wheels, not bus servos — so IDs 11–15 are absent from this bus entirely.
- The servo **physically moves** when commanded via `T=133`. The `SyncWritePosEx` write path therefore works — IDs 1 and 2 exist on the bus and respond to writes.
- Since writes succeed but reads always fail on the same IDs, wrong servo IDs are ruled out. The problem is in the receive path.

### Implication

This conclusively rules out both "wrong telemetry command" and "wrong servo IDs" as root causes:

- `T=130` is the correct request; `T=1001` is the correct response payload.
- IDs 1 and 2 are correct — the servos move when commanded to those IDs.
- The failure is isolated to the read direction of the servo bus.

---

## Root Cause Identified (May 2026)

Source analysis of `waveshareteam/ugv_base_general` — specifically `SCServo/SCSerial.cpp` and `SCServo/SCS.cpp` — identifies the bug precisely.

### UART wiring

The servo bus uses two separate GPIO pins wired to the same single-wire half-duplex bus:

```cpp
// General_Driver/ugv_config.h
#define S_RXD 18   // GPIO 18
#define S_TXD 19   // GPIO 19
```

Every byte the ESP32 transmits is immediately echoed back on the RX pin.

### The bug: `wFlushSCS()` is empty

```cpp
// SCServo/SCSerial.cpp
void SCSerial::wFlushSCS()
{
    // completely empty — does nothing
}
```

This function is called after every transmission before reading begins. Its intended purpose is to:

1. Wait until the UART hardware has fully shifted out every queued byte.
2. Drain the echo of those bytes from the RX buffer.

**It does neither.** It is a no-op.

### What actually happens on every `FeedBack()` call

The call chain in `SCS.cpp` for a read request is:

```
rFlushSCS()   → clears stale RX bytes (correct)
writeBuf()    → queues request packet into TX FIFO and returns immediately
wFlushSCS()   → does nothing  ← BUG
checkHead()   → immediately reads from RX looking for 0xFF 0xFF
```

By the time `checkHead()` runs, the UART is still transmitting. The TX echo arrives on RX first — and the request packet begins with `0xFF 0xFF`. `checkHead()` finds those bytes, treats them as the start of a servo response, then tries to parse what follows. What follows is the rest of the outgoing request, not a servo reply. The checksum fails or the length mismatches, `Read()` returns 0, and `FeedBack()` returns `-1`.

The servo's actual response packet arrives after the echo, but nothing is reading at that point.

Writes succeed because `SyncWritePosEx()` calls `syncWrite()`, which only transmits and never reads anything back.

### The fix

A two-line patch to `SCServo/SCSerial.cpp`:

```cpp
void SCSerial::wFlushSCS()
{
    pSerial->flush();  // block until TX FIFO fully drains
    rFlushSCS();       // discard the echoed TX bytes from RX buffer
}
```

`pSerial->flush()` on Arduino/ESP32 blocks until the hardware UART has shifted out every queued byte. `rFlushSCS()` (which already exists and does `while(pSerial->read()!=-1)`) then drains the echo. After that, the RX buffer is empty and the servo's actual response will arrive cleanly.

---

## Failure Points — Final Status

### 1. ~~`FeedBack()` / `ReadPos()` not called per telemetry cycle~~ — RULED OUT

Firmware source confirmed that `getGimbalFeedback()` calls `st.FeedBack(GIMBAL_PAN_ID)` before every `ReadPos(-1)`. The call is present and the pattern is correct.

---

### 2. ~~Wrong servo IDs~~ — RULED OUT

The servo physically moves when commanded via `T=133` targeting IDs 1 and 2. Writes to those IDs succeed. The servo ID scan confirmed only IDs 1 and 2 appear in T=1005 failures — consistent with correct IDs that simply cannot be read back, not with IDs that don't exist on the bus.

---

### 3. ~~`pan` field sourced from software target, not servo register~~ — RULED OUT

Firmware source confirms `panAngleCompute(gimbalFeedback[0].pos)` is used, sourced from the `FeedBack()` read path.

---

### 4. ~~Servo in Motor Mode~~ — RULED OUT

Irrelevant given root cause identified. `FeedBack()` never succeeds at all — the failure is before any position data is interpreted.

---

### 5. ~~UART half-duplex direction control misconfigured~~ — PARTIALLY CORRECT, SUPERSEDED

The problem is indeed in the half-duplex RX path, but the specific mechanism is not a direction/DE pin — it is the empty `wFlushSCS()` function causing the TX echo to be misread as the servo's response. See Root Cause section above.

---

### 6. `wFlushSCS()` empty — TX echo read as servo response — **CONFIRMED ROOT CAUSE**

See Root Cause Identified section above for full analysis and fix.

---

## Next Steps — Flash Patched Firmware

### 1. Clone the firmware repository

On a machine with the Arduino IDE or PlatformIO installed (not the Raspberry Pi):

```bash
git clone https://github.com/waveshareteam/ugv_base_general.git
cd ugv_base_general
```

### 2. Apply the patch

Edit `SCServo/SCSerial.cpp`. Find `wFlushSCS()` and replace the empty body:

```cpp
// BEFORE (broken)
void SCSerial::wFlushSCS()
{
}

// AFTER (fixed)
void SCSerial::wFlushSCS()
{
    pSerial->flush();  // block until TX FIFO fully drains
    rFlushSCS();       // discard the echoed TX bytes from RX buffer
}
```

No other files need changing.

### 3. Configure Arduino IDE for ESP32

1. Install the **ESP32 board package** via Arduino IDE → Boards Manager → search "esp32" → install Espressif's package.
2. Install required libraries via Library Manager:
   - **ArduinoJson**
   - **ESP32Servo** (if not already present)
3. Open `General_Driver/General_Driver.ino` in Arduino IDE.
4. Set board: **Tools → Board → ESP32 Arduino → ESP32 Dev Module**.
5. Set upload speed: **921600** (or lower if connection is unreliable).

### 4. Connect and flash

1. Connect a USB cable between your computer and the ESP32's USB port on the UGV Rover.
2. Identify the COM port (Windows Device Manager, or `/dev/ttyUSB0` on Linux).
3. Select the correct port in Arduino IDE under **Tools → Port**.
4. Click **Upload** (the → arrow button).
5. If the upload hangs at "Connecting…", hold the **BOOT** button on the ESP32 while the upload begins, then release.

### 5. Verify the fix

After flashing, run the existing probe script on the Raspberry Pi:

```bash
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30
```

- `T=1001.pan` should now change with servo position.
- T=1005 errors should no longer appear.
- Pan commanded to 30° should produce a `T=1001.pan` value close to `30.0` rather than `-179.9560394`.

---

## Probe Scripts

### `ugv-check-pan-tilt-feedback`

```
ugv-follower/tools/check_pan_tilt_feedback.py
```

Commands pan to a target angle, listens for `T=1001.pan`, and collects T=1005 error packets.

```bash
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30 --no-init
ugv-check-pan-tilt-feedback --port /dev/ttyAMA0 --angle 30 --duration 15
```

### `ugv-scan-servo-ids`

```
ugv-follower/tools/scan_servo_ids.py
```

Listens for T=1005 packets over a configurable window and classifies failures by servo group to distinguish wrong IDs from a bus-wide RX failure.

```bash
ugv-scan-servo-ids                    # 20 s window, /dev/ttyAMA0
ugv-scan-servo-ids --duration 30
ugv-scan-servo-ids --no-init
```
