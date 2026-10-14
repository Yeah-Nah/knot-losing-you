# Firmware Telemetry Cadence Investigation Handoff

## Purpose

Investigate why the ESP32 firmware produces valid pan telemetry (`T=1001`) at a very sparse, bursty cadence. The host-side investigation is complete enough to move the primary root-cause search into the firmware repository.

This document is intended to be copied into the firmware repository and given to an agent as the starting context.

## Repositories

Host project:

- `c:\Users\alexa\git\knot-losing-you`
- Python follower runs on a Raspberry Pi and communicates with the ESP32 over serial.

Firmware project to investigate:

- `c:\Users\alexa\git\ugv_base_general`
- Relevant source is expected under `General_Driver/` and `SCServo/`.

## Hardware and Protocol

- Waveshare UGV Rover.
- ESP32 runs the lower-controller firmware and bridges JSON commands over serial.
- Pan servo: ST3215 serial bus servo on a half-duplex TTL bus.
- Host UART: Raspberry Pi to ESP32 at 115200 baud.
- Servo bus: approximately 1 Mbps.

Relevant JSON messages:

- `T=130`: host requests base telemetry.
- `T=1001`: firmware telemetry response containing `pan` and `tilt` in gimbal mode.
- `T=133`: host sends pan/tilt commands.
- `T=900`: module initialization.
- `T=1005`: servo feedback failure diagnostic when `InfoPrint=1`.

## Important Distinction: Completed Bug vs Current Bug

### Completed: stuck pan readback

The earlier symptom was `T=1001.pan` staying at approximately `-179.956°` even while the servo physically moved. The cause was confirmed and fixed:

- `SCServo/SCSerial.cpp` had an empty `SCSerial::wFlushSCS()`.
- The servo bus echoes transmitted bytes on RX.
- Without waiting for TX completion and draining the echo, the receive path interpreted the outgoing request as the servo response.
- `FeedBack()` returned `-1`, leaving `gimbalFeedback[].pos` at zero.
- `panAngleCompute(0)` produces approximately `-179.956°`.

The applied fix was conceptually:

```cpp
void SCSerial::wFlushSCS()
{
    pSerial->flush();
    rFlushSCS();
}
```

This was reflashed and validated. Live pan feedback worked afterward; a commanded `20°` produced approximately `18.9°`, and gimbal `T=1005` failures disappeared.

Do not treat the old stuck `-179.956°` value as the current investigation target. The current problem is sparse availability/cadence of otherwise valid feedback.

## Current Symptom

The host sends `T=130` repeatedly, but fresh `T=1001` responses arrive only intermittently:

- Repeated query timeouts.
- Earlier runs commonly showed `lines_read=0`, meaning no serial line arrived during the active read window.
- Fresh replies appear in short bursts separated by recurring gaps of roughly `7` seconds.
- Successful responses can be valid and low-latency when they appear.
- During stale periods the controller uses cached or estimated pan values, then makes a hard re-anchor when fresh telemetry returns.

This telemetry sparsity can cause pan command discontinuities and oscillation even when the visual target is stationary.

## Host-Side Evidence Already Completed

Do not repeat these as the primary investigation path unless firmware changes produce new contradictory evidence.

1. **Telemetry-only testing**
   - Sparse replies persist without mixed `T=1` drive traffic or `T=133` pan-command traffic.
   - Mixed-traffic contention is therefore not the leading explanation.

2. **Host poll-rate sweep**
   - Increasing request rate did not improve fresh telemetry density.
   - At fixed `2.0s` timeout, poll rates from `2.0s` down to `0.2s` produced approximately `23.5%` to `27.8%` success.
   - Roughly `7s` inter-success gaps remained.

3. **Timeout sweep**
   - Increasing timeout from `0.2s` to `0.3s` did not fix sparsity.
   - Increasing timeout to `2.0s` improved capture probability but did not remove the multi-second gap pattern.

4. **RX flush ON/OFF comparison**
   - Flush OFF greatly increased parsed-success counts.
   - However, successful values were overwhelmingly the same repeated pan value, with bursty near-zero read latency. This is consistent with draining queued replies, not fresh per-request correlation.
   - Flush OFF did not remove the long gaps.
   - Keep flush ON in the control path for now because it better preserves freshness intent.

5. **Linger / attribution run**
   - The host used a `2.0s` read timeout plus a post-timeout linger window.
   - Approximately `30%` of nominal timeout cycles had a valid `T=1001` reply arrive just after the timeout boundary.
   - Linger hits occurred periodically every third query.
   - The effective extra capture margin was only about `20-23ms`, yet the replies were consistently found there.
   - This strongly indicates an upstream periodic firmware emission/service window. Host timing affects whether a reply is captured, but does not explain the underlying cadence.

6. **Sequence-token attempt**
   - The host attempted to include a sequence token in `T=130` and look for it in `T=1001`.
   - Firmware did not echo the token, so exact request/response correlation could not be computed.
   - This is a firmware capability gap, not evidence of token mismatch.

The detailed raw logs are in the host repository under:

- `docs/bug_fixing/logs/`
- `docs/bug_fixing/debug_log.txt`
- `docs/bug_fixing/pan_oscillation_issues.md`

## Current Working Conclusion

The primary root-cause class is firmware/device-side telemetry cadence limiting or starvation. The most likely mechanisms are:

- An explicit feedback interval gate.
- A synchronous single-threaded `loop()` blocked by servo-bus reads.
- Long-running command handlers or mission playback blocking telemetry publication.
- A fixed service window or retry behavior in the servo bus layer.
- A combination of the above.

Host read-window and flush behavior is a secondary contributor: it can discard or miss late replies, but it does not generate the recurring approximately `7s` firmware cadence.

## Required Investigation Sequence

### Step 1: Trace explicit feedback gating

Find every declaration, read, and write of `feedbackFlowExtraDelay` and related feedback interval settings.

In particular:

- Locate its default value, expected units, and initialization path.
- Trace `setFeedbackFlowInterval(...)`.
- Trace `CMD_FEEDBACK_FLOW_INTERVAL` through the JSON command dispatcher.
- Inspect `baseFeedbackFlow` and the exact condition that allows `baseInfoFeedback()` to publish.
- Check whether the boot mission, startup configuration, or runtime command stream can set a non-zero value.
- Check the actual `/boot.mission` contents on the target if it is stored outside the source tree.

A non-zero gate that explains the observed period would be a direct root cause. If the source default is zero, verify the runtime value rather than stopping at the static default.

### Step 2: Map the synchronous main loop

Inspect the actual `loop()` implementation in `General_Driver/General_Driver.ino` and document the order of:

- Serial command handling.
- HTTP/server handling.
- Module-specific feedback.
- Command dispatch.
- Motion/control updates.
- Display and IMU updates.
- `baseInfoFeedback()`.

Identify which branches run on every iteration and which run only after commands or timers. Confirm whether telemetry publication shares the same execution thread with servo feedback and long handlers.

### Step 3: Inventory blocking operations

Trace the calls made from the loop and record worst-case blocking time for:

- `FeedBack(...)`.
- `Read(...)` and `readSCS(...)`.
- Serial read timeouts and retry loops.
- `getFeedback(...)`.
- `getGimbalFeedback()`.
- `waitMove2Goal(...)`.
- `RoArmM2_delayMillis(...)`.
- `missionPlay(...)` and `moveToStep(...)`.
- Any `delay(...)` in arm, gimbal, or command handlers.
- Watchdog/recovery paths.

Important existing observation to verify: the gimbal module appears to perform two servo feedback reads per loop, while the arm module can perform four. Servo reads can wait up to roughly `100ms` per attempt when a response is missing, depending on configured `IOTimeOut`.

### Step 4: Identify the actual publisher path

Determine which code path emits the sparse responses:

- `baseInfoFeedback()` and `FEEDBACK_BASE_INFO`.
- Base feedback timer/gate.
- Gimbal feedback refresh.
- A command handler or alternate telemetry path.

Confirm whether `T=130` directly triggers publication or only requests data that is later emitted by a loop/timer. Establish whether one host request should produce one response according to the firmware design.

### Step 5: Check serial/bus scheduling behavior

Inspect `SCServo/SCSerial.cpp`, `SCS.cpp`, `SMS_STS.cpp`, and `SCSCL.cpp` for:

- Fixed delays.
- Read retries.
- Timeout accumulation.
- RX/TX buffer handling.
- Half-duplex turnaround assumptions.
- Any periodic service or batching behavior.
- Error handling that can suppress or defer telemetry.

The `wFlushSCS()` fix must be preserved. Do not undo it while investigating cadence.

## Suggested Instrumentation

If static inspection does not explain the cadence, add lightweight instrumentation only. Avoid verbose serial logging that could itself alter timing.

Useful fields/events:

1. Timestamp when a host `T=130` request is received.
2. Timestamp when telemetry handling begins.
3. Timestamp before and after each servo-bus feedback read.
4. Timestamp when `baseInfoFeedback()` begins.
5. Timestamp when the telemetry JSON is formatted.
6. Timestamp when the response is written/transmitted.
7. Current `feedbackFlowExtraDelay` and last-publish timestamp.
8. Counter for feedback-gate suppressions.
9. Marker for entry and exit of long handlers and mission steps.
10. Loop period and total time spent in each module branch.

Prefer a compact binary/counter/debug mode or a low-rate diagnostic packet. Do not print on every loop iteration unless timing impact has been measured.

## Discriminator Tests

Run these in order and record exact firmware configuration and timestamps.

### Gate test

Force the existing feedback interval command to zero immediately before measurement. Compare the inter-success gaps with the prior approximately `7s` pattern.

- If gaps collapse, classify as feedback-gate configuration or reapplication.
- If gaps persist, continue.

### Servo workload test

Use an existing module/configuration mode that minimizes or disables unnecessary servo feedback polling while keeping telemetry enabled.

- If gaps collapse, classify as synchronous loop starvation from servo-bus polling.
- If gaps persist, continue.

### Blocking-handler test

Repeat telemetry-only measurement while ensuring no mission playback, movement-to-goal, arm delay, or other long-running handler is active.

- If gaps align with handler execution, classify as shared-loop command-handler blocking.
- If gaps persist, continue.

### Bus timing test

Instrument or measure individual servo reads, retries, and response timeouts.

- If accumulated bus waits explain the periodic release pattern, classify as bus-layer scheduling/retry starvation.
- Otherwise inspect telemetry publication scheduling and timer logic more deeply.

## Acceptance Criteria After a Fix

A firmware-side fix is considered successful when telemetry-only validation shows:

- No recurring approximately `7s` fresh-sample ceiling.
- Median inter-success gap below `0.5s`.
- Reply latency consistently below `500ms`.
- Fresh pan values update with actual commanded/motion context rather than one repeated value dominating the run.
- No regression in live pan readback or gimbal servo communication.
- `T=1005` diagnostics remain absent for valid gimbal IDs during normal operation.

If sequence echo is added to the firmware, the host can additionally target at least `80%` proven same-cycle request/response matches. Sequence echo is useful but not required for the initial cadence fix.

## Parallel Host Mitigation

The host-side long-term mitigation is a continuous serial reader that:

- Continuously consumes complete JSON lines.
- Timestamps each received frame.
- Stores the latest `T=1001` sample.
- Exposes sample age/freshness to the controller.
- Avoids losing late replies through a strict flush-before-query/read-window cycle.

This is a mitigation and measurement improvement. It does not replace finding the firmware cause of the approximately `7s` cadence.

## Expected Deliverable From This Investigation

Produce a short report in the firmware repository containing:

1. The exact telemetry publisher path.
2. All feedback-gate defaults and mutation sites.
3. A loop/blocking-call timing inventory.
4. The firmware mechanism that explains the observed cadence, or a clear statement of what remains unmeasured.
5. The smallest plausible firmware fix.
6. A validation procedure and measured before/after results.

Do not reopen broad host poll-rate, parser, or small-timeout sweep campaigns unless a firmware change produces evidence that contradicts the conclusions above.
