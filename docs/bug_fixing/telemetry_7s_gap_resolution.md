# Pan Telemetry ~7 s Gaps: Root Cause, Fix and Validation

**Status:** the main issue (Issue 9, ~7 s telemetry gaps) is fixed and validated on hardware as of 2026-10-21. End-to-end validation with the follower passed on 2026-10-22 (§6.4). The ~50 ms reply gate in §4.3 was traced to Waveshare's stock `app.py` running alongside the follower and no longer occurs. The IMU hardware fault and some small cleanups remain (see [Next steps](#6-next-steps)).

**Repositories:**
- Host: `knot-losing-you`. The Python follower runs on the Raspberry Pi.
- Firmware: `ugv_base_general`. The ESP32 `General_Driver` firmware.

**Raw logs:** [docs/bug_fixing/logs/](logs/) and [debug_log.txt](debug_log.txt).

---

## 1. The original issue

The host polls pan angle by sending `T=130` over UART (115200 baud). The firmware answers with a `T=1001` line that contains `pan` and `tilt`. Fresh `T=1001` replies arrived only in short bursts, separated by a fixed gap of about **7 s**.

| Pre-fix telemetry-only run | Success | Median gap | Longest gap |
|---|---|---|---|
| `flush_on_default_30s` (poll 0.05 s, timeout 0.3 s) | 4/103 (3.9%) | 7.27 s | 7.50 s |
| `corr_poll05_timeout2` (poll 0.5 s, timeout 2.0 s) | 5/18 (27.8%) | 7.01 s | 7.04 s |
| `flush_off_default_30s` (RX flush off) | 433/509 (85%) | — | 7.21 s |

Timeouts were almost always `lines_read=0`, meaning nothing arrived on the serial line at all. With flush off, the success count rose, but this only drained a backlog of queued replies: 318 of 433 arrived under 2 ms, and every reply carried the same pan value.

**Effect on the rover:** during each gap the pan controller ran on cached or estimated angles. It then made a hard re-anchor when a fresh sample arrived, which caused pan command discontinuities and oscillation. The stall also blocked the whole firmware loop, so `T=133` pan commands, `T=1` drive commands and the heartbeat stop were all serviced only about once every 7 s. That is a control-latency problem, not just a telemetry problem.

> **Related but separate, already fixed:** an earlier bug left `T=1001.pan` stuck at about `-179.956°`. The cause was an empty `SCSerial::wFlushSCS()`: the half-duplex servo bus echo was being read back as the servo's reply. The fix waits for TX to complete and then drains RX (`pSerial->flush(); rFlushSCS();`). Keep that fix. Details are in [pan_servo_feedback_investigation.md](pan_servo_feedback_investigation.md).

---

## 2. Root cause

**Blocking IMU I2C reads in `updateIMUData()` stalled the firmware's single-threaded `loop()` for about 7 s per pass.**

- `General_Driver.ino` runs `serialCtrl()`, the web server, gimbal feedback, PID, OLED, `updateIMUData()` and `baseInfoFeedback()` in sequence on one thread. Nothing in that loop runs while any one step blocks, and that includes reading the next `T=130`.
- `imuDataGet()` makes 7 `requestFrom` calls per pass (20 I2C transactions across the QMI8658 and the AK09918).
- This IMU was already faulty: `r`, `p` and `y` were `null` (NaN) in every logged `T=1001`.
- On ESP32 core 2.0.14 (ESP-IDF 4.4), an I2C transaction that never completes most likely blocks for about 1 s. Seven of those per pass is about 7 s. I recalled this ~1 s floor from memory and haven't checked it against the IDF source.
- `DebugLevel=none` compiles out the `i2cRead ... Error 263` messages that would have revealed the stall. That is why the gaps were completely silent.

The deciding evidence: `InfoPrint=1` and `uartCmdEcho=1` make the firmware echo every received command before handling it. That echo was **also** missing during the gaps. A gate inside `baseInfoFeedback()` can't suppress the echo, so `serialCtrl()` itself wasn't running. In other words, the loop was blocked, not rate-limited.

---

## 3. The firmware change

Firmware commit `2c9eb13` ("Improve IMU data polling logic to prevent stalls") changed `General_Driver/IMU_ctrl.h` only. It was flashed as build `2d6dda4e` (source commit `7d5f482`, which includes `2c9eb13`).

```cpp
#define IMU_STALL_LIMIT_MS 250   // a healthy imuDataGet() takes ~5-15 ms
bool imuPollingEnabled = true;

void updateIMUData() {
	temp = temperatureRead();            // moved first; not on I2C
	if (!imuPollingEnabled) return;
	unsigned long imuStartMs = millis();
	imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
	if (millis() - imuStartMs > IMU_STALL_LIMIT_MS) {
		imuPollingEnabled = false;       // stop polling the IMU until reboot
	}
	// ... unchanged assignments to ax ... icm_yaw
}
```

This is a stall breaker, not a repair. The first stalled call trips it, so the robot stalls at most once after boot, and after that the IMU is never polled again until reboot. Roll, pitch and yaw stay at their last values (NaN, sent as `null`). Steady mode (`gimbalSteady()`, which uses `icm_pitch`) was already non-functional because of the faulty IMU.

---

## 4. Validation results (2026-10-21)

### 4.1 Acceptance criteria: all pass

| Criterion | Target | Before | After |
|---|---|---|---|
| Recurring ~7 s ceiling | none | 7.0–7.5 s gaps | longest gap 0.052 s (0.500 s at a 0.5 s poll) |
| Median inter-success gap | < 0.5 s | ~7.0 s | 0.051 s |
| Reply latency | < 500 ms | — | max 51.5 ms |
| Flush-off queue-drain bursts | gone | 318/433 under 2 ms | evenly spaced about 51 ms apart |
| Live pan readback | tracks motion | — | 1.3° → 18.9° over ~150 ms for a 20° command |
| Gimbal `T=1005` errors | none | — | none |

| Post-fix run | Success | Latency min/median/max | Median gap |
|---|---|---|---|
| Default (flush on, poll 0.05 s, timeout 0.3 s) | 589/589 | 12.0 / 50.9 / 51.5 ms | — (longest 0.052 s) |
| Poll 0.5 s, timeout 2.0 s | 60/60 | 4.1 / 25.8 / 46.0 ms | 0.500 s |
| Flush off | 590/590 | 2.2 / 50.9 / 51.5 ms | 0.051 s |

In `ugv-check-pan-tilt-feedback --angle 20`, all 40 explicit `T=130` polls returned a value, and the run ended with "No gimbal T=1005 errors seen". The `T=1005` packets for IDs 11, 12, 14 and 15 come from the `T=105`/`T=106` probes addressing arm servos the rover doesn't have, so they can be ignored.

`C_uniq = 1` in the telemetry-only runs is expected, because the gimbal was stationary.

**Interpretation:** the gaps disappeared and nothing else changed, so the IMU is confirmed as the blocker.

### 4.2 Reading the latency numbers

With the periodic feedback stream on (`baseFeedbackFlow=1`), the firmware pushes a `T=1001` about every **51 ms** whether or not it is asked. The host's "reply" is then simply the next stream frame:
- At a 50 ms poll, latency is pinned at about 51 ms.
- At a 500 ms poll, latency is spread from 4 to 46 ms.

Neither figure is true request→reply latency.

With the stream off, every reply is a direct answer to `T=130`. The true round-trip time is **11.6–16.1 ms** (median about 13 ms).

### 4.3 The ~50 ms reply gate (caused by Waveshare's `app.py`; resolved 2026-10-22)

> **Update 2026-10-22: caused by `app.py`, now gone.** Waveshare's stock `~/ugv_rpi/app.py` was being started at every boot by an `@reboot` line in the `ws` user's crontab, and it held `/dev/ttyAMA0` open alongside our code (see [pan_telemetry_serial_worker_crash.md](pan_telemetry_serial_worker_crash.md)). The runs below were almost certainly made while it was running. It has been disabled.
>
> In the first follower run without it ([logs/post_port_release.jsonl](logs/post_port_release.jsonl)), **871 of 871** `T=130` queries were sent 9–45 ms (median 36 ms) after the previous reply, and **every one was answered** (round trip median 15 ms, max 63 ms). Under the gate described below, all of them would have been dropped. So `feedbackFlowExtraDelay` was effectively 0 in that run.
>
> This fits the open question at the end of this section: the stock `ugv_rpi` service was the most likely thing setting `feedbackFlowExtraDelay` to about 50 ms at runtime. A second reader on the same UART would also have consumed some replies. Which of the two produced the alternation hasn't been separated, and doesn't need to be while `app.py` stays disabled. The gate code in the firmware is real, so it will come back if anything sends `T=142` with a non-zero interval.
>
> The original analysis is kept below as evidence.

`ugv-check-pan-tilt-feedback` sends a bare `{"T":131}` in its probe list. That defaults to `cmd=0` and **turns the feedback stream off**. With the stream off, the telemetry-only tool showed strict success/timeout alternation:

| Stream-off run | Success | Pattern |
|---|---|---|
| Default (poll 0.05 s, timeout 0.3 s) | 96/192 | every query sent within about 0–50 ms of the previous reply got nothing |
| Poll 0.1 s (about 85 ms idle between reply and next query) | **300/300** | — |
| Timeout 1.0 s | 31/61 | odd queries got nothing even after 1 s, so they were dropped, not late |

The cause is the existing interval gate in `baseInfoFeedback()` (`ugv_advance.h:375-379`). Direct `T=130` requests and the periodic stream both go through it, and they share `last_feedback_time`:

```cpp
if (millis() - last_feedback_time < feedbackFlowExtraDelay) {
	return;   // silently sends nothing
}
```

In these runs `feedbackFlowExtraDelay` was about **50 ms** at runtime. That matches both the cut-off above and the 51 ms stream period. The source default is `0` (`ugv_config.h:359`), and nothing in the host repo sends `T=142` (`CMD_FEEDBACK_FLOW_INTERVAL`). So something else sets it at runtime, most likely the stock `ugv_rpi` service or the boot mission. (See the update at the top of this section.)

This isn't a regression from the IMU fix. Before the fix, replies inside a burst arrived 17 ms apart, so the gate was 0 during those earlier runs. And the 7 s stall would have hidden it anyway. It doesn't affect `T=1` or `T=133` commands.

---

## 5. What the problem was *not*

Each of these was suspected at some point and has now been ruled out:

| Suspected cause | Why it's ruled out |
|---|---|
| Bus contention from mixed `T=1`/`T=133` traffic | The gaps persisted in telemetry-only runs that sent only `T=130`. |
| Host poll rate | At a fixed 2 s timeout, polling from 2.0 s down to 0.2 s gave a flat 23.5–27.8% success with the same ~7 s gaps. |
| Host timeout too short | Going from 0.2 to 0.3 to 2.0 s improved capture odds but never removed the gaps. |
| RX flush strategy | Flush off only drained queued replies (under 2 ms, one repeated value, gaps still present). Flush on stays the right choice. |
| Host JSON parsing | Timeouts were `lines_read=0`: nothing arrived to parse. |
| `feedbackFlowExtraDelay` gate (as the 7 s cause) | The command echo was also missing during gaps, and in-burst replies were 17 ms apart, so the gate was effectively 0. The separate ~50 ms effect in §4.3 almost certainly came from Waveshare's `app.py` running alongside the follower. |
| "Replies arriving ~2 s late" | The 2026-07-14 linger run read late hits at about 2000 ms as delayed replies. They were actually queued requests being served the moment the loop unblocked. |
| Sequence-token mismatch | The firmware never echoes a token. ρ was indeterminate, not a mismatch. |
| Servo-bus starvation (`FeedBack`/`ReadMode` timeouts) | Capped at about 0.2 s per pass with no `T=1005` lines, so it can't produce a fixed 7.0 s block. Fixing only the IMU removed the gaps. |
| Blocking handlers or mission playback | Telemetry-only runs triggered none. The boot mission runs once in `setup()`. |
| Board reset loop | No ROM banner or setup text appeared, and module state (pan field present) survived across gaps. |
| `server.handleClient()` / a Wi-Fi client | Only the IMU code changed, and that was enough to remove the gaps. |

---

## 6. Next steps

### 6.1 Diagnose the IMU hardware (root fault still present)
The breaker hides the fault; it doesn't fix it. Roll, pitch and yaw are unavailable, and so is gimbal steady mode.
- **Boot-timing check:** time from power-on until the OLED shows "Initialize LittleFS". IMU init runs just before it. About 5 s is normal. A much longer time means the IMU was already stalling at boot.
- **Board mismatch:** the README lists ICM20948 libraries, but the code drives a QMI8658 and an AK09918. Confirm which IMU is actually fitted to this board revision.
- Check the I2C wiring and addresses. Optionally have `imuInit()` return the result of `qmi8658_.begin()`, so polling is skipped when init fails.
- If the IMU is repaired, leave the breaker in place as a safeguard.

### 6.2 Keep `app.py` off the UART
With Waveshare's `app.py` disabled, polling `T=130` every 50 ms works: 873 of 873 queries answered, with a fresh sample about every 50 ms. The continuous stream reader is no longer needed to work around the gate. It is still an option later, if it turns out to be simpler than polling.
- Keep the `app.py` line in the `ws` crontab commented out. Run `fuser -v /dev/ttyAMA0` before a test if in doubt.
- The code hardening in [pan_telemetry_serial_worker_crash.md](pan_telemetry_serial_worker_crash.md) (check the port before opening it; keep the serial worker alive on errors) makes a second process on the UART fail loudly instead of silently.
- **Optional firmware hardening:** let `CMD_BASE_FEEDBACK` bypass the interval check, for example with a `force` argument to `baseInfoFeedback()`, so direct requests always get a reply even if something sets `T=142` again.

### 6.3 Fix the probe tool so it stops turning the stream off
In `ugv-follower/tools/check_pan_tilt_feedback.py`, `_PROBE_COMMANDS` includes `{"T": 131}`, which disables the stream until reboot. Either remove that probe or restore the stream at the end of the run with `{"T":131,"cmd":1}`. Until then, power-cycle the ESP32 or send `{"T":131,"cmd":1}` after running it.

### 6.4 End-to-end validation with the follower (passed 2026-10-22)
[logs/post_port_release.jsonl](logs/post_port_release.jsonl): the full follower with `T=1` and `T=133` traffic, the person moving across the frame.
- [x] No telemetry gaps: fresh samples every ~50 ms (max 93 ms), no timeouts, no stale or degraded mode.
- [x] Every control cycle used `measured-fresh` as its base. No `estimated` or cached bases, and no hard re-anchors.
- [x] Measured pan followed the commands, for example 1° → 24° → 51° → 58° for commands of 29° → 54° → 58°.
- [ ] Pan oscillation is smaller but still present. It is now caused by vision latency and the controller, not telemetry. See [pan_oscillation_vision_latency.md](pan_oscillation_vision_latency.md).
- [ ] Revisit the stale-telemetry mitigations (estimated pan, degraded mode). They never triggered in this run, so they may be simplified or removed.

### 6.5 Firmware cleanups found during review (not related to the gaps)
- **Wrong status field.** `gimbal_module.h:90` and `:112` set `servoFeedback[0/1].status` (the arm's array) on a failed gimbal read, instead of `gimbalFeedback[n].status`. A stale `pos` is then reported as valid.
- **Unused reads.** The `ReadMode(GIMBAL_*_ID)` calls write `gimbalFeedback[].mode`, which is never read. Removing them saves two bus transactions per loop.
- **Thread safety.** `OnDataRecv` (ESP-NOW, `esp_now_ctrl.h`) runs `jsonCmdReceiveHandler()` and servo calls on the Wi-Fi task, outside `loop()`. If ESP-NOW control is ever used, that is a thread-safety risk for `st` and the shared JSON documents.

---

## Documents this replaces

The following investigation notes and handoffs are superseded by this file and can be deleted. The raw logs in [logs/](logs/) and [debug_log.txt](debug_log.txt) should be kept as evidence.

**Host repo (`knot-losing-you`):**
- `docs/bug_fixing/firmware_investigation_handoff.md`
- `docs/bug_fixing/issue9_firmware_schedule_investigation.md`
- `docs/bug_fixing/step_b_minimal_refactor_spec.md` (the tooling it specified is implemented)
- The Issue 9 findings section of `docs/bug_fixing/pan_oscillation_issues.md`. That file covers other issues too, so trim the section rather than deleting the file.

**Firmware repo (`ugv_base_general`):**
- `docs/bug_fixing/firmware_investigation_handoff.md` (an identical copy of the host one)
- `docs/bug_fixing/firmware_feedback_gate_step_summary.md`
- `docs/bug_fixing/gimbal_telemetry_refactor_proposal.md`
- `docs/bug_fixing/gimbal_telemetry_source_review.md`

Keep [pan_servo_feedback_investigation.md](pan_servo_feedback_investigation.md) for the separate, already-fixed `wFlushSCS()` bug. Keep the general theory notes in `docs/engineering_theory/` as well.
