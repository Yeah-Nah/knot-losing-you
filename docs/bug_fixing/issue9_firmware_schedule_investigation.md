# Issue 9 Reset Brief - Firmware Scheduling and Synchronization

## Purpose
This document is a concise reset for the next investigation branch.
It tells a new agent what is already known, what has been ruled out or deprioritized, and what must be investigated next.

## Original Issue (Concise)
Pan telemetry replies (T=1001) are too sparse and weakly correlated to control updates. This causes long stale periods, estimate drift, and abrupt command re-anchoring when fresh telemetry returns. The dominant observed symptom has been repeated query timeouts with lines_read=0 and recurring multi-second fresh-sample gaps near ~7 seconds.

## What Has Already Been Established
- The immediate bottleneck is telemetry availability/cadence, not plausibility-guard rejection logic.
- Sparse telemetry persists in telemetry-only runs (without mixed T=1 or T=133 traffic).
- Increasing host request rate did not materially improve fresh-sample density.
- Small timeout increases (0.2s to 0.3s) did not fix sparse fresh updates.
- Longer timeout windows improve capture ratio but do not remove the recurring multi-second gap pattern.
- Flush OFF raises parsed-success counts but shows queue-drain behavior and does not prove fresh-per-request correlation.
- Linger evidence shows many valid replies arrive just after timeout boundary; strict flush-then-window host logic can discard near-boundary late replies.
- Attribution evidence indicates the ~7 second periodicity is generated upstream of host parsing/request-rate tuning.

## Ruled Out or Deprioritized for Primary Root-Cause Search
Do not spend primary investigation time on these unless new contradictory evidence appears:

- Further host poll-rate sweep campaigns as the main path.
- Parser-centric debugging as the main path (timeouts are dominated by no-line-arrival windows).
- Treating flush OFF as a control-path fix (it inflates capture while preserving cadence/queue signatures).
- Small timeout-only iteration loops as a standalone fix path.
- Re-running completed correlation sweeps and attribution run without a new behavioral change trigger.

## Working Interpretation
Primary root-cause class is firmware/device-side response cadence and scheduling behavior, with host read-window strategy as a secondary contributor to lost late replies.

This means the question is no longer "is host polling too fast/too slow?" but "which firmware scheduling, servicing, or synchronization mechanism is creating the effective ~7s telemetry cadence ceiling?"

## New Investigation Direction (Required)
Focus on firmware scheduling, bus servicing, and main-loop starvation.

### Findings From The Current Code Review
- Your `SCSerial::wFlushSCS()` fix is on the normal `Read()`/`Ping()`/`FeedBack()` path, so it is relevant to servo reliability and echo cleanup.
- The sync-read API still bypasses `wFlushSCS()`, but the inspected firmware telemetry paths do not appear to rely on sync-read for the pan/arm feedback that is currently sparse.
- The more likely source of the recurring long gaps is the firmware's synchronous single-loop execution model plus blocking servo-bus work and inline command handlers.
- There is also an explicit feedback gate in the firmware: `feedbackFlowExtraDelay` in `General_Driver/ugv_advance.h`, set through `CMD_FEEDBACK_FLOW_INTERVAL` in `General_Driver/json_cmd.h` and `setFeedbackFlowInterval(...)`. If this is non-zero, it can suppress feedback regardless of bus health.
- I did not find a visible task/priority split in the inspected runtime surface; the code that matters here is mostly serialized through `loop()` and the handlers it calls.
- Arm/gimbal feedback paths appear to poll servo feedback continuously in loop-driven paths, not only when host telemetry requests arrive; this keeps servo-bus work in the critical path even in telemetry-focused runs.
- The strongest starvation pattern match remains: single-threaded loop + chained blocking servo reads + occasional inline blocking handlers, which can naturally produce stale periods and bursty catch-up replies.

### Core Questions
- Which path is actually publishing the sparse telemetry replies: base feedback flow, arm feedback, gimbal feedback, or a command handler?
- Is the delay coming from servo-bus reads blocking the main loop, or from the telemetry emission path itself?
- Is `feedbackFlowExtraDelay` ever being set to a large value at boot or by a runtime command?
- Are `getFeedback(...)` and `getGimbalFeedback()` monopolizing `loop()` long enough to delay reply publication for seconds?
- Are long-running handlers such as `waitMove2Goal(...)`, `RoArmM2_movePosGoalfromLast(...)`, `missionPlay(...)`, or `RoArmM2_delayMillis(...)` blocking the same loop that services telemetry?
- Does any path still accumulate stale RX data or trigger retries despite the current flush fix, especially around half-duplex echo and packet framing?

### Relevant Firmware Files
Start here first when investigating this branch:

- `C:\Users\alexa\git\ugv_base_general\General_Driver\General_Driver.ino`
- `C:\Users\alexa\git\ugv_base_general\General_Driver\RoArm-M2_module.h`
- `C:\Users\alexa\git\ugv_base_general\General_Driver\gimbal_module.h`
- `C:\Users\alexa\git\ugv_base_general\General_Driver\ugv_advance.h`
- `C:\Users\alexa\git\ugv_base_general\General_Driver\uart_ctrl.h`
- `C:\Users\alexa\git\ugv_base_general\General_Driver\json_cmd.h`
- `C:\Users\alexa\git\ugv_base_general\General_Driver\ugv_config.h`
- `C:\Users\alexa\git\ugv_base_general\SCServo\SCSerial.cpp`
- `C:\Users\alexa\git\ugv_base_general\SCServo\SCS.cpp`
- `C:\Users\alexa\git\ugv_base_general\SCServo\SMS_STS.cpp`
- `C:\Users\alexa\git\ugv_base_general\SCServo\SCSCL.cpp`

### Scheduler-Path Inspection Expectations
Inspect and document:
- which `loop()` branches run on every iteration versus only on commands
- queue sizes and queue-full behavior, if any async queues exist in the firmware surface
- lock/mutex contention points, if any async primitives exist in the firmware surface
- blocking calls and worst-case blocking durations, especially serial bus reads and `delay(...)` calls
- any watchdog/recovery behavior that can create periodic release patterns
- any explicit telemetry emission gating logic, especially `feedbackFlowExtraDelay`

### Firmware Instrumentation Expectations
If you later decide to instrument, add lightweight timestamps/counters around these events:
- host request received
- telemetry handling start
- servo-bus read dispatched
- servo-bus read returned
- telemetry payload formatted
- response enqueued/transmitted to host

## Next Steps
Execute these in order so each step cleanly rules in/out one bottleneck class.

### A) Fast Code-Path Verification
Purpose: find the shortest plausible code-path explanation for the sparse telemetry before running more experiments. This step is trying to answer: is the firmware itself deliberately spacing out telemetry, or is the main loop too busy or blocked to send replies promptly?

- Trace every writer of `feedbackFlowExtraDelay` from boot through runtime command handlers. Record defaults, all mutation sites, and units (ms).
- Confirm whether any boot script, mission init, or startup command sequence sets `CMD_FEEDBACK_FLOW_INTERVAL` to a non-zero value.
- Map `loop()` branches to determine which feedback functions execute every iteration versus conditionally (`getFeedback(...)`, `getGimbalFeedback()`, base feedback sender).
- Build a blocking-call inventory in loop-critical paths: serial reads, retries, and all explicit `delay(...)` usage.

### B) Discriminator Test Sequence (No Firmware Edits)
Purpose: use runtime-only tests to separate the major root-cause classes without changing firmware yet. This step is trying to discover whether the gaps are caused by an interval gate, by servo-bus workload, or by some command path reapplying a bad setting.

- Gate test: force `feedbackFlowExtraDelay = 0` via existing command path, then rerun telemetry-only polling and compare inter-success gaps.
- Polling load test: run equivalent telemetry polling with servo-feedback-heavy module logic minimized/disabled (where supported by existing mode/module config) and compare cadence.
- Runtime reapply test: keep a long telemetry-only run and log inbound command traffic to verify whether any periodic command resets a non-zero feedback interval.
- Bus traffic isolation: repeat with gimbal steady mode OFF to avoid adding avoidable bus load during diagnosis.

### C) Minimal Instrumentation If A/B Are Inconclusive
Purpose: add just enough timing visibility to catch where time is actually being spent when the simple checks are not decisive. This step is trying to turn a vague "firmware feels slow" suspicion into timestamped evidence about which loop segment is starving telemetry.

- Add lightweight per-loop timing and counters (no heavy logging): loop period, time spent in arm feedback, time spent in gimbal feedback, telemetry publish timestamp.
- Add one counter for feedback-gate suppressions (how often publish was skipped due to interval gate).
- Add one marker when long handlers run (`waitMove2Goal(...)`, `missionPlay(...)`, arm move/delay helpers) to correlate with telemetry drought windows.

### D) Decision Criteria For Root-Cause Attribution
Purpose: force a clean conclusion from the evidence instead of collecting more data without changing the diagnosis. This step is trying to decide which cause has actually been proven so the branch can move from investigation to a targeted fix.

- If setting interval gate to zero collapses multi-second gaps: classify as telemetry gate misconfiguration or unintended gate reapplication.
- If module/polling minimization collapses gaps while gate remains zero: classify as loop starvation from servo-bus polling.
- If gaps align with long handler execution windows: classify as command-handler blocking in shared loop.
- If none of the above explain gaps, escalate to deeper bus-layer timing/retry analysis despite flush-path improvements.

## Host-Side Parallel Mitigation (Do In Parallel, Not Instead)
Continue host-side redesign toward a continuous reader with receive timestamps and freshness tagging so late-but-valid replies are captured rather than discarded by pre-query flush windows.

## Exit Criteria for This Branch
This branch is complete when evidence identifies the firmware-side starvation or gating mechanism and a fix path is defined.

Minimum acceptance targets after fix validation:
- no recurring ~7s fresh-sample ceiling
- median inter-success gap below 0.5s in telemetry-only mode
- reply latency consistently below 500ms target window
- telemetry diversity reflects motion context (not single-value dominance in active runs)

## Notes for Future Agents
Start from this brief and avoid repeating already-completed host-side sweep work.
If new firmware evidence contradicts any "deprioritized" item above, record the contradiction explicitly before reopening that path.
Do not re-center the search on a deliberate 7 second telemetry limiter unless a concrete code path proves it.

## Step A Findings

- `feedbackFlowExtraDelay` is the only explicit firmware-side cadence gate I found. It is declared in `General_Driver/ugv_config.h` with a default of `0`, and the only writer found in the source tree is `setFeedbackFlowInterval(int)` in `General_Driver/ugv_advance.h`, which stores `abs(inputCmd)` directly. The command dispatcher routes `CMD_FEEDBACK_FLOW_INTERVAL` to that setter through `General_Driver/uart_ctrl.h`.
- The loop is fully synchronous. `General_Driver/General_Driver.ino` runs `serialCtrl()`, `server.handleClient()`, the module branch, command dispatch, motion PID, OLED refresh, IMU update, and `baseInfoFeedback()` on the same `loop()` thread with no task split or queue between them.
- The likely telemetry publication path for the sparse base replies is `baseInfoFeedback()`, which emits `FEEDBACK_BASE_INFO` and is called only when `baseFeedbackFlow` is enabled and the extra-delay gate has expired. Arm and gimbal paths refresh feedback state inline every loop, but they are also coupled to the same main loop and do not provide isolation from starvation.
- The servo feedback path is blocking by design. `getFeedback(...)` calls `st.FeedBack(...)`, which flows into `SCS::Read(...)` and `SCSerial::readSCS(...)`; that read loop waits for bytes until the configured `IOTimeOut` expires, so a missing or slow servo response can hold the loop for up to about 100 ms per read attempt.
- `moduleType_RoArmM2()` performs four servo feedback reads every loop iteration, and `moduleType_Gimbal()` performs two. That means the loop spends a meaningful amount of time on bus reads even before any command handler runs.
- There are several explicit blocking handlers in the same shared loop: `waitMove2Goal(...)` busy-waits with `delay(10)`, `RoArmM2_delayMillis(...)` calls `delay(inputTime)`, `missionPlay(...)` iterates mission steps synchronously via `moveToStep(...)`, and the arm/gimbal helper functions contain multiple fixed delays (for example `delay(1200)`, `delay(1000)`, `delay(5)`, and `delay(SERVO_STOP_DELAY)`).
- Boot-time source code does not set a non-zero feedback interval. `setup()` creates the `boot` mission and immediately runs `missionPlay("boot", 1)`, but the static source tree does not show any startup command that writes `CMD_FEEDBACK_FLOW_INTERVAL`. If a non-zero value appears at boot, it would have to come from runtime mission data or an out-of-band command, not from the visible startup code.

> Callout: the runtime contents of `/boot.mission` were not source-scannable in this audit. If that file already contains a `CMD_FEEDBACK_FLOW_INTERVAL` step on device, it would bypass this static-source conclusion and should be checked directly on the target firmware storage.

> Callout: the strongest verified explanation from step A is not a single "bad setting" in source, but a shared-loop design where servo feedback, mission playback, and long handlers can all block the same execution path that publishes telemetry.
