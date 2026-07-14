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
