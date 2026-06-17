# Concurrency and Telemetry Polling Primer

## Purpose of This Document

This guide explains the core ideas behind concurrency, telemetry polling, and control-loop timing in a robotics-style project. It starts with basic terms and builds up to practical design patterns you can use in this codebase.

If you are new to robotics, embedded systems, or control software, read this from top to bottom once. Then use the glossary sections as a reference when specific terms come up in future chats.

---

## Part 1: Base Concepts

### 1.1 What Is a Program?

A program is a sequence of instructions that a computer executes. In robotics software, the program often has to do many things repeatedly, such as:

- read sensors,
- process camera frames,
- run control logic,
- send motor or servo commands,
- log status.

### 1.2 What Is a Loop?

A loop is repeated execution of a block of code.

In this project, common loops include:

- vision loop (capture frame -> infer target -> compute control),
- telemetry loop (query pan angle -> update latest sample),
- command loop (send actuator commands at a fixed or near-fixed cadence).

### 1.3 What Is a Thread?

A thread is one execution path inside a process.

- Single-threaded program: one path of execution.
- Multi-threaded program: multiple paths of execution running concurrently.

Why threads are used:

- to keep one task from blocking another task,
- to increase responsiveness,
- to separate responsibilities.

### 1.4 What Is Concurrency?

Concurrency means multiple tasks make progress over overlapping time.

Important distinction:

- Concurrency: tasks overlap in time.
- Parallelism: tasks literally run at the same instant on different CPU cores.

You can have concurrency even without true parallelism.

### 1.5 Blocking vs Non-Blocking

Blocking call:

- the calling thread waits until the operation finishes or times out.

Non-blocking call:

- the call returns quickly; completion is checked later.

A blocking sensor read inside a critical loop can reduce responsiveness.

### 1.6 What Is Latency?

Latency is delay from request to response.

Examples:

- time from sending a pan query to receiving pan angle,
- time from camera frame capture to control command output.

### 1.7 Throughput vs Latency

- Throughput: how much work is completed per unit time.
- Latency: how long one specific operation takes.

A system may have high throughput but still bad latency for a specific task.

---

## Part 2: Robotics and Control Context

### 2.1 What Is Telemetry?

Telemetry is measurement data reported by hardware/software components.

For pan control, telemetry might include:

- measured pan angle,
- timestamp,
- validity/freshness indicators.

### 2.2 What Is Polling?

Polling means repeatedly asking for data.

A poll cycle looks like:

1. send query,
2. wait for response,
3. parse response,
4. store latest value,
5. repeat.

Polling rate is how often this cycle runs.

### 2.3 What Is Serial I/O?

Serial I/O is communication over a serial link (for example UART or USB-serial).

- "I/O" means Input/Output.
- In serial communication, data is sent as a stream of bytes over time.
- Typical operations are write bytes, then read bytes.

In practice, a pan/tilt controller often uses serial I/O commands for query and control.

### 2.4 What Is a Timeout?

A timeout is a maximum wait duration for an operation.

Example:

- query pan with timeout 0.1 seconds.
- if no valid response by 0.1s, return timeout result (for example None).

Timeout protects the loop from waiting forever.

### 2.5 Control Loop and Inference Loop

- Inference loop: processes camera frames and detections.
- Control loop: converts state and target error into actuator commands.

Sometimes these are one combined loop, sometimes separated.

### 2.6 Fresh vs Stale Telemetry

- Fresh telemetry: recent enough for reliable control decisions.
- Stale telemetry: too old to trust for normal control behavior.

Freshness often uses age threshold:

- telemetry_age = now - sample_timestamp.

If telemetry_age exceeds threshold, degrade behavior.

---

## Part 3: Why Telemetry Can Be "Starved"

### 3.1 Starvation (in this context)

Starvation means an important task does not run often enough because other work dominates scheduling time.

If pan polling is inside a heavy vision loop:

- frame/inference/post-processing delays can reduce poll regularity,
- telemetry may arrive too infrequently,
- control may rely on stale data.

### 3.2 Key Insight About Timeouts

Inside one synchronous call, there is usually no extra work between send and wait. But starvation can still occur by:

- delaying when the next query is issued,
- causing irregular spacing between queries,
- increasing chance that responses are read late or out of cadence.

So there are two delay types:

- device-side latency (device responds slowly),
- scheduling-side delay (software asks too late or too irregularly).

---

## Part 4: Decoupling Telemetry Polling From Vision Loop

### 4.1 What Decoupling Means

Decoupling means telemetry polling runs independently of vision processing.

Typical architecture:

- Vision/control thread reads latest telemetry snapshot.
- Telemetry polling thread owns serial queries and keeps snapshot updated.

### 4.2 What It Solves

- prevents vision workload from directly delaying telemetry query schedule,
- reduces blocking impact on the control path,
- improves regularity of telemetry updates,
- makes freshness tracking cleaner.

### 4.3 What It Does Not Solve

- it does not make a slow device respond faster,
- it does not remove the need for degraded behavior when telemetry is old,
- it does not automatically prevent concurrency bugs.

---

## Part 5: Concurrency Complexities (and What They Mean)

### 5.1 Threading or Async Coordination

Meaning:

- multiple execution paths must be started, supervised, and stopped correctly.

Typical failure modes:

- one thread dies silently,
- shutdown hangs because thread never exits,
- error in one path is not propagated to others.

### 5.2 Shared-State Synchronization

Meaning:

- two or more threads access the same data.

Risk:

- one thread reads half-updated data,
- inconsistent state causes wrong control decisions.

### 5.3 Race Conditions

Meaning:

- correctness depends on unpredictable timing/order of operations.

Example:

- writer updates angle then timestamp separately,
- reader sees new angle with old timestamp,
- freshness logic becomes wrong.

### 5.4 Poll Thread Running While Control Is Idle

Meaning:

- independent polling may continue when tracking/control is inactive.

Risk:

- unnecessary serial traffic,
- unnecessary CPU work,
- noisy logs.

---

## Part 6: Guardrails to Negate Concurrency Risk

### 6.1 Guardrail: Clear Ownership

Rule:

- one owner per responsibility.

Example:

- telemetry thread owns serial read/query path,
- control thread consumes snapshots only.

Benefit:

- avoids hidden contention and duplicated logic.

### 6.2 Guardrail: Explicit Lifecycle

Rule:

- define start/stop/join and error handling behavior.

Checklist:

- stop event for cooperative shutdown,
- join with timeout,
- heartbeat or health status,
- log reason for thread exit.

### 6.3 Guardrail: Immutable Telemetry Snapshot

Rule:

- package related fields into one record and publish atomically.

Suggested fields:

- pan_deg,
- sample_time_monotonic,
- seq,
- source,
- status.

Benefit:

- prevents mixed-field reads.

### 6.4 Guardrail: Lock or Queue Discipline

Rule:

- protect shared state with a lock, or use a queue with clear semantics.

Good practice:

- keep lock hold times short,
- avoid nested locks,
- avoid ad-hoc writes from many modules.

### 6.5 Guardrail: Read-Once Per Control Cycle

Rule:

- at cycle start, copy one telemetry snapshot and use only that copy for the cycle.

Benefit:

- deterministic behavior within each cycle.

### 6.6 Guardrail: Freshness States

Rule:

- classify telemetry into fresh/stale/expired and define behavior for each.

Example policy:

- fresh: normal control,
- stale: limited gains/rate,
- expired: fail-safe or hold behavior.

### 6.7 Guardrail: Mode-Aware Poll Rates

Rule:

- adjust poll cadence by system mode.

Example:

- tracking mode: faster polling,
- initialising mode: moderate polling,
- idle mode: slower polling.

Benefit:

- reduces unnecessary load while preserving responsiveness when needed.

### 6.8 Guardrail: Backoff on Repeated Timeouts

Rule:

- if repeated timeouts occur, increase poll interval temporarily.

Benefit:

- avoids hammering serial bus and reduces useless work.

### 6.9 Guardrail: Bounded Buffers/Queues

Rule:

- use small queue sizes for latest-state data.

Example:

- size 1 latest-value queue with overwrite/drop-old behavior.

Benefit:

- prevents backlog-induced latency.

### 6.10 Guardrail: Late Reply Handling

Rule:

- identify stale replies and ignore them.

Approach:

- use sequence ids or age checks,
- discard responses older than allowed control age window.

### 6.11 Guardrail: Instrumentation and Metrics

Rule:

- measure what matters.

Useful metrics:

- poll timeout rate,
- median and 95th percentile response time,
- telemetry age at control tick,
- control loop jitter.

Benefit:

- gives objective evidence for tuning decisions.

---

## Part 7: Project-Specific Design Pattern

This is a practical shape suitable for this repository.

### 7.1 Single Serial Arbiter Pattern

Concept:

- one module/thread is the only place that touches serial for pan telemetry.

Other code:

- sends requests to that module,
- consumes published snapshots,
- never does direct serial reads in parallel.

Why this helps:

- eliminates multi-writer serial contention,
- centralizes timeouts/retry/parsing logic,
- simplifies debugging.

### 7.2 Snapshot Data Contract

Define one telemetry snapshot structure and treat it as the interface boundary.

Suggested contract fields:

- pan_deg: float,
- sample_time_monotonic: float,
- seq: int,
- valid: bool,
- status: string (fresh/stale/expired/error),
- age_s: float (computed at read time or attached at publish).

### 7.3 State-Machine Integration

Tie telemetry freshness into controller state transitions.

Example states:

- initialising,
- tracking,
- degraded,
- hold/failsafe.

Transition examples:

- if age_s > stale_threshold: tracking -> degraded,
- if age_s <= fresh_threshold for N samples: degraded -> tracking.

### 7.4 Mode-Aware Polling and Idle Behavior

Use explicit mode table:

- tracking: short poll interval,
- initialising: medium interval,
- idle/lost target: long interval.

This avoids wasting resources while preserving needed responsiveness.

---

## Part 8: Common Misconceptions

### Misconception 1: "If timeout happens, the loop must have interrupted the waiting call."

Reality:

- a synchronous blocking call usually waits uninterrupted in that thread.
- loop effects are often before/after the call (query schedule irregularity), not inside the blocking wait.

### Misconception 2: "Decoupling always reduces device latency."

Reality:

- decoupling does not speed up the device response.
- it improves system responsiveness and scheduling isolation.

### Misconception 3: "More threads always means better performance."

Reality:

- more threads can increase complexity and bug risk.
- design quality matters more than thread count.

---

## Part 9: Practical Validation Plan

When testing architecture changes, measure and compare before/after.

### 9.1 Baseline Measurements

Collect:

- timeout frequency,
- response latency distribution,
- telemetry freshness distribution,
- control oscillation indicators,
- frame/inference cycle timing.

### 9.2 After Decoupling

Check:

- did telemetry age at control tick improve?
- did timeout rate change?
- did control stability improve?
- did frame processing cadence remain healthy?

### 9.3 Decision Rule

If telemetry remains too stale even after decoupling:

- root cause is likely device/protocol side,
- focus next on protocol reliability, command cadence, timeout tuning, and hardware path.

---

## Part 10: Quick Reference Glossary

- Concurrency: overlapping progress of multiple tasks.
- Thread: execution path inside a process.
- Blocking call: call that waits until completion/timeout.
- Latency: delay from request to response.
- Polling: repeated querying for updated data.
- Serial I/O: byte-stream communication over serial link.
- Telemetry: measured system data (for example pan angle).
- Timeout: maximum wait duration before giving up.
- Freshness: recency of telemetry relative to now.
- Stale data: data too old for normal control.
- Race condition: timing-dependent bug from unsynchronized access.
- Shared state: data accessible by multiple threads.
- Lock: synchronization primitive to protect shared state.
- Queue: thread-safe message passing buffer.
- Snapshot: one coherent set of telemetry fields.
- Jitter: variation in loop timing.
- Degraded mode: safer reduced-authority control behavior under uncertainty.
- Serial arbiter: single owner of serial operations.
- Backoff: temporary slowdown after repeated failures/timeouts.

---

## Part 11: Study Path (Recommended)

If these concepts are new, study in this order:

1. loops, blocking calls, and latency,
2. telemetry polling and serial I/O basics,
3. threading and shared-state fundamentals,
4. race conditions and synchronization patterns,
5. control-loop freshness/degraded-state design,
6. project-specific architecture (serial arbiter + snapshot + mode-aware polling).

This order builds understanding from foundational computer science concepts to practical robotics-control software design.
