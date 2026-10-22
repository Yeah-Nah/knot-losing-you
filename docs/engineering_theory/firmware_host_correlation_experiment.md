# Firmware-Host Correlation Experiment Theory

This note explains the technical core of the Issue 9 investigation in this project: why pan telemetry samples are sparse, and how to prove whether the bottleneck is firmware-side cadence limiting or host-side read misses.

---

## 0. Symbol Definitions

All symbols used in this document are defined here.

- $i$ - index of a host telemetry request cycle.
- $j$ - index of a firmware telemetry response.
- $q_i$ - the $i$th host telemetry request (for this project, a `T=130` request).
- $r_j$ - the $j$th telemetry response (for this project, a `T=1001` pan report).
- $s_i$ - sequence token attached to request $q_i$ (echoed by firmware in the matching response).
- $t_{send,i}$ - host timestamp when request $q_i$ is written to serial.
- $t_{rx,j}^{first}$ - host timestamp when first byte of response $r_j$ is received.
- $t_{parse,j}$ - host timestamp when response $r_j$ is fully parsed.
- $\Delta t_{i,j}$ - request-response delay, defined as $t_{rx,j}^{first} - t_{send,i}$.
- $W_i$ - host read window for request $q_i$ (from send time until timeout).
- $\tau$ - timeout duration used by the host read for one request.
- $g_k$ - gap between consecutive accepted telemetry successes.
- $\tilde{g}$ - median inter-success gap.
- $\rho$ - matched-sample ratio (fraction of accepted samples provably tied to same-cycle request).
- $f_{poll}$ - host poll rate in requests per second.
- $T_{poll}$ - host poll period, where $T_{poll}=1/f_{poll}$.
- $f_{fw}$ - effective firmware response cadence for telemetry.
- $C_{uniq}$ - count of distinct pan values observed over a run.

---

## 1. What a Correlation Experiment Is

A correlation experiment is a measurement experiment designed to answer this question:

Does event A reliably correspond to event B in time and identity?

For this project:

- Event A is a host telemetry request (`T=130`).
- Event B is an accepted pan telemetry response (`T=1001`).

The experiment is not only about "did some response eventually arrive?" It is about proving whether an accepted response belongs to the same control cycle request that just happened.

Why this matters:

- If responses are old/queued and not correlated, control can re-anchor on stale state.
- If responses are truly same-cycle and still sparse, the bottleneck is likely upstream cadence.

In short, this experiment separates freshness from mere availability.

---

## 2. How to Prove a Pan Sample Matches a Same-Cycle Request

Without request/response identity, proof is weak. With identity, proof is strong.

### 2.1 Minimum proof requirements

To call a sample "same-cycle matched," all conditions should hold:

1. Identity match: response includes echoed sequence token $s_i$ that matches request $q_i$.
2. Timing inclusion: first response byte lands inside the request window $W_i$.
3. One-to-one consistency: that same token is not reused for another in-flight request.

Formally, for response $r_j$ to match request $q_i$:

$$
\text{match}(q_i, r_j) \iff (s(r_j)=s_i) \land (t_{rx,j}^{first} \in W_i)
$$

Then define the matched-sample ratio:

$$
\rho = \frac{\#\{\text{accepted samples with proven same-cycle match}\}}{\#\{\text{all accepted samples}\}}
$$

Your current acceptance target from the Issue 9 notes is $\rho \ge 0.8$ (80%).

### 2.2 Why timing alone is not enough

If you only check "response arrived quickly," queue drain can fake success:

- a stale buffered packet can be read in less than 2 ms,
- but that packet may correspond to an earlier request.

That is exactly why sequence echo is the key gating mechanism.

### 2.3 Practical implementation sketch

1. Host sends `T=130` with a token $s_i$.
2. Firmware copies token $s_i$ into its `T=1001` response.
3. Host logs `send_ts`, `first_byte_ts`, `parse_ts`, token, and accept/reject reason.
4. Analysis script computes $\rho$, $\tilde{g}$, and distribution of $\Delta t$.

---

## 3. What Firmware Is and What Firmware-Side Cadence Limiting Means

Firmware is software running on the device microcontroller (the servo controller side), not on the host computer.

In this project, firmware-side cadence limiting means the device effectively emits telemetry at a bounded or bursty rate, regardless of host request rate.

Common reasons:

- Scheduler slotting: telemetry task runs only every fixed interval.
- Rate limiter: explicit cap such as "one telemetry reply every N milliseconds."
- Priority inversion: motor/control tasks preempt telemetry formatting/transmit.
- Serial buffering policy: telemetry is batched or delayed until a service window.

Why it can look like your data:

- Host poll rate changes do not materially improve success ratio.
- Long quasi-periodic gaps (for example near 7 s) remain across host settings.
- Responses may appear in short clusters when firmware service window opens.

In control terms, host is requesting at $f_{poll}$, but effective response cadence behaves closer to a smaller $f_{fw}$.

---

## 4. What Host-Side Misses Are and Why They Occur

A host-side miss means the firmware may have produced a useful response, but host logic failed to capture or correctly associate it for the intended cycle.

Typical mechanisms:

- Window miss: response arrives just after timeout $\tau$.
- Flush discard: pre-query flush removes delayed-but-valid packet before parsing.
- Strategy mismatch: strict request-then-read window misses asynchronous arrival patterns.
- Parse path miss: non-telemetry lines or framing issues prevent extraction.
- Concurrency timing: multiple traffic sources create unlucky read alignment.

This project has evidence of `lines_read=0` dominating many timeouts, which means no line arrived during that read window. That can be true no-reply, or a schedule alignment problem where replies exist but land outside the active capture window.

---

## 5. What a Host Read Strategy Is

A host read strategy is the policy used to acquire serial data, decide freshness, and map data to control cycles.

Three common strategies:

1. Strict synchronous query window.
The host sends one request and reads for $\tau$ only in that immediate window.

2. Query plus continuous reader.
A dedicated reader thread continuously consumes serial bytes, timestamps frames, and stores latest matched telemetry.

3. Hybrid strategy.
Continuous reader provides continuity; control cycle only accepts samples meeting freshness and correlation rules.

Given your findings, the long-term robust path is usually strategy 2 or 3 with explicit freshness tagging and sequence correlation.

---

## 6. The 7 s Success Gap: What It Means and Best Current Guess

### 6.1 Definition

The "7 s success gap" refers to recurring long intervals between accepted fresh telemetry samples:

$$
g_k = t_{success,k} - t_{success,k-1}
$$

with many runs showing $g_k \approx 7$ s at least intermittently.

### 6.2 Why this is important

- A 7 s gap is far too long for smooth pan feedback in a fast control loop.
- During that gap, controller depends on estimate/cached base, increasing re-anchor risk.
- It is a signature of cadence bottleneck, not simple noise.

### 6.3 Best current hypothesis (based on your logs)

Most plausible primary cause is an upstream cadence gate (firmware scheduler/rate-limit/service window), with host strategy effects layered on top.

Reasoning chain:

- Poll-rate increases did not substantially change success ratio in fixed-timeout matrix.
- Flush OFF boosts parsed-success counts but also shows queue-drain behavior and low value diversity.
- Long multi-second gaps persist even when parsed-success is high.

Interpretation:

- Host can improve capture probability of whatever is available.
- Host alone has not yet demonstrated elimination of the underlying long-gap cadence behavior.

This remains a hypothesis until sequence-echo correlation data confirms where each accepted packet originated in request time.

---

## 7. What You Are Actually Testing Next

You are testing a causality question, not just a performance question.

Core test objective:

Can we attribute accepted telemetry samples to the same-cycle requests with high confidence?

If yes, then evaluate whether cadence is still limited.
If no, then host strategy is still too ambiguous to diagnose root cause.

Concrete pass/fail framing tied to this project:

- Correlation pass: $\rho \ge 0.8$ for accepted samples.
- Cadence pass: $\tilde{g} < 0.5$ s in telemetry-only mode and no recurring near-7 s ceiling.
- Diversity sanity: $C_{uniq}$ should vary with real motion context, not remain effectively constant over long windows.

---

## 8. Extra Concepts That Help Interpret Results

### 8.1 Observability vs controllability in this context

- Controllability side: you can send pan commands frequently.
- Observability side: you can only stabilize well if angle observations are timely and attributable.

Issue 9 is primarily an observability bottleneck.

### 8.2 Why queue-drain success can mislead

A high success percentage can still be poor for control if samples are stale or weakly correlated to current cycle requests. Success count alone is not a control-quality metric.

### 8.3 Why one-value dominance is a red flag

If one pan value dominates nearly all successes while the system is active, that suggests either stale replay/queue reuse or extremely low effective measurement refresh.

### 8.4 Why edge timestamps are essential

For each request, logging `send_ts`, `first_byte_ts`, and `parse_ts` distinguishes:

- true no-reply,
- late-reply outside window,
- parse delay after bytes arrive.

Without edge timing, many failure modes collapse into the same "timeout" symptom.

---

## 9. Recommended Reading Order For This Repo

To connect theory to code and diagnostics, read in this order:

1. `docs/bug_fixing/pan_oscillation_issues.md` (Issue 9 findings and acceptance criteria).
2. `docs/engineering_theory/concurrency_and_telemetry_polling_primer.md` (threading and polling behavior).
3. `docs/engineering_theory/control_loop_rate_theory.md` (timing and loop-rate implications).
4. This document (correlation proof and root-cause interpretation framework).

---

## 10. Summary

The firmware-host correlation experiment is about proving attribution, not merely increasing apparent success rate.

- Sequence echo plus edge timing transforms the diagnosis from inference to evidence.
- If same-cycle attribution is strong but cadence remains sparse, focus firmware scheduler/rate limiting.
- If attribution is weak, redesign host read strategy around continuous capture plus freshness/correlation gating.

That is the key technical bridge from your current logs to a defensible root-cause conclusion for pan oscillation behavior.