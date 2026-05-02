# Control Loop Rate Theory - Reference

Covers the timing theory behind the main follower control loop used in this project.
Focus: how the nominal loop rate of 10 Hz is obtained from the code, why that is only an idealised rate,
and how to calculate the real cycle time once blocking work such as camera reads and inference are included.

---

## 0. Symbol Definitions

Key symbols used throughout this document:

- **$f$** - loop frequency or loop rate, measured in hertz (Hz), meaning cycles per second.
- **$T$** - loop period or cycle time, measured in seconds, meaning the time taken for one full loop iteration.
- **$T_{\text{sleep}}$** - fixed sleep time inserted at the end of each loop iteration.
- **$T_{\text{cycle}}$** - total real cycle time for one complete pass through the loop.
- **$T_{\text{lidar}}$** - time spent reading and processing LiDAR data during one iteration.
- **$T_{\text{cmd}}$** - time spent deciding and applying the chassis motion command during one iteration.
- **$T_{\text{cam}}$** - time spent acquiring a camera frame during one iteration.
- **$T_{\text{inf}}$** - time spent running object-detection inference on the frame during one iteration.
- **$T_{\text{pan}}$** - time spent updating the pan controller and issuing the pan command during one iteration.
- **$T_{\text{stream}}$** - time spent annotating and pushing a frame to the MJPEG stream during one iteration.
- **$n$** - number of loop iterations.
- **$t$** - time, usually measured in seconds.
- **Hz** - hertz, the unit for frequency; 1 Hz means one cycle per second.

The most important relationship is:

$$
f = \frac{1}{T}
$$

where:

- **$f$** is frequency in Hz.
- **$T$** is period in seconds per cycle.

This equation says frequency and period are reciprocals: if one goes up, the other goes down.

---

## 1. What "10 Hz" Means

When we say the main follower control loop runs at **10 Hz**, we mean it completes about:

$$
10 \text{ cycles/second}
$$

That is equivalent to saying each cycle takes about:

$$
T = 0.1 \text{ s} = 100 \text{ ms}
$$

because:

$$
f = \frac{1}{T} = \frac{1}{0.1} = 10 \text{ Hz}
$$

where:

- **$f$** is the loop rate in Hz.
- **$T$** is the loop period in seconds.
- **0.1 s** is 100 milliseconds.

In words: a 10 Hz loop is a loop that would ideally repeat once every 100 ms.

---

## 2. Where The 10 Hz Value Comes From In This Project

In this repo, the 10 Hz figure comes from the main pipeline code, not from YAML configuration.

The runtime entrypoint constructs `Pipeline`, and the pipeline initialiser sets a fixed loop sleep period:

$$
T_{\text{sleep}} = 0.1 \text{ s}
$$

Using the reciprocal relationship gives the nominal loop rate:

$$
f_{\text{nominal}} = \frac{1}{T_{\text{sleep}}} = \frac{1}{0.1} = 10 \text{ Hz}
$$

where:

- **$T_{\text{sleep}}$** is the fixed programmed sleep time per loop iteration.
- **$f_{\text{nominal}}$** is the nominal loop rate implied by that sleep time alone.

This is why people naturally describe the loop as "10 Hz": the code explicitly inserts a 100 ms delay at the end of each iteration.

However, that is only the **nominal** or **idealised** rate. It is not the true loaded runtime rate.

---

## 3. Why The Nominal Rate Is Not The Real Rate

The loop does not only sleep. Before the sleep happens, it also performs work.

In broad terms, one iteration of the follower loop does the following:

1. Update LiDAR state.
2. Decide and apply the chassis command.
3. Read a camera frame.
4. Run inference on the frame.
5. Update pan control from the detection result.
6. Push an annotated frame to the stream output.
7. Sleep for 100 ms.

That means the real cycle time is not just the sleep time. It is the sum of all loop work plus the sleep:

$$
T_{\text{cycle}} = T_{\text{lidar}} + T_{\text{cmd}} + T_{\text{cam}} + T_{\text{inf}} + T_{\text{pan}} + T_{\text{stream}} + T_{\text{sleep}}
$$

where:

- **$T_{\text{cycle}}$** is the total real time for one full loop iteration.
- **$T_{\text{lidar}}$** is LiDAR read and processing time.
- **$T_{\text{cmd}}$** is motion-command decision and application time.
- **$T_{\text{cam}}$** is camera frame acquisition time.
- **$T_{\text{inf}}$** is inference time.
- **$T_{\text{pan}}$** is pan-update time.
- **$T_{\text{stream}}$** is stream/annotation time.
- **$T_{\text{sleep}}$** is the fixed sleep time.

This equation is the key timing model for the real control loop.

---

## 4. The Real Loop Rate

Once the true cycle time is known, the real loop rate is:

$$
f_{\text{real}} = \frac{1}{T_{\text{cycle}}}
$$

where:

- **$f_{\text{real}}$** is the actual loop frequency under load.
- **$T_{\text{cycle}}$** is the full measured cycle time.

This matters because the controller does not act at the nominal 10 Hz unless all of the other work takes negligible time.

If the loop work is expensive, then $T_{\text{cycle}}$ grows and $f_{\text{real}}$ falls.

---

## 5. Worked Examples

### Example A - Idealised 10 Hz Case

Suppose the loop work took effectively zero time and only the programmed sleep mattered:

$$
T_{\text{cycle}} \approx T_{\text{sleep}} = 0.1 \text{ s}
$$

Then:

$$
f_{\text{real}} \approx \frac{1}{0.1} = 10 \text{ Hz}
$$

where:

- **$T_{\text{cycle}}$** is approximately equal to the sleep time.
- **$f_{\text{real}}$** is therefore approximately 10 Hz.

This is the best-case interpretation of the hard-coded 100 ms sleep.

### Example B - Moderately Loaded Loop

Suppose the loop timing is:

- $T_{\text{lidar}} = 0.005 \text{ s}$
- $T_{\text{cmd}} = 0.002 \text{ s}$
- $T_{\text{cam}} = 0.015 \text{ s}$
- $T_{\text{inf}} = 0.080 \text{ s}$
- $T_{\text{pan}} = 0.003 \text{ s}$
- $T_{\text{stream}} = 0.005 \text{ s}$
- $T_{\text{sleep}} = 0.100 \text{ s}$

Then:

$$
T_{\text{cycle}} = 0.005 + 0.002 + 0.015 + 0.080 + 0.003 + 0.005 + 0.100 = 0.210 \text{ s}
$$

and so:

$$
f_{\text{real}} = \frac{1}{0.210} \approx 4.76 \text{ Hz}
$$

So even though the code contains a 100 ms sleep, the real loop is now running at only about 4.8 cycles per second.

### Example C - Heavier Pi Inference Load

If inference becomes slower, say:

$$
T_{\text{inf}} = 0.250 \text{ s}
$$

and the rest stays roughly similar, then:

$$
T_{\text{cycle}} \approx 0.005 + 0.002 + 0.015 + 0.250 + 0.003 + 0.005 + 0.100 = 0.380 \text{ s}
$$

which gives:

$$
f_{\text{real}} = \frac{1}{0.380} \approx 2.63 \text{ Hz}
$$

This is why a loop that is casually described as "10 Hz" can, in practice, operate closer to 2 to 5 Hz when running full perception on embedded hardware.

---

## 6. Why Real Cycle Time Matters For Control

Control quality depends heavily on how quickly the system can observe, decide, and act.

If $T_{\text{cycle}}$ is small, then new commands are issued frequently and the controller reacts to fresher information.
If $T_{\text{cycle}}$ is large, every decision is based on older state.

In a vision-based pan loop, the controller is effectively doing this:

1. Observe target position in the image.
2. Compute the heading error.
3. Send a pan correction.
4. Wait until the next loop to observe the result.

If the next observation comes late, the controller may be reacting to a scene that is already out of date.
That increases the chance of overshoot and oscillation.

This is why timing is part of control design, not just a software performance detail.

---

## 7. Relation To Per-Cycle Pan Limits

In this project, pan motion is limited partly by a maximum change per cycle, denoted here as:

$$
\Delta \theta_{\max}
$$

where:

- **$\Delta \theta_{\max}$** is the maximum allowed pan change in one loop iteration, usually in degrees per cycle.

If the controller can change pan by at most $\Delta \theta_{\max}$ each cycle, then the maximum approximate pan speed is:

$$
\dot{\theta}_{\max} \approx \Delta \theta_{\max} \cdot f_{\text{real}}
$$

where:

- **$\dot{\theta}_{\max}$** is the approximate maximum pan speed in degrees per second.
- **$\Delta \theta_{\max}$** is the maximum pan step in degrees per cycle.
- **$f_{\text{real}}$** is the actual loop rate in cycles per second.

If we use the nominal 10 Hz figure and a per-cycle cap of $1.25^\circ$/cycle, then:

$$
\dot{\theta}_{\max} \approx 1.25 \times 10 = 12.5^\circ/\text{s}
$$

But if the real loop rate is only 4 Hz, then the same per-cycle cap gives:

$$
\dot{\theta}_{\max} \approx 1.25 \times 4 = 5^\circ/\text{s}
$$

So the real loop rate directly changes how aggressive or sluggish the pan system feels.

---

## 8. Nominal Timing Versus Measured Timing

It is useful to distinguish three different ideas:

### Configured or Programmed Timing

This is what the software asks for, such as:

$$
T_{\text{sleep}} = 0.1 \text{ s}
$$

This gives a **nominal** timing target.

### Device Timing

This is what individual hardware devices try to do, such as camera frame rate:

$$
f_{\text{camera}} = 30 \text{ Hz}
$$

where:

- **$f_{\text{camera}}$** is the camera capture rate in frames per second.

This is not the same as the control-loop rate. The camera may produce 30 frames per second while the control loop only consumes a few of them.

### Measured Runtime Timing

This is what the system actually achieves when running. The clean way to compute an average measured loop rate over $n$ iterations is:

$$
\bar{T}_{\text{cycle}} = \frac{t_{\text{end}} - t_{\text{start}}}{n}
$$

$$
\bar{f}_{\text{real}} = \frac{1}{\bar{T}_{\text{cycle}}}
$$

where:

- **$\bar{T}_{\text{cycle}}$** is the average measured cycle time.
- **$\bar{f}_{\text{real}}$** is the average measured loop frequency.
- **$t_{\text{start}}$** is the wall-clock time when measurement begins.
- **$t_{\text{end}}$** is the wall-clock time when measurement ends.
- **$n$** is the number of completed loop iterations.

This is the most honest description of runtime loop performance.

---

## 9. Main Takeaways

1. The often-quoted 10 Hz value comes from the fixed 100 ms sleep in the main pipeline loop.
2. That value is nominal, not guaranteed.
3. The real control-loop cycle time is the sum of all loop work plus the fixed sleep.
4. The real loop rate is the reciprocal of that real cycle time.
5. Slower real loop rates reduce responsiveness and increase effective sensing-to-command delay.
6. Any per-cycle command limit becomes a per-second speed limit only after multiplying by the real loop rate.

---

## 10. Practical Interpretation For This Project

For this follower pipeline, a useful mental model is:

$$
\text{nominal loop rate} \neq \text{actual loop rate under load}
$$

The nominal rate tells you what the software delay alone implies.
The actual rate tells you how often the controller is really observing and correcting.

For pan stability, the second quantity is the one that matters most.