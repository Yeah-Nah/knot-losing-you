# Pan Telemetry Serial Worker Crash

**Status:** Root cause found and fixed on the rover (2026-10-22). The code hardening below is still open.
**Evidence:** [logs/post_background_grabber_thread.jsonl](logs/post_background_grabber_thread.jsonl) (failing run), [logs/post_port_release.jsonl](logs/post_port_release.jsonl) (run after the fix).

## Symptom

During the `ugv-run` test that checked the background camera grabber (commit `bc390cb`), the person at the edge of the frame was detected correctly but the pan motor never moved.

The `PanTelemetrySerialWorker` thread crashed on its first `T=130` query, about 1 ms after it started:

```
serial.serialutil.SerialException: device reports readiness to read but returned
no data (device disconnected or multiple access on port?)
```

- `_serial_worker_loop` has no exception handling, so the exception killed the thread silently.
- With no thread updating it, the telemetry snapshot stayed `"initialising"` for the rest of the run.
- `Pipeline._update_pan_state` returns early on `"initialising"`, so the pan controller never ran and no `T=133` was sent (126 of 126 cycles logged `holding pan position`).

The camera change didn't cause this.

## Root cause

Waveshare's stock app, `~/ugv_rpi/app.py`, starts at boot from the `ws` user's crontab:

```
@reboot XDG_RUNTIME_DIR=/run/user/1000 ~/ugv_rpi/ugv-env/bin/python ~/ugv_rpi/app.py >> ~/ugv.log 2>&1
```

After a fresh boot, `fuser -v /dev/ttyAMA0 /dev/video0` showed this one process (PID 857) holding **both** the UART and the camera. Two processes reading `/dev/ttyAMA0` is exactly what the `SerialException` describes. The PID 848 that the camera preflight killed in the failing run was the same app from an earlier boot. The preflight released `/dev/video0` with `fuser -k`, but by then the serial worker had already died.

No systemd unit, root crontab entry or autostart `.desktop` file starts anything else that touches these devices. The Jupyter server started by the second `@reboot` line (`start_jupyter.sh`) doesn't hold either device.

## Fix applied on the rover

```bash
# Comment out the app.py @reboot line (leaves the Jupyter line alone)
crontab -l | sed 's|^\(@reboot .*ugv_rpi/app\.py.*\)$|# \1|' | crontab -

# Stop the running instance. It ignores SIGTERM, so pkill alone isn't enough.
pkill -f ugv_rpi/app.py
kill -9 <pid>

# Check: ttyAMA0 should have no holders; only pipewire/wireplumber on video0
fuser -v /dev/ttyAMA0 /dev/video0
```

To restore the Waveshare app later, uncomment the line with `crontab -e`. If a Waveshare notebook ever opens the serial port from Jupyter, it will cause the same conflict.

## Verification

[logs/post_port_release.jsonl](logs/post_port_release.jsonl), 2026-10-22 11:46–11:47, with a person starting at the edge of the frame and then walking to the other side:

- The serial worker ran for the whole ~45 s run: **873 of 873** `T=130` queries answered, no serial errors, no stale or degraded mode.
- Query round trip: median 15 ms, max 63 ms. Fresh sample interval: median 50 ms (the tracking cadence), max 93 ms.
- Every control cycle used `base=…(measured-fresh)`.
- The pan moved to the person at the edge of the frame (1° → 29° → 54° → 58°, limited by the slew cap) and settled centred at the end.

**Side effect: the `T=130` timeouts are gone.** The 2026-10-21 run in [pan_oscillation_vision_latency.md](pan_oscillation_vision_latency.md) had 158 of 350 queries (45%) time out. This run had none. That earlier run was almost certainly also sharing the UART with `app.py`, which would read and discard some of our replies. The remaining pan oscillation is covered in that doc.

## Remaining hardening (code)

These make sure a port conflict can never again silently disable pan control.

- [ ] **Check the serial port before opening it.** In `UGVController.connect()` ([ugv_controller.py](../../ugv-follower/src/ugv_follower/control/ugv_controller.py)), call `ensure_character_device_available()` on the UGV port before `serial.Serial(...)`. Fail fast with the holder's PID and command line, or release it. Today only the camera is checked, and only after the telemetry poller has already started.
- [ ] **Make the serial worker survive errors.** In `_serial_worker_loop` ([pan_telemetry.py](../../ugv-follower/src/ugv_follower/control/pan_telemetry.py)), wrap `query_pan_deg()` in `try/except (serial.SerialException, OSError)`. On an error, log a WARNING, back off briefly and keep looping. A port fault then makes telemetry go `stale` and then `expired` instead of killing the thread.
- [ ] **Make a dead feed obvious.** In `Pipeline._update_pan_state` ([pipeline.py](../../ugv-follower/src/ugv_follower/pipeline.py)), log a single WARNING/ERROR when telemetry stays `initialising` for more than a few seconds, or when the poller's thread is no longer alive. Today it is a DEBUG line on every frame, which is easy to miss.
- [ ] **Add unit tests** for the cases above:
  - [ ] the worker keeps running after `query_pan_deg` raises
  - [ ] `connect()` refuses or releases a busy port
  - [ ] the pipeline warns when telemetry stays `initialising`
- [ ] **Reject out-of-range startup readings.** The first query after homing returned `-179.96°` (`lines_read=4`), before the servo had reported its homed position. It arrived before the pipeline used telemetry, so it did no harm, but readings outside the mechanical pan range should be discarded.
- [ ] **(Optional) Narrow the camera preflight release.** `fuser -k` kills every process holding the camera, including pipewire and wireplumber, which systemd restarts every run. Either kill only our own python processes, or stop pipewire from claiming the camera.
