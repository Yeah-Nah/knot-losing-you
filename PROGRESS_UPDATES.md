# Progress Updates — knot-losing-you

Progress is logged chronologically here — one entry per meaningful milestone.

---

<!-- Add entries below in the format:

## Entry N: Title
*Date: Month DD, YYYY*

Short description of what was done and why.

**Section Header:**
- Bullet point 1
- Bullet point 2

**Key Achievements:**
- First achievement
- Second achievement

-->

# Phase 2: Sensor Calibration

## Entry 6: UGV Drive Calibration Finally Complete
*Date: April 20, 2026*

Finally FINALLY got the rover drive calibration working end-to-end and locked down the parameters needed for reliable yaw-rate control. This turned into a much bigger job than expected because the rover's servo motors were inconsistent from run to run, which made the whole thing a pain in the butt and forced a lot of work on sweep structure, recentering, camera robustness, and command shaping before the measurements were trustworthy. 🥲😭

**What was done:**
- Built and iterated on `calibrate_ugv_drive.py` into a browser-driven calibration workflow that measures real turn response, dead-band, and drive gains, then writes the results into `sensor_config.yaml`
- Reworked the sweep procedure several times to handle unreliable motor behaviour, including directional CW/CCW passes, recenter pauses, mid-block recentering, manual moved-centre confirmation, and better stall/retry handling
- Hardened the camera measurement path with preflight checks, retry/reopen logic, and fisheye/undistortion support so calibration could survive dropouts and still produce usable bearing measurements
- Added command shaping plus controller/config updates so the calibrated drive behaviour can be applied more smoothly at runtime, not just measured offline
- Wrote the supporting theory and validation docs for rover drive calibration, control strategy, and the latest manual testing results

**Key Achievements:**
- UGV drive calibration is now finally sorted, with track width, surface-specific tuning, dead-band, and angular velocity gain parameters stored in config for downstream follower control
- Reached a calibration workflow that is resilient enough to use on the real rover despite the servo motor inconsistencies
- Added broad automated test coverage for the drive calibration and command shaping paths, reducing the risk of breaking the workflow during future control work
- Removes one of the last major actuator unknowns before the full follower loop in Phase 3

Updated the stream to have a better centre marker:

<img src="other/images/Screenshot 2026-04-19 155054.png" alt="Testing Camera Connection" width="500">

---

## Entry 5: Pan-Tilt Servo Curve Calibration Complete
*Date: April 1, 2026*

Built and hardened the pan-tilt servo curve calibration workflow so pan commands can now be converted into real physical angles using measured data, not assumptions. This closes the actuator-modeling gap needed for reliable angle-based tracking in Phase 3.

**What was done:**
- Built `calibrate_pantilt_servo.py` as a full headless HTTP calibration workflow (stream, confirm, status, abort, save, reset) with sweep execution and state-managed progress reporting
- Implemented offset-corrected angle measurement using camera intrinsics plus forward-offset geometry, so measured pan response accounts for the camera lens being physically forward of the pan axis
- Added config-driven sweep and acquisition controls in `calibration_config.yaml` (sweep bounds, settle timing, averaging, noise floor, geometry terms), including `precondition_cycles` and `precondition_settle_time_s` to warm the mechanism before recording
- Added offline replay/refit support from saved CSV runs, plus expanded validation around sweep data quality, monotonic model generation, and save-time schema consistency
- Landed a large test expansion for the servo calibration path and refactored calibration/test layout into dedicated module folders for cleaner maintenance

**Key Achievements:**
- Servo calibration now writes a full pan response model to `sensor_config.yaml`, including dead-band (`+5.0` / `-5.0` deg), hysteresis (`2.4364` deg mean), linear fit metrics (MAE `1.525` deg, max abs error `2.9681` deg), and piecewise forward/reverse curves
- Verified usable measured pan range captured from approximately `-36.43` deg to `+38.10` deg across the commanded `-45` to `+45` sweep
- Warmup/preconditioning behavior was parameterized and optimized, reducing time-to-calibration while preserving measurement-phase settle timing
- Phase 2 pan-tilt actuator calibration step complete, with runtime-ready model artifacts stored in config for downstream control work

The super high tech set up for the pan tilt calibration. Aka a broom taped to my balcony to align to the cameras optical axis.

<img src="other/images/Screenshot 2026-04-01 220152.png" alt="Testing Camera Connection" width="500">

---

## Entry 4: LiDAR–Camera Angular Offset Calibration Complete
*Date: March 25, 2026*

Built and verified the angular offset calibration tool — the second step in Phase 2. The LiDAR and pan-tilt camera now share a common angular reference frame, so a bearing measured in the camera frame can be converted to a LiDAR bearing with a single addition.

**What was done:**
- Built `calibrate_angular_offset.py`, a headless HTTP server that streams a live annotated camera feed to a browser while simultaneously driving the LiDAR and UGV pan-tilt
- The camera feed overlays a vertical guide line at `cx` (the optical centre from intrinsic calibration, not the pixel midpoint), so the operator aligns the calibration target to the true optical axis rather than the geometric centre of the frame
- A state machine (WAITING_ALIGNMENT → SCANNING → COMPLETE / FAILED) controls the workflow; the operator confirms alignment via a `/confirm` endpoint, at which point the script accumulates LiDAR returns in the forward arc for a configurable dwell period and computes the angular centroid as the median signed angle to avoid wrap-around artefacts near 0°
- Added `mounting_offset_deg: 270` to the LiDAR config to account for the physical mounting direction — on this chassis the LiDAR's 0° beam faces the left side of the rover, so all bearings must be rotated into the rover forward frame before use
- Wrote reference documentation covering the theory: the angular offset is a restricted special case of the full 6-DOF extrinsic calibration, valid because the LiDAR scans a single horizontal plane and the follower only needs to convert bearings, not project 3D points into pixel coordinates

**Key Achievements:**
- LiDAR-to-pan-tilt angular offset measured at **2.8°** from 414 LiDAR cluster points and stored in `sensor_config.yaml` under the `extrinsic` key
- LiDAR mounting offset correctly characterised and added to config so all downstream code works in the rover forward frame
- Entire calibration workflow runs headlessly on the Pi over a browser, consistent with the intrinsic calibration tooling
- Extrinsic calibration step of Phase 2 complete ✅

Extrinsic calibration of the angular offset between the LiDAR and Waveshare pan tilt camera:

<img src="other/images/Screenshot 2026-03-25 210703.png" alt="Testing Camera Connection" width="500">

---

## Entry 3: Waveshare RGB Camera Intrinsic Calibration Complete
*Date: March 23, 2026*

Built and verified the full intrinsic calibration pipeline for the Waveshare RGB camera — the first step in Phase 2. The camera can now report where things actually are in the image, correcting for lens distortion and pixel geometry.

**What was done:**
- Built a headless image capture tool that streams a live annotated feed to a laptop browser — no screen needed on the Pi
- Built a calibration tool that processes the captured images and writes the resulting camera parameters directly into the sensor config
- Wrote reference documentation covering the underlying camera geometry

**Key Achievements:**
- Waveshare RGB camera fully characterised — focal lengths, principal point, and distortion coefficients measured and stored in `sensor_config.yaml`
- Entire calibration workflow runs headlessly on the Pi, operated from a laptop browser
- Intrinsic calibration step of Phase 2 complete ✅

Screenshot of the intrinsic camera calibration process:

<img src="other/images/Screenshot 2026-03-22 192017.png" alt="Testing Camera Connection" width="500">

---

# Phase 0/1: Baseplate Setup and Hardware Integration

## Entry 2: All Three Sensors Verified & End-to-End Smoke Test Passing
*Date: March 21, 2026*

Implemented and verified standalone connection scripts for the OAK-D Lite camera, LDRobot D500 LiDAR, and Waveshare UGV Rover, then wired them together into a single end-to-end smoke test via `run_follower.py`.

**What was done:**
- Each sensor tested independently via a dedicated script before integration
- All three confirmed operational on the Pi: camera streaming at 1080p @ 30fps, LiDAR parsing packets with CRC validation, rover responding to drive commands
- Smoke test (`run_follower.py`) connects all sensors in sequence and confirms all hardware operational in a single command; includes timeouts on sensor wait loops to fail fast with a clear message if a device is unplugged or misconfigured

**Key Achievements:**
- All three sensors independently verified on the Raspberry Pi before integration
- Unified smoke test confirms end-to-end hardware chain: `python run_follower.py`
- Phase 1 — Hardware Integration fully complete ✅

Log from running smoke test to connect to all sensors:

<img src="other/images/Screenshot 2026-03-21 204358.png" alt="Testing Camera Connection" width="500">


## Entry 1: Hardware Confirmed & Dev Environment Stood Up
*Date: March 20, 2026*

Physically assembled the rover with all three sensors attached. Confirmed all devices are detected by the Raspberry Pi via `lsusb` and stood up the Python dev environment with pre-commit passing cleanly.

**Physical Hardware Setup:**
- Assembled Waveshare UGV rover with OAK-D Lite, LDRobot D500 LiDAR, and HD RGB camera attached
- Confirmed all devices detected on Pi via `lsusb`: OAK-D Lite (Intel Movidius MyriadX), LiDAR (Silicon Labs CP210x), HD RGB camera (Xitech), UGV base controller (QinHeng CH341)
- Identified D500 LiDAR baud rate as 921600; updated `sensor_config.yaml` accordingly

**Key Achievements:**
- All three sensors physically connected and detected by the Pi before writing a single line of sensor code
- Clean pre-commit baseline established — ruff, mypy, and file checks all green

The "Nauti-Bot" in question (yes I am keeping to the nautical theme):

<img src="other/images/20260323_175328.JPG" alt="Testing Camera Connection" width="500">

Connecting to the rover via the Waveshare UI and testing functionality:

<img src="other/images/Screenshot 2026-03-20 190359.png" alt="Testing Camera Connection" width="500">
