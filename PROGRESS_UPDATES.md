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
