# Progress Updates — knot-losing-you

> Autonomous UGV that follows you around the boat park using OAK-D Lite and LiDAR.
> No sea legs required.

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

Connecting to the rover via the Waveshare UI and testing functionality:

<img src="other/images/Screenshot 2026-03-20 190359.png" alt="Testing Camera Connection" width="500">
