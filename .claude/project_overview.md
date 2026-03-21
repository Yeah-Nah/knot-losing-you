# Project Overview — knot-losing-you

Reference document for AI assistant context. Describes the current state of the project.

---

## Project Summary

Autonomous UGV that follows a person around the boat park using an OAK-D Lite camera (RGB + depth) and
LiDAR for obstacle awareness. Runs on a Raspberry Pi 5 mounted to a Waveshare UGV Rover (6-wheel 4WD).
The Pi runs a perception pipeline (YOLO person detection + depth estimation) and issues drive commands to
the UGV motors via a JSON serial protocol to the ESP32 sub-controller.

**Tech stack:**

| Layer | Technology |
|---|---|
| Hardware | Waveshare UGV Rover (6WD), Raspberry Pi 5, OAK-D Lite, LDRobot D500 LiDAR |
| OS | Raspberry Pi OS Bookworm (headless) |
| Object Detection | YOLO (custom or COCO person class) |
| Computer Vision | OpenCV (headless), DepthAI V3 |
| Language | Python 3.11 |
| Version Control | Git |

---

## Repo Structure

```
knot-losing-you/
├── .claude/
│   ├── project_overview.md     ← this file
│   └── code_standards.md       ← coding conventions reference
├── .github/
│   ├── pull_request_template.md
│   └── workflows/
│       └── linting_validation.yaml
├── ugv-follower/               ← installable Python package
│   ├── run_follower.py         ← CLI entry point
│   ├── pyproject.toml
│   ├── configs/
│   │   ├── pipeline_config.yaml   ← feature toggles
│   │   ├── model_config.yaml      ← YOLO settings
│   │   └── sensor_config.yaml     ← OAK-D Lite + LiDAR settings
│   ├── models/                    ← YOLO .pt / .onnx files (gitignored)
│   ├── output/                    ← recordings and logs (gitignored)
│   └── src/
│       ├── __init__.py
│       ├── pipeline.py            ← main orchestrator
│       ├── settings.py            ← config loading + validation
│       ├── perception/
│       │   ├── __init__.py
│       │   ├── camera_access.py   ← OAK-D Lite hardware layer
│       │   └── lidar_access.py    ← LiDAR sensor layer
│       ├── inference/
│       │   ├── __init__.py
│       │   └── object_detection.py ← YOLO wrapper
│       ├── control/
│       │   ├── __init__.py
│       │   └── ugv_controller.py  ← Waveshare UGV serial SDK wrapper
│       └── utils/
│           ├── __init__.py
│           └── config_utils.py    ← YAML loading, project root resolution
├── .gitignore
├── .pre-commit-config.yaml
├── planning.md                 ← full phase plan
├── PROGRESS_UPDATES.md
├── LICENSE
└── README.md
```

---

## Entry Point

`run_follower.py` — parses CLI args for config paths (defaults to `configs/*.yaml`), constructs `Settings`,
constructs `Pipeline`, calls `pipeline.run()`.

```
python run_follower.py
python run_follower.py --pipeline-config configs/pipeline_config.yaml
                       --model-config    configs/model_config.yaml
                       --sensor-config   configs/sensor_config.yaml
```

---

## Configuration

### `pipeline_config.yaml`

| Key | Default | Effect |
|---|---|---|
| `dev_or_pi` | `"dev"` | Target runtime environment flag |
| `inference_enabled` | `true` | Run YOLO detection on frames |
| `live_view_enabled` | `true` | Show camera feed in OpenCV window |
| `recording_enabled` | `false` | Write video frames to disk |
| `output_dir` | `"output/"` | Output directory (relative to package root) |

### `model_config.yaml`

| Key | Default | Effect |
|---|---|---|
| `model` | `"yolo11n.pt"` | Model filename (resolved from `models/`) |
| `conf` | `0.5` | Detection confidence threshold |
| `classes` | `[0]` | Class IDs to detect (0 = person in COCO) |
| `persist` | `true` | Enable multi-frame tracking |
| `verbose` | `false` | Suppress YOLO console output |

### `sensor_config.yaml`

| Key | Default | Effect |
|---|---|---|
| `camera.colour_resolution` | `[1920, 1080]` | OAK-D Lite colour output resolution |
| `camera.fps` | `30` | Target frame rate |
| `lidar.port` | `"/dev/ttyUSB0"` | LiDAR serial port |
| `lidar.baud_rate` | `230400` | LiDAR serial baud rate (D500 = 230400) |
| `ugv.port` | `"/dev/ttyAMA0"` | UGV sub-controller serial port (Pi 5) |
| `ugv.baud_rate` | `115200` | UGV sub-controller baud rate |
| `ugv.chassis_main` | `2` | Chassis type: 1=RaspRover, 2=UGV Rover 6WD, 3=Beast tracked |
| `ugv.chassis_module` | `0` | Module type: 0=none, 1=arm, 2=pan-tilt |
| `ugv.track_width` | `0.3` | Wheel centre-to-centre distance in metres |

---

## Module Responsibilities

### `settings.py` — `Settings`
- Loads all three YAML configs at init.
- Resolves relative paths to absolute paths anchored at the package root.
- Validates that the model file exists (if inference enabled) and that a display is available (if
  live view enabled).
- Exposes all config values as typed properties, including all UGV, camera, and LiDAR sensor parameters.

### `pipeline.py` — `Pipeline`
The central orchestrator. Wires all components together and owns the main processing loop.
- `__init__`: instantiates `UGVController`, `CameraAccess`, and `LidarAccess` from `Settings`.
- `_main_loop()`: smoke-test sequence — connects UGV, starts camera and LiDAR, waits for first frame
  and first scan, logs confirmation of all sensors operational, then idles until Ctrl+C.
- `_shutdown()`: stops camera, stops LiDAR, disconnects UGV — each in an independent try/except.

### `perception/camera_access.py` — `CameraAccess`
OAK-D Lite hardware layer using DepthAI V3 API.
- `start()`: discovers colour camera socket (`CameraSensorType.COLOR`), builds DepthAI pipeline,
  opens device, creates output queue.
- `get_frame()`: returns the latest colour frame as a NumPy array (BGR, shape `(1080, 1920, 3)`),
  or `None` if no frame is ready. Uses `queue.has()` + `queue.get().getCvFrame()` (V3 pattern).
- `stop()`: closes the DepthAI device.
- **Verified working on Pi**: 1080p @ 30fps confirmed.

### `perception/lidar_access.py` — `LidarAccess`
LDRobot D500 binary packet parser.
- LDRobot LD06/LD19 binary streaming protocol at 230400 baud — no start command required.
- 47-byte packets: `0x54 0x2C` header, 12 distance/angle/intensity readings, CRC-8/MAXIM checksum.
- `start()` / `stop()`: open and close the serial connection.
- `get_scan()`: reads one 47-byte packet, verifies CRC, interpolates angles, returns a list of 12
  `{"angle": float, "distance": int, "intensity": int}` dicts. Returns `None` on CRC error.
- **Implemented; not yet tested on Pi.**

### `inference/object_detection.py` — `ObjectDetection`
YOLO model wrapper.
- Loads the model, runs `track()` per frame, returns detection results.
- **Stub** — not yet implemented.

### `control/ugv_controller.py` — `UGVController`
Waveshare UGV ESP32 sub-controller serial wrapper.
- JSON newline-terminated protocol at 115200 baud over `/dev/ttyAMA0`.
- `connect()`: opens serial port, sends chassis-type initialisation command `{"T":900,"main":2,"module":0}`.
- `move(linear, angular)`: differential drive → `L = linear − angular × track_width / 2`,
  sends `{"T":1,"L":…,"R":…}`. ESP32 auto-stops after ~4 s of silence.
- `stop()`: sends zero velocities.
- `disconnect()`: stops motors, closes serial port.
- **Verified working on Pi**: rover drives.

### `utils/config_utils.py`
- `get_project_root()`: resolves from `__file__` location (3 levels up from `src/utils/`).
- `load_yaml(path)`: loads and validates a YAML file, raises on missing or empty file.

---

## Key Design Decisions

- **Config-driven:** all hardware settings (resolution, ports, FPS, track width) live in YAML — no hardcoded values.
- **Fail fast:** `Settings._validate()` checks all prerequisites at startup before any hardware is touched.
- **Modular separation:** perception, inference, and control are independent modules with clean interfaces.
- **Headless-first:** `opencv-python-headless` is used (not `opencv-python`) — Pi OS Bookworm has no display
  server; `live_view_enabled: false` in config enforces this.
- **DepthAI V3 API:** uses `getAllAvailableDevices()`, `CameraSensorType.COLOR`, `has()` + `get()` queue
  pattern (not V2's `tryGet()`).
- **LiDAR protocol:** raw binary CRC-8/MAXIM parsing, no external SDK dependency.
- **UGV heartbeat:** the ESP32 sub-controller auto-stops motors after ~4 s of silence — the pipeline must
  issue periodic `move()` calls or an explicit `stop()` on shutdown.

---

## Hardware Notes

- **UGV serial**: `app.py` on the rover's Pi runs at boot via crontab and holds `/dev/ttyAMA0`. Must be
  disabled (`crontab -e`, comment out the `app.py` line, reboot) before running custom code.
- **OAK-D Lite udev**: USB vendor `03e7`; rule required on first setup:
  `echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules`
  then `sudo udevadm control --reload-rules && sudo udevadm trigger`, replug camera.
- **Python environment**: Pi OS Bookworm is externally-managed — always use the project venv:
  `cd ~/knot-losing-you/ugv-follower && python3 -m venv .venv && source .venv/bin/activate && pip install -e .`

---

## Current Status

| Component | Status |
|---|---|
| `UGVController` | ✅ Implemented & verified — rover drives |
| `CameraAccess` | ✅ Implemented & verified — 1080p @ 30fps |
| `LidarAccess` | ✅ Implemented — not yet tested on Pi |
| `Pipeline` (smoke test) | ✅ Implemented — not yet run end-to-end |
| `ObjectDetection` | ⬜ Stub — not yet implemented |

---

## Planned Next Steps

- Run end-to-end smoke test: `python run_follower.py` on Pi — expect all three sensors to confirm operational
- Test `LidarAccess` standalone: `python test_lidar_access.py` on Pi
- Phase 2: person detection and following — YOLO inference, bounding box centroid, LiDAR distance,
  proportional velocity controller wired into `Pipeline._main_loop()`
- Train or adapt YOLO model for person following at the boat park
