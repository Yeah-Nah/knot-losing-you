# Project Overview — knot-losing-you

Reference document for AI assistant context. Describes the current state of the project.

---

## Project Summary

Autonomous UGV that follows a person around the boat park using an OAK-D Lite camera (RGB + depth) and
LiDAR for obstacle awareness. Runs on a Raspberry Pi mounted to a Waveshare UGV Rover. The Pi runs a
perception pipeline (YOLO person detection + depth estimation) and issues drive commands to the UGV motors
via the Waveshare serial SDK.

**Tech stack:**

| Layer | Technology |
|---|---|
| Hardware | Waveshare UGV Rover, Raspberry Pi, OAK-D Lite, LiDAR |
| OS | Ubuntu 22.04 (Pi) |
| Object Detection | YOLO (custom or COCO person class) |
| Computer Vision | OpenCV, DepthAI SDK |
| Language | Python 3.13 |
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
| `lidar.baud_rate` | `115200` | LiDAR serial baud rate |

---

## Module Responsibilities

### `settings.py` — `Settings`
- Loads all three YAML configs at init.
- Resolves relative paths to absolute paths anchored at the package root.
- Validates that the model file exists (if inference enabled) and that a display is available (if
  live view enabled).
- Exposes all config values as typed properties.

### `pipeline.py` — `Pipeline`
The central orchestrator. Wires all components together and owns the main processing loop.
- **Stub** — main loop not yet implemented.

### `perception/camera_access.py` — `CameraAccess`
OAK-D Lite hardware layer.
- Opens the DepthAI device, builds the pipeline, exposes colour frames and depth frames.
- **Stub** — DepthAI integration not yet implemented.

### `perception/lidar_access.py` — `LidarAccess`
LiDAR sensor layer.
- Opens the serial connection, starts the scan motor, exposes distance readings.
- **Stub** — LiDAR integration not yet implemented.

### `inference/object_detection.py` — `ObjectDetection`
YOLO model wrapper.
- Loads the model, runs `track()` per frame, returns detection results.
- **Stub** — same pattern as `now-you-sea-me`'s `ObjectDetection`.

### `control/ugv_controller.py` — `UGVController`
Waveshare UGV serial SDK wrapper.
- Opens serial connection, sends velocity commands (linear + angular).
- **Stub** — Waveshare SDK integration not yet implemented.

### `utils/config_utils.py`
- `get_project_root()`: resolves from `__file__` location (3 levels up from `src/utils/`).
- `load_yaml(path)`: loads and validates a YAML file, raises on missing or empty file.

---

## Key Design Decisions

- **Config-driven:** all hardware settings (resolution, ports, FPS) live in YAML — no hardcoded values.
- **Fail fast:** `Settings._validate()` checks all prerequisites at startup before any hardware is touched.
- **Modular separation:** perception, inference, and control are independent modules with clean interfaces.
- **Stub-first:** all hardware-layer classes raise `NotImplementedError` until implemented, making stubs
  safe to import and test around.
- **Headless support:** `live_view_enabled: False` in config disables the OpenCV window for Pi operation.

---

## Planned Next Steps

- Implement `CameraAccess` using DepthAI V3 API (follow `now-you-sea-me` pattern)
- Implement `LidarAccess` for connected LiDAR model
- Implement `UGVController` using Waveshare UGV Python SDK
- Wire `Pipeline._main_loop()` to drive the robot
- Train or adapt YOLO model for person following at the boat park
