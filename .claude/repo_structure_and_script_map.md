# Knot-Losing-You: Repo Structure and Script Map

Purpose: compact, high-signal context for AI coding agents (Claude/Copilot) to understand project layout and script responsibilities without deep file-by-file archaeology.

## 1) Current Repository Structure (High Level)

```text
knot-losing-you/
├─ .claude/
│  ├─ code_standards.md
│  ├─ pyright_type_hints_reference.md
│  ├─ repo_structure_and_script_map.md
│  └─ settings.json
├─ .github/
│  ├─ .instructions.md
│  ├─ pull_request_template.md
│  ├─ prompts/               (create-claude-prompt, create-pr, create-progress-entry)
│  └─ workflows/
│     └─ linting_validation.yaml
├─ .gitignore
├─ .pre-commit-config.yaml
├─ README.md
├─ PROGRESS_UPDATES.md
├─ planning.md
├─ manual_validation_testing.md
├─ docs/
│  ├─ engineering_theory/
│  └─ math_theory/
├─ other/
│  └─ images/
└─ ugv-follower/
   ├─ pyproject.toml
   ├─ pyrightconfig.json
   ├─ configs/
   │  ├─ calibration_config.yaml
   │  ├─ model_config.yaml
   │  ├─ pipeline_config.yaml
   │  └─ sensor_config.yaml
   ├─ calibration/
   │  └─ images/ (+ generated calibration outputs)
   ├─ model/
   │  └─ yolo11n.pt
   ├─ src/ugv_follower/
   │  ├─ pipeline.py
   │  ├─ settings.py
   │  ├─ control/              (command_shaper, motion_command, pan_controller, ugv_controller)
   │  ├─ perception/           (camera_access, lidar_access, lidar_geometry)
   │  ├─ inference/
   │  └─ utils/                (camera_preflight, config_utils, fisheye_utils, mjpeg_server)
   ├─ tools/
   │  ├─ run_follower.py
   │  ├─ check_battery.py
   │  └─ calibration/
   └─ tests/                   (includes pan_controller, mjpeg_server, object_detection coverage)
```

## 2) Runtime Architecture Snapshot

- Runtime entrypoint is `ugv-run` -> `tools/run_follower.py`.
- `Settings` loads/validates YAML config and exposes typed properties.
- `Pipeline` wires camera, LiDAR, and UGV controller, then runs a thin safe control loop.
- Motion command path is normalized through `MotionCommand` and `apply_motion_command`.
- Pan tracking logic is factored into `PanController`; MJPEG frame serving is available via `MjpegServer`.
- Current autonomy logic is intentionally minimal (hold/zero commands + mode/estop sequencing).
- Runtime frame source is the Waveshare RGB pan-tilt camera (`WaveshareCamera` / V4L2); OAK-D (`OakdCamera`) is retained for the `check_camera_access.py` smoke tool only.
- Object detection (`ObjectDetection`) is fully wired to the Waveshare frame path; `run()` calls Ultralytics YOLO when `inference_enabled: true`.

## 3) Script Breakdown (All Python Files)

### 3.1 Package and Namespace Markers

- `src/ugv_follower/__init__.py`: package marker for core library.
- `src/ugv_follower/control/__init__.py`: control subpackage marker.
- `src/ugv_follower/inference/__init__.py`: inference subpackage marker.
- `src/ugv_follower/perception/__init__.py`: perception subpackage marker.
- `src/ugv_follower/utils/__init__.py`: utils subpackage marker.
- `tools/__init__.py`: tools package marker.
- `tools/calibration/__init__.py`: calibration tools package marker.
- `tests/__init__.py`: tests package marker.

### 3.2 Core Runtime Library (`src/ugv_follower`)

- `src/ugv_follower/pipeline.py`
  - Main orchestrator for camera/LiDAR/controller lifecycle.
  - Runs startup smoke checks (frame + scan availability).
  - Implements Phase 3A-lite control shell: mode switching, one-shot transition stop, estop override, thin-run scenario, and command logging.

- `src/ugv_follower/settings.py`
  - Loads `pipeline_config.yaml`, `model_config.yaml`, and `sensor_config.yaml`.
  - Resolves project-relative paths and validates key runtime constraints.
  - Provides structured accessors for UGV, camera, LiDAR, and calibration-derived parameters.

#### Control

- `src/ugv_follower/control/ugv_controller.py`
  - Serial transport and command API for Waveshare rover.
  - Converts linear/angular velocity to differential wheel speeds.
  - Supports optional command shaping and pan-tilt control commands.

- `src/ugv_follower/control/command_shaper.py`
  - Background thread that rate-limits wheel commands and enforces reversal dwell around zero crossing.
  - Contains pure `_step_wheel` logic that is heavily unit tested.

- `src/ugv_follower/control/motion_command.py`
  - Canonical motion command contract (`MotionCommand`, source tagging, validation).
  - Single adapter that applies validated commands to controller.

- `src/ugv_follower/control/pan_controller.py`
  - Converts detection bounding-box centroids into absolute pan servo commands.
  - Applies fisheye-derived bearing estimation, tilt correction, deadband suppression, and clamp logic.

#### Perception and Inference

- `src/ugv_follower/perception/camera_access.py`
  - OAK-D color camera startup/discovery and frame retrieval via DepthAI.
  - Used by `tools/check_camera_access.py` smoke script only; not used in the runtime follower pipeline.

- `src/ugv_follower/perception/waveshare_camera.py`
  - cv2.VideoCapture wrapper for the Waveshare RGB pan-tilt camera (V4L2).
  - Runtime frame source for the follower pipeline: inference, pan servo control, and MJPEG streaming.
  - Calls `camera_preflight.ensure_camera_device_available` on start.

- `src/ugv_follower/perception/lidar_access.py`
  - D500 packet parsing over serial (sync, CRC, angle interpolation, point extraction).
  - Returns 12-point scan packets; full scan aggregation is left to higher layers.

- `src/ugv_follower/perception/lidar_geometry.py`
  - Converts raw D500 polar coordinates into rover body-frame Cartesian coordinates.
  - Provides `filter_forward_arc` and `nearest_forward_point` for Phase 3 forward-obstacle ranging.
  - Bearing sign convention is consistent with `fisheye_utils.pixel_to_bearing_deg` (positive = right).

- `src/ugv_follower/inference/object_detection.py`
  - Placeholder YOLO wrapper scaffold.
  - Constructor/config path exists; `run()` is not yet implemented.

#### Utilities

- `src/ugv_follower/utils/config_utils.py`
  - Project-root detection and YAML loading helpers.

- `src/ugv_follower/utils/camera_preflight.py`
  - Camera device guardrail: detect holder processes, optionally release busy `/dev/video*`, and verify character-device validity.

- `src/ugv_follower/utils/fisheye_utils.py`
  - Fisheye calibration math helpers (load K/D, pixel-to-bearing transforms, undistortion map caching).
  - Shared by calibration tools and intended for future runtime heading estimation.

- `src/ugv_follower/utils/mjpeg_server.py`
  - Thread-safe HTTP MJPEG server that streams the latest pipeline frame at `/stream`.
  - Encodes pushed OpenCV frames to JPEG and serves them from a daemon thread.

### 3.3 Operational and Calibration Tools (`tools`)

- `tools/run_follower.py`
  - CLI entry for full follower pipeline.
  - Loads settings, instantiates pipeline, handles config-related startup failures.

- `tools/check_battery.py`
  - Serial listener/prober for rover telemetry with voltage interpretation.
  - Attempts passive read first, then probes several command IDs.

- `tools/check_camera_access.py`
  - Hardware smoke script for headless camera frame retrieval and FPS logging.

- `tools/check_lidar_access.py`
  - Hardware smoke script for LiDAR packet reads and basic distance stats.

- `tools/check_ugv_controller.py`
  - Hardware smoke script for controller connect/move/stop/disconnect sequence.

#### Calibration Tools

- `tools/calibration/capture_calibration_images.py`
  - Runs Pi-hosted MJPEG server for checkerboard capture.
  - Detects corners live and only saves frames when valid corners are present.
  - Zeros pan-tilt before capture and exposes `/stream`, `/capture`, `/status`.

- `tools/calibration/calibrate_waveshare_camera.py`
  - Offline intrinsic fisheye calibration from checkerboard images.
  - Validates calibration quality and writes `waveshare_rgb` block into `sensor_config.yaml`.

- `tools/calibration/calibrate_pantilt_servo.py`
  - Interactive/server-based pan sweep calibration.
  - Measures commanded pan vs observed angle (with forward-offset correction).
  - Fits piecewise/linear characteristics, estimates hysteresis/dead-band, writes CSV + sensor config model.

- `tools/calibration/calibrate_angular_offset.py`
  - Interactive extrinsic calibration between LiDAR forward axis and camera/pan-tilt zero.
  - Uses camera alignment + LiDAR cluster centroid around forward arc.
  - Writes `extrinsic.lidar_to_pantilt_offset_deg` to sensor config.

- `tools/calibration/calibrate_ugv_drive.py`
  - Interactive turn-rate/dead-band calibration for rover drive model.
  - Uses clicked target bearings with fisheye correction + camera offset correction.
  - Produces gain/dead-band estimates and writes `ugv_drive` calibration outputs to sensor config plus CSV.

### 3.4 Test/Validation Scripts (`tests`)

- `tests/test_command_shaper.py`
  - Unit + integration tests for shaper state machine and controller interaction.

- `tests/test_motion_command.py`
  - Unit tests for motion command contract validation and pipeline command-path behaviour.

- `tests/test_object_detection.py`
  - Unit tests for object-detection wrapper configuration and placeholder runtime behaviour.

- `tests/test_pan_controller.py`
  - Unit tests for heading sign, tilt correction, deadband handling, clamp logic, and pan command updates.

- `tests/test_calibrate_pantilt_servo.py`
  - Unit tests for pan-tilt calibration math/fit/quality/csv/config helper functions.

- `tests/test_calibrate_ugv_drive.py`
  - Unit tests for drive calibration geometry, parsing, fitting, and orchestrator helper behaviour.

- `tests/test_lidar_geometry.py`
  - Unit tests for `lidar_geometry.py` helpers: `wrap_360`, `wrap_180`, `lidar_point_to_body_frame`, `filter_forward_arc`, and `nearest_forward_point`.

- `tests/test_mjpeg_server.py`
  - Unit tests for MJPEG frame storage, server lifecycle, and `/stream` versus 404 HTTP responses.

- `tests/test_waveshare_camera.py`
  - Unit tests for `WaveshareCamera`: pre-start guard, start/stop lifecycle, frame acquisition, and read-failure handling (mocked cv2.VideoCapture).

## 4) Non-Script Context Files Worth Keeping in View

- `README.md`: operator workflow, setup, and calibration runbook.
- `planning.md`: implementation roadmap and phase progress.
- `PROGRESS_UPDATES.md`: execution log / milestone narrative.
- `manual_validation_testing.md`: manual validation procedures.
- `docs/engineering_theory/*`: calibration/control rationale and assumptions.
- `docs/math_theory/*`: supporting math references.

## 5) State of Build Maturity (Agent-Oriented)

- Calibration tooling is substantial and appears to be the most mature area.
- Core hardware access layers are implemented and test-covered.
- Pan servo command generation and MJPEG streaming infrastructure are implemented and unit tested.
- Runtime autonomy loop is intentionally conservative (safe hold path + mode/estop behaviour).
- Runtime frame source is the Waveshare RGB pan-tilt camera; detection and pan tracking are fully wired to it.
- Sensor config is the central contract between calibration outputs and runtime behaviour.

## 6) Suggested Update Pattern For This File

When architecture evolves, update only these sections:

1. Section 1 if directories change.
2. Section 3 entries for added/removed scripts.
3. Section 5 maturity notes when major capabilities become active.

Keep per-script text to 1-3 bullets max to avoid large rewrites.
