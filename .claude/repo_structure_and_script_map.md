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
│  ├─ pan_oscillation_issues.md        (active investigation into pan servo oscillation)
│  ├─ pan_servo_feedback_investigation.md
│  ├─ engineering_theory/              (9 theory docs on calibration, control, coordinate systems)
│  └─ math_theory/                     (Laplace transform primer)
├─ other/
│  └─ images/
└─ ugv-follower/
   ├─ pyproject.toml
   ├─ pyrightconfig.json
   ├─ configs/
   │  ├─ calibration_config.yaml
   │  ├─ model_config.yaml
   │  ├─ pipeline_config.yaml
   │  └─ sensor_config.yaml          (central contract: calibration outputs → runtime params)
   ├─ calibration/
   │  └─ images/ (+ generated calibration outputs)
   ├─ model/
   │  └─ yolo11n.pt
   ├─ src/ugv_follower/
   │  ├─ pipeline.py                 (Phase 3A: mode/estop/transition-stop logic + thin autonomy)
   │  ├─ settings.py
   │  ├─ control/
   │  │  ├─ motion_command.py        (canonical MotionCommand contract + adapter)
   │  │  ├─ ugv_controller.py        (serial rover API + differential wheel solver)
   │  │  ├─ command_shaper.py        (rate-limit + reversal dwell background thread)
   │  │  └─ pan_controller.py        (bbox → pan command: bearing, tilt, deadband, clamp)
   │  ├─ perception/
   │  │  ├─ waveshare_camera.py      (V4L2 RGB pan-tilt; runtime frame source; preflight guard)
   │  │  ├─ lidar_access.py          (D500 serial packet parsing; 12-point scans)
   │  │  ├─ lidar_geometry.py        (polar→Cartesian; forward-arc filtering)
   │  │  └─ camera_access.py         (OAK-D; used only in check_camera_access.py smoke tool)
   │  ├─ inference/
   │  │  └─ object_detection.py      (YOLO scaffold; wired to Waveshare frame path)
   │  └─ utils/
   │     ├─ camera_preflight.py      (device holder detect/release; character-dev guard)
   │     ├─ fisheye_utils.py         (K/D load, pixel-to-bearing, undist-map cache)
   │     ├─ config_utils.py          (project-root detect, YAML load)
   │     └─ mjpeg_server.py          (thread-safe /stream HTTP JPEG encoder)
   ├─ tools/
   │  ├─ run_follower.py             (CLI entrypoint → pipeline)
   │  ├─ check_battery.py            (serial voltage telemetry; preflight guard on UART)
   │  ├─ check_camera_access.py      (OAK-D smoke test)
   │  ├─ check_lidar_access.py       (LiDAR packet + distance stats smoke test)
   │  ├─ check_ugv_controller.py     (rover move/stop sequence smoke test)
   │  ├─ check_pan_tilt_feedback.py  (NEW: pan-tilt servo feedback smoke test)
   │  ├─ scan_servo_ids.py           (NEW: servo ID discovery utility)
   │  └─ calibration/
   │     ├─ capture_calibration_images.py     (Pi MJPEG server + live corner detect)
   │     ├─ calibrate_waveshare_camera.py     (intrinsic fisheye; writes sensor_config)
   │     ├─ calibrate_pantilt_servo.py        (pan sweep; fit model; writes sensor_config + CSV)
   │     ├─ calibrate_angular_offset.py       (LiDAR↔pan-tilt extrinsic; writes sensor_config)
   │     └─ calibrate_ugv_drive.py            (turn-rate + dead-band; block-ordered sweep; writes sensor_config + CSV)
   └─ tests/                         (10 test modules; heavy pan/drive/mjpeg/camera coverage)
```

## 2) Runtime Architecture Snapshot

### Entrypoint & Config
- Runtime entrypoint: `ugv-run` → `tools/run_follower.py` → `Pipeline` class.
- `Settings` loads/validates three YAML files (pipeline, model, sensor) and resolves project-relative paths.
- Sensor config is the central contract: calibration tools write calibration outputs; `Settings` reads them at startup.

### Phase 3A Control Architecture (Implemented)
- **Motion Command Path**: All commands normalized through canonical `MotionCommand` contract (linear_m_s, angular_rad_s, source tag, timestamp).
  - Adapter: `apply_motion_command()` enforces validation and routes to `UGVController`.
  - Rate limiter: `CommandShaper` background thread enforces max wheel command rate and reversal dwell around zero crossing.
- **Mode Switching**: `PipelineMode.AUTONOMOUS` ↔ `PipelineMode.MANUAL` with enforced one-shot zero-velocity transition-stop command on each mode change.
- **Estop Latch**: `_estop_active` boolean latch; once asserted, forces `MotionCommand.zero(source=ESTOP)` between decision and apply; clear-estop allows normal commands to resume.
- **Autonomy Loop**: Currently thin and intentionally conservative (hold/zero commands + mode/estop sequencing). Suitable for expanded autonomous behaviours in later phases.
- **Command Logging**: Each emitted command logged with mode, estop state, source, velocities, and reason tags for post-run analysis.

### Sensor I/O & Pan Tracking
- **Frame Source**: Waveshare RGB pan-tilt camera (`WaveshareCamera`, V4L2, preflight-guarded).
  - Runs camera preflight (`ensure_camera_device_available`) on startup to detect/release busy V4L2 devices.
  - Feeds object detection, pan servo tracking, and MJPEG streaming.
- **Pan Servo Tracking**: `PanController` converts detection bbox centroids → pan servo commands.
  - Math: fisheye bearing + tilt correction + deadband suppression + clamp logic.
  - **Active Issue**: pan oscillation with stationary target; see `docs/pan_oscillation_issues.md` for investigation.
- **MJPEG Streaming**: `MjpegServer` provides thread-safe frame streaming at `/stream` (JPEG-encoded, daemon thread).
- **LiDAR Input**: D500 over serial; `lidar_access.py` parses packets (CRC, sync, angle interp); returns 12-point scans.
  - Geometry: `lidar_geometry.py` converts polar → Cartesian (body frame); provides `filter_forward_arc` and `nearest_forward_point` for forward-obstacle ranging.
- **Object Detection**: Placeholder YOLO wrapper scaffold; fully wired to Waveshare frame path when `inference_enabled: true`.
- **OAK-D Camera**: Retained in `camera_access.py` for `check_camera_access.py` smoke tool only; not used in runtime pipeline.

### UART/Serial Preflight
- Battery check (`check_battery.py`) and UGV controller use generic character-device preflight guards to detect/release busy UART holders (e.g., active follower pipeline).
- This ensures smoke tests can run without manually stopping the main follower or releasing V4L2/serial devices.

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
  - Runs character-device preflight to detect/release busy UART holders (e.g., active follower).
  - Attempts passive read first, then probes several command IDs if no passive telemetry.

- `tools/check_camera_access.py`
  - Hardware smoke script for headless camera frame retrieval and FPS logging.
  - Uses OAK-D camera (not Waveshare); tests DepthAI discovery and frame capture.

- `tools/check_lidar_access.py`
  - Hardware smoke script for LiDAR packet reads and basic distance stats.
  - Parses D500 packets; reports scan count and point distances.

- `tools/check_ugv_controller.py`
  - Hardware smoke script for rover serial connect/move/stop/disconnect sequence.
  - Basic motor sanity check before full follower runs.

- `tools/check_pan_tilt_feedback.py`
  - NEW: Hardware smoke script for pan-tilt servo feedback and position verification.
  - Tests servo ID discovery and position readback.

- `tools/scan_servo_ids.py`
  - NEW: Servo ID discovery utility for pan-tilt servos.
  - Scans servo bus to identify active servo addresses.

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
  - **Block-ordered sweep**: CCW block followed by CW block with per-block recenter pauses to reduce template noise.
  - **Known Issue**: directional dead-band asymmetry (CCW ~3.25 rad/s, CW ~4.0–4.5 rad/s on smooth concrete).
  - **Known Issue**: near-stall readings (|Δ_corrected| < 2°) corrupt OLS fit; minimum delta gate needed.
  - **Known Issue**: load-dependent yaw dead-band when wheels lifted vs on-ground; verify physical threshold before trusting vision-fit outputs.
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
- `docs/pan_oscillation_issues.md`: **ACTIVE**: investigation into stationary-target pan oscillation (intrinsic to control loop; Issues 2/3 are stronger candidates than Issue 1).
- `docs/pan_servo_feedback_investigation.md`: feedback path analysis and servo readback investigation.
- `docs/engineering_theory/*`: calibration/control theory docs (9 total; covers angular calibration, coordinate transforms, extrinsic/intrinsic calibration, servo curves, control loop rate theory, etc.).
- `docs/math_theory/*`: supporting math references (Laplace transform primer).

## 5) State of Build Maturity (Agent-Oriented)

### Mature/Stable
- **Calibration tooling**: Substantial and heavily exercised (camera intrinsic, pan-tilt, angular offset, drive model).
- **Hardware access layers**: Implemented and test-covered (camera, LiDAR, UGV controller, preflight guards).
- **Pan servo command generation**: Fully implemented with bearing, tilt correction, deadband, clamp logic; unit tested.
- **MJPEG streaming**: Thread-safe server with frame pooling and HTTP serving; unit tested.
- **Motion command contract & adapter**: Canonical interface for all commands; normalized source tagging and validation.
- **Phase 3A control shell**: Mode switching, estop latch, transition-stop, command logging all implemented and working.
- **Sensor config contract**: Central YAML-based bridge between calibration outputs and runtime parameters.

### Active Issues & Investigations
- **Pan oscillation with stationary target**: Confirmed intrinsic to control loop (not target motion). See `docs/pan_oscillation_issues.md`; Issues 2 and 3 are primary candidates (feedback delay, control gain).
- **UGV drive calibration challenges**:
  - Directional dead-band asymmetry (CCW vs CW thresholds differ by ~0.75–1.25 rad/s).
  - Near-stall readings corrupt OLS fit; quality gate insufficient.
  - Block-ordered sweep with recenter pauses reduces template noise but still noisy after ~4 steps.
  - Lifted vs on-ground yaw dead-band variance; physical threshold verification needed.

### Placeholder/Stub Areas
- **Object detection runtime**: YOLO wrapper skeleton exists; `run()` not yet implemented.
- **Autonomous navigation**: Currently intentionally thin (hold/zero + mode/estop); ready for expanded behaviours in future phases.
- **OAK-D pipeline**: Retained only for smoke test; not used in main follower (Waveshare RGB is primary frame source).

### Code Quality & Type Safety
- Project enforces Pyright strict type checking; see `.claude/pyright_type_hints_reference.md`.
- Linting via pre-commit hooks and CI workflow.
- Heavy unit test coverage on control, calibration, and perception components.
- Terminal sessions often start in `ugv-follower/` rather than repo root; use `pwd` before cd operations.

## 6) Helpful Context for Code Planning & Changes

### Import & Config Loading Patterns
- Use `config_utils.detect_project_root()` to find project root; store relative paths in YAML (not absolute).
- Calibration scripts use offline mode flags to skip camera preflight (e.g., replay/refit scenarios).
- `Settings` class provides typed accessors; always validate config presence at startup (see `pipeline.py` smoke checks).

### UART/Serial & Device Preflight
- V4L2 and serial UART both use preflight guards to detect/release busy device holders.
- This allows smoke tests to run without stopping the main follower; preflight will evict the holder if needed.
- Replay/offline modes skip preflight to allow multiple processes on same config.

### Calibration Output → Runtime Flow
- Calibration tools write results to `sensor_config.yaml` (intrinsic, extrinsic, drive model, servo model).
- `Settings` reads and validates these blocks at startup.
- Pan command generation and UGV controller both depend on sensor_config calibration outputs.
- Always test calibration output parsing and fallback values when refactoring.

### Pan Servo Oscillation & Control Loop
- Oscillation confirmed intrinsic (not target motion related).
- Primary suspects: feedback path latency, proportional gain, control loop timing.
- See `docs/pan_oscillation_issues.md` for detailed investigation; pan_controller.py includes extensive comments on tilt correction and deadband.

### UGV Drive Calibration Challenges
- Directional dead-band asymmetry is real and load-dependent; do not assume symmetric thresholds.
- Quality gate (`|Δ_corrected| < 2°`) insufficient; implement minimum delta gate to reject near-stall noise.
- Block-ordered sweep with recenter pauses helps reduce template drift; mid-block recenter or block-splitting needed beyond ~4 steps.
- Always validate on-ground before trusting calibration outputs; lifted testing gives false confidence.

### Loguru String Formatting
- Use `{}` placeholders in loguru messages, not `%s` or `%d`; incorrect formatting can break log output and hide status updates.
- Keep MJPEG overlay alive on cap.read() failures by reusing last-good frame (status changes remain visible).

### Test Coverage & Smoke Tests
- Heavy unit test coverage on control, pan controller, calibration fitting, and MJPEG server.
- Smoke tests (`check_*.py`) provide hardware sanity checks; all support preflight guards to co-exist with main follower.
- New smoke tools should follow preflight pattern (see `check_battery.py` and `check_camera_access.py` for examples).

## 7) Suggested Update Pattern For This File

When architecture evolves, update only these sections:

1. Section 1 if directories change.
2. Section 3 entries for added/removed scripts.
3. Section 5 maturity notes when major capabilities become active or issues are resolved.
4. Section 6 helpful context when new patterns emerge or pitfalls are discovered.

Keep per-script text to 1-3 bullets max to avoid large rewrites.
