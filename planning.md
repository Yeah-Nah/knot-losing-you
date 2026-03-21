# Planning — knot-losing-you

Autonomous UGV that follows you around the boat park using OAK-D Lite and LiDAR.
No sea legs required.

---

## Phase 0 — Baseplate Setup ✅

> Establish repo structure, tooling, and coding conventions.

- [x] Repo structure mirroring `now-you-sea-me`
- [x] `pyproject.toml` with all dependencies
- [x] `.pre-commit-config.yaml` (ruff, mypy, nbstripout)
- [x] GitHub Actions CI (linting + validation)
- [x] `.claude/` context files (project overview, code standards)
- [x] Config YAML files (pipeline, model, sensor)
- [x] Source stubs for all modules (perception, inference, control, utils)

---

## Phase 1 — Hardware Integration

> Get real data flowing from each sensor on Raspberry Pi.

- [x] OAK-D Lite colour camera frames via DepthAI V3 (`CameraAccess`)
- [x] LiDAR scan data via serial (`LidarAccess`)
- [x] Waveshare UGV Rover basic drive commands (`UGVController`)
- [x] Verify sensor pipeline end-to-end with logging

---

## Phase 2 — Configurable Object Detection & Following

> Detect and track the target object; compute drive commands. Target class is configurable — default is `person` (COCO class `0`); swap in a custom model for jacket detection without changing code.

- [ ] YOLO detection with configurable target class (set via `model_config.yaml classes`; default: `0` = person)
- [ ] Support custom trained model via `model_config.yaml` (drop-in replacement — e.g. jacket detector trained externally)
- [ ] Bounding-box centroid → lateral error calculation
- [ ] LiDAR distance → range-to-target
- [ ] Proportional controller: lateral error + range → linear/angular velocity
- [ ] Integrate `Pipeline._main_loop()` with all modules
- [ ] Live view overlay (bounding box, track ID, distance)

---

## Phase 3 — Obstacle Avoidance

> Stop or steer around obstacles detected by LiDAR.

- [ ] LiDAR scan → nearest obstacle distance + bearing
- [ ] Emergency stop if obstacle within safety radius
- [ ] Basic avoidance steering (go-around logic)

---

## Phase 4 — Manual Controller Mode

> Allow the rover to be driven manually via the Waveshare gamepad controller. Autonomous and manual modes are exclusive — a dedicated button toggles between them.

> **Hardware note:** The Waveshare controller may communicate directly with the ESP32 sub-controller, bypassing the Pi entirely. Before implementing, verify whether the Pi can see the controller as a USB HID device (check `lsusb` / `ls /dev/input/`). If the controller talks only to the ESP32, we intercept at the Pi serial layer or remap a spare button to signal mode switching.

- [ ] Confirm controller connection path (USB HID to Pi vs. direct to ESP32 sub-controller)
- [ ] Read gamepad input on the Pi (`inputs` or `pygame` library)
- [ ] Dedicated button toggles **AUTONOMOUS ↔ MANUAL** mode (exclusive, not an override)
- [ ] Zero velocity before every mode transition (safety stop)
- [ ] Manual drive: map joystick axes → `UGVController` linear/angular velocity commands
- [ ] Log active mode clearly on each transition

---

## Phase 5 — Web Dashboard & Remote Control

> Lightweight web UI served from the Pi over its WiFi hotspot. Access from any phone or tablet via browser — no app install required. Primary use: toggle recording per camera when operating in the field without a laptop.

- [ ] FastAPI server running on boot (served over Pi WiFi hotspot)
- [ ] Mobile-friendly browser UI
- [ ] Toggle recording independently: Waveshare RGB camera and OAK-D Lite RGB camera
- [ ] Mode toggle: autonomous ↔ manual (mirrors the controller button)
- [ ] Status display: active mode, recording state per camera, battery level

---

## Phase 6 — Stretch Goals

> Nice-to-haves once the core loop is solid.

- [ ] OAK-D Lite depth data for closer-range obstacle sensing
- [ ] Target re-identification after occlusion
- [ ] MLflow experiment logging (confidence, latency, tracking metrics)
- [ ] Jupyter notebook for offline analysis of recorded sessions
- [ ] Extend web dashboard: live camera feed + telemetry overlay (bounding boxes, distance, confidence)
