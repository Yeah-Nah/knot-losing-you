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

- [ ] OAK-D Lite colour camera frames via DepthAI V3 (`CameraAccess`)
- [ ] LiDAR scan data via serial (`LidarAccess`)
- [ ] Waveshare UGV Rover basic drive commands (`UGVController`)
- [ ] Verify sensor pipeline end-to-end with logging

---

## Phase 2 — Person Detection & Following

> Detect and track the target person; compute drive commands.

- [ ] YOLO person detection on each frame (`ObjectDetection`)
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

## Phase 4 — Stretch Goals

> Nice-to-haves once the core loop is solid.

- [ ] OAK-D Lite depth data for closer-range obstacle sensing
- [ ] Target re-identification after occlusion
- [ ] MLflow experiment logging (confidence, latency, tracking metrics)
- [ ] Jupyter notebook for offline analysis of recorded sessions
- [ ] Web dashboard for live telemetry
