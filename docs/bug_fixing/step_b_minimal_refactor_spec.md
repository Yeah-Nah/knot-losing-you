# Step B Minimal Refactor - Planning Handoff Spec

## Why This Exists

Step B experiments need to be reproducible from one command invocation of `ugv-check-pan-telemetry-only`, without manual serial scripting before the run starts.

Today, `tools/check_pan_telemetry_only.py` can run telemetry-only polling and optional module init, but it cannot:

1. Apply experiment setup commands (for example gate=0 / steady-mode off) immediately before measurement.
2. Produce a deterministic outbound command trace showing exactly what was sent and when.

This spec defines a minimal, low-risk refactor so a planning/implementation agent can add those capabilities while preserving default behavior.

## Problem Statement

The current Step B workflow has two repeatability gaps:

1. Pre-run setup is often performed manually outside the tool, so run-to-run setup drift is possible.
2. Existing logs focus on RX outcomes (`--log-jsonl`) and summary stats, but there is no structured TX evidence for attribution when investigating reapply behavior.

Result: attribution confidence is weaker than desired when comparing gate/steady/reapply experiments.

## Goal (Definition Of Done)

Enhance `ugv-check-pan-telemetry-only` so a single command can:

1. Apply optional pre-commands before the timed measurement loop.
2. Optionally record every outbound command (`init`, `pre_cmd`, `query`) to JSONL with monotonic timestamps.

Done means:

1. New flags work as documented.
2. Existing invocations without new flags remain behaviorally unchanged.
3. Unit tests cover both new behavior and default-path preservation.

## Relevant Files And Ownership

Primary implementation target:

1. [ugv-follower/tools/check_pan_telemetry_only.py](../../ugv-follower/tools/check_pan_telemetry_only.py)

Primary test target:

1. [ugv-follower/tests/test_check_pan_telemetry_only.py](../../ugv-follower/tests/test_check_pan_telemetry_only.py)

Context docs (read-only, reference only):

1. [docs/engineering_theory/firmware_host_correlation_experiment.md](../engineering_theory/firmware_host_correlation_experiment.md)
2. [docs/bug_fixing/issue9_firmware_schedule_investigation.md](issue9_firmware_schedule_investigation.md)
3. [docs/bug_fixing/pan_oscillation_issues.md](pan_oscillation_issues.md)

Out of scope file changes:

1. No required updates to other tool scripts.
2. No required protocol mapping updates in this repo.

## Current Behavior Baseline (Must Be Preserved)

Current run sequence in `run(...)`:

1. Open serial via `_open_serial(...)`.
2. Optional init via `_maybe_init(...)` (`T=900` then buffer reset).
3. Timed polling via `_run_measurement_loop(...)`, which calls `_send_and_await_pan(...)`.

Current direct TX write sites (to be unified):

1. `_maybe_init(...)` writes init payload directly.
2. `_send_and_await_pan(...)` writes query payload directly.

Critical preservation rule:

1. If none of the new flags are used, default operational behavior and existing metrics/output intent must remain unchanged.

## In Scope

1. Add pre-command CLI inputs.
2. Add optional TX JSONL logging.
3. Route existing TX writes through a shared wrapper helper.
4. Add tests for sequence ordering, fail-fast validation, and default-path safety.

## Out Of Scope

1. No serial protocol redesign.
2. No background reader architecture changes.
3. No firmware command discovery or payload hardcoding beyond validating generic JSON command shape (`T` integer).
4. No mandatory changes to existing RX result JSONL schema (`--log-jsonl`).

## Assumptions And External Dependencies

1. Firmware command IDs and payload fields for gate/steady controls are defined in the firmware repository, not here.
2. This refactor must accept arbitrary valid command payloads and not enforce command-specific schema beyond `T` being an integer.
3. Existing serial transport expectations stay unchanged (newline-delimited JSON writes over the same UART settings).
4. Existing CI/test environment may not have hardware; verification relies primarily on unit tests and mocked serial behavior.

## Required CLI Additions

1. `--pre-cmd-json TEXT`
2. `--pre-cmd-jsonl PATH`
3. `--pre-cmd-delay-ms FLOAT` (default `100`)
4. `--tx-log-jsonl PATH`

Behavioral intent:

1. `--pre-cmd-json` sends one command before measurement starts.
2. `--pre-cmd-jsonl` sends one command per line, in file order.
3. If both are supplied: apply single JSON first, then JSONL entries.
4. Delay `pre_cmd_delay_ms` after each pre-command send.

## Required Behavior Contract

1. Pre-commands execute after serial open and after optional init, but before measurement loop begins.
2. Invalid JSON from either source fails fast with clear error and no measurement-loop start.
3. Each pre-command payload must include key `T` with integer value.
4. TX log is append-only JSONL and does not alter summary math.
5. Existing command lines with no new flags continue to function unchanged.

## Minimal Internal Design (Implementation Guidance)

Implement as a small extension of existing structure rather than a redesign.

### 1. Parsing helpers

Add helper(s) to:

1. Parse one JSON string into `dict[str, Any]`.
2. Parse a JSONL file into `list[dict[str, Any]]`.
3. Validate each parsed payload has integer `T`.
4. Raise clear `ValueError`/user-facing error for malformed inputs.

### 2. Unified write wrapper

Add helper similar to:

1. `_write_command(ser, payload, phase, tx_log_file)`

Responsibilities:

1. Serialize compact JSON + newline.
2. Send via serial.
3. When TX log enabled, append `{ts_monotonic, phase, payload}`.

### 3. Replace direct TX writes

Use wrapper in:

1. `_maybe_init(...)` (`phase="init"`)
2. `_send_and_await_pan(...)` (`phase="query"`)
3. New pre-command dispatch path (`phase="pre_cmd"`)

### 4. Run flow extension

Desired high-level order:

1. Open serial.
2. Optional init.
3. Apply pre-commands.
4. Start timed measurement loop.

### 5. TX log lifecycle

1. Open `--tx-log-jsonl` once in `run(...)`.
2. Pass handle to lower-level helpers that emit TX.
3. Close at run end via context manager.

## TX Log Schema (New Output)

Each line in `--tx-log-jsonl`:

1. `ts_monotonic`: float
2. `phase`: string (`init`, `pre_cmd`, `query`)
3. `payload`: object

Notes:

1. Keep schema stable and minimal.
2. No requirement to include RX-derived fields in TX log.

## Error Handling Expectations

1. Input parse/validation errors must stop before measurement loop.
2. Error text should identify whether failure came from `--pre-cmd-json` or `--pre-cmd-jsonl` (and ideally line number for JSONL).
3. Serial exceptions should follow existing error-handling style where practical.
4. Empty lines in `--pre-cmd-jsonl` should be ignored, not treated as commands.

## Planning Sequence Gates

Use these gates to structure the implementation plan:

1. Gate 1 - CLI + parsing design complete:
    flag definitions, parsing behavior, and validation error semantics are specified.
2. Gate 2 - TX write-path refactor complete:
    unified write helper is wired into init and query paths with no default-path regression.
3. Gate 3 - Pre-command execution wired:
    run order is open -> optional init -> pre-commands -> measurement loop.
4. Gate 4 - TX log lifecycle complete:
    one run-scoped file handle, append-per-event schema, and phase tagging validated.
5. Gate 5 - Tests + verification complete:
    required test set passes, plus explicit default-path safety checks.

## Test Plan Requirements

Add or extend tests in [ugv-follower/tests/test_check_pan_telemetry_only.py](../../ugv-follower/tests/test_check_pan_telemetry_only.py).

Minimum required test coverage:

1. Single pre-command is sent before first query.
2. JSONL pre-commands are all sent in file order.
3. Invalid pre-command JSON aborts before measurement loop.
4. TX log captures `init`, `pre_cmd`, and `query` phases with payloads.
5. Default path remains unchanged when new flags are omitted.

Recommended implementation test style:

1. Continue using mocks/fakes for serial writes and time behavior.
2. Assert call order explicitly where ordering is critical.
3. Use temporary files for JSONL input/output assertions.

## Acceptance Criteria

1. Existing command lines still run without modification.
2. Step B gate/steady experiments can be configured with no manual pre-run serial scripting.
3. Reapply investigations can capture deterministic outbound command evidence via `--tx-log-jsonl`.
4. New and existing relevant unit tests pass.

## Non-Regression Checklist

1. Existing CLI flags keep prior meanings and defaults.
2. Existing per-query JSONL output (`--log-jsonl`) schema remains unchanged.
3. Existing summary math and timeout/success accounting remain unchanged.
4. Existing no-new-flags invocation does not require new inputs or files.

## Suggested Verification Commands

From `ugv-follower/`:

1. `pytest tests/test_check_pan_telemetry_only.py -v`

Optional quick manual smoke (hardware environment only):

1. `ugv-check-pan-telemetry-only --duration 5 --poll-interval 0.05 --timeout 0.3 --tx-log-jsonl tx_smoke.jsonl`

## User-Facing Examples After Refactor

1. Gate test setup run:

    `ugv-check-pan-telemetry-only --pre-cmd-json '{"T":<feedback_interval_cmd>,"cmd":0}' --duration 180 --poll-interval 0.05 --timeout 0.3 --log-jsonl gate0.jsonl --tx-log-jsonl gate0_tx.jsonl`

2. Multiple setup commands from file:

    `ugv-check-pan-telemetry-only --pre-cmd-jsonl stepb_setup.jsonl --pre-cmd-delay-ms 150 --duration 180 --log-jsonl run.jsonl --tx-log-jsonl run_tx.jsonl`

3. Runtime reapply long run:

    `ugv-check-pan-telemetry-only --duration 900 --poll-interval 0.05 --timeout 0.3 --log-jsonl reapply_probe.jsonl --tx-log-jsonl reapply_probe_tx.jsonl`

## Planning Agent Deliverable Expectations

The downstream planning agent should produce an implementation plan that includes:

1. Exact function-level change list in `check_pan_telemetry_only.py`.
2. New/updated test case list in `test_check_pan_telemetry_only.py`.
3. Risk notes for default-path regression.
4. Verification steps and pass/fail gates.

## Important Note On Firmware Payloads

Concrete command payloads for feedback interval and gimbal steady mode come from firmware command mapping outside this repository. This refactor must remain payload-agnostic and only enforce generic command-shape validation.
