# Claude Instructions

Read `.claude/code_standards.md` before making code changes and follow it as the default engineering standard for this repository.

## Mandatory Rules

1. Keep changes minimal and focused on the task.
2. Follow PEP 8, Black-style 88 character lines, and existing project style.
3. Add type hints to all new or changed function signatures.
4. Use NumPy-style docstrings for all new or changed public functions and classes.
5. Keep functions small and single-purpose. Extract helper functions when logic becomes hard to follow.
6. Prefer config-driven behavior over hardcoded values.
7. Use descriptive names. Avoid one-letter variables except in trivial math or loop cases.
8. Handle errors explicitly with clear messages and safe cleanup.
9. Do not change calibration math, runtime behavior, or file formats unless the task requires it.
10. Do not refactor unrelated code while implementing a feature or fix.

## Validation Rules

1. Run relevant tests for the files you changed.
2. Run linting and formatting checks when applicable.
3. If a validation step cannot be run, say so explicitly and explain why.
4. Do not claim completion if code has obvious lint, type, or test failures related to your changes.

## Repository-Specific Expectations

1. Keep hardware, calibration, perception, control, and utility concerns clearly separated.
2. Preserve existing config structure unless a change is necessary.
3. Keep calibration outputs backward-compatible unless explicitly asked to change the schema.
4. Prefer small helper methods over large `__init__` methods or deeply nested logic.
5. Add or update tests when behavior changes.

## When Standards and Task Conflict

Follow the user’s request, but call out any tradeoffs clearly. If a requested change conflicts with `.claude/code_standards.md`, minimize the deviation and explain it in the final summary.
