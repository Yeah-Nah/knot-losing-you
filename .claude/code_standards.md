# Code Standards & Best Practices

## PEP 8 Compliance

- **Line length**: Maximum 88 characters (Black formatter standard)
- **Imports**: Organize in order: standard library, third-party, local imports
- **Naming conventions**:
  - `snake_case` for functions, methods, variables
  - `PascalCase` for classes
  - `UPPER_CASE` for constants
  - `_leading_underscore` for private/internal methods
- **Docstrings**: Use NumPy-style docstrings for all functions and classes
- **Type hints**: Required for all function signatures and class attributes

---

## Clean Code Principles

### Functions & Methods
- **Single Responsibility**: Each function should do one thing well
- **Keep methods short**: Target < 30 lines; extract complex logic into helper methods
- **Descriptive names**: Function names should describe what they do, not how
- **Use helper methods**: Extract complex initialization or conditional logic from `__init__`

### `__init__` Method Structure
✅ Good:
```python
def __init__(self, settings: Settings) -> None:
    self._settings = settings
    self._camera = self._create_camera()
    self._controller = self._create_controller()
    self._detector = self._create_detector()
```

❌ Avoid:
```python
def __init__(self, settings: Settings) -> None:
    # 50 lines of complex conditional logic
```

### Error Handling
- **Specific exceptions first**: Catch specific exceptions before generic ones
- **Always use `from exc`**: Chain exceptions with `raise NewError(...) from exc`
- **Clean up on failure**: Set state to `None` or cleanup resources in exception handlers
- **Meaningful error messages**: Include context about what failed and why

### Code Organization
- **Properties for read-only access**: Use `@property` for computed values or clean API
- **Constants at module level**: Extract magic numbers as named constants
- **Context managers**: Implement `__enter__` and `__exit__` for resource management
- **Type checking imports**: Use `if TYPE_CHECKING:` block to avoid circular imports

---

## Python Best Practices

### Resource Management
```python
# Always use context managers for resources
with Pipeline(settings) as pipeline:
    pipeline.run()
```

### Type Hints

- All files: `from __future__ import annotations` at the top.
- Checker: **pyright** in `standard` mode — write annotations pyright accepts.
- Use lower-case built-ins: `list[x]`, `dict[k, v]`, `tuple[...]`, `x | None` (never `List`, `Dict`, `Optional` from `typing`).
- Expensive/circular imports for annotations only → inside `if TYPE_CHECKING:` guard.
- `NDArray` must be parameterised: `NDArray[np.uint8]`.
- Use `TypedDict` for structured dicts that cross function boundaries.
- Every function/method needs a return type, including `-> None`.
- Full pattern reference (for fixing pyright errors): `.claude/pyright_type_hints_reference.md`.

### Logging
- Use `logger.debug()` for development/diagnostic info
- Use `logger.info()` for important state changes
- Use `logger.warning()` for recoverable issues
- Use `logger.error()` for errors that need attention

---

## Project-Specific Conventions

### Configuration
- All settings should be config-driven, not hardcoded
- Validate settings at initialization, fail fast
- Use properties in `Settings` class for type-safe access

### File Organization
- Keep related functionality in the same module
- Separate concerns: perception, inference, control, utils
- Tests should mirror source structure

### Docstrings
Use NumPy-style format:
```python
def method_name(self, param1: int, param2: str) -> bool:
    """Short one-line summary.

    Longer description if needed, explaining behaviour,
    edge cases, or important implementation details.

    Parameters
    ----------
    param1 : int
        Description of param1.
    param2 : str
        Description of param2.

    Returns
    -------
    bool
        Description of return value.

    Raises
    ------
    RuntimeError
        When something goes wrong.

    Examples
    --------
    >>> method_name(42, "test")
    True
    """
```

Use UK english spelling:
- E.g. "behaviour" instead of "behaviour"

---

## Code Review Checklist

Before committing, verify:
- [ ] All functions have type hints and docstrings
- [ ] Complex logic is extracted into helper methods
- [ ] Magic numbers are replaced with named constants
- [ ] Error handling is specific and includes context
- [ ] Resources are properly managed (closed/cleaned up)
- [ ] No hardcoded paths or device-specific values
- [ ] Pre-commit hooks pass (ruff, pyright)
- [ ] Code is testable (methods can be unit tested independently)
