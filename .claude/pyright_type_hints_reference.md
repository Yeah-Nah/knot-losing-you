# Pyright Type Hints Reference

Purpose: fix pyright `standard` mode errors in this codebase.
Load this file when resolving pre-commit or CI type-checking failures.

Checker config: `ugv-follower/pyrightconfig.json` — Python 3.11, standard mode.

---

## 1. File Header (required in every file)

```python
from __future__ import annotations
```

This enables postponed annotation evaluation, allowing forward references
everywhere without quoting.

---

## 2. Built-in Generic Types

Use lower-case built-ins. Never import `List`, `Dict`, `Tuple`, `Set`,
`FrozenSet`, or `Optional` from `typing`.

```python
# correct
def fn(items: list[str], mapping: dict[str, int]) -> tuple[float, float]: ...
def maybe(x: str | None) -> str | None: ...
def either(x: int | str) -> int | str: ...

# wrong — do not write
from typing import Dict, List, Optional, Tuple
def fn(items: List[str]) -> Optional[str]: ...
```

---

## 3. TYPE_CHECKING Guard

Imports used only in annotations (to avoid circular imports or heavy runtime
cost) go inside the guard.

```python
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray
    from ugv_follower.perception.lidar_access import LidarPoint
    from ugv_follower.settings import Settings
```

At runtime the block is skipped; pyright still type-checks it fully.

---

## 4. NumPy Arrays

Always parameterise `NDArray` with the dtype. Import via `TYPE_CHECKING`
unless the array is constructed in the same file (in which case `numpy` is
already a runtime import).

```python
if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

def process_frame(frame: NDArray[np.uint8]) -> NDArray[np.uint8]: ...
def compute_transform(pts: NDArray[np.float64]) -> NDArray[np.float64]: ...
```

Common dtypes: `np.uint8`, `np.float32`, `np.float64`, `np.int32`.

---

## 5. TypedDict

Use for structured dicts that cross function or module boundaries (e.g. LiDAR
scan points, calibration rows, config blocks). Declare every field explicitly.
Use `total=False` only for genuinely optional keys.

```python
from typing import TypedDict

class BodyFramePoint(TypedDict):
    x_m: float
    y_m: float
    distance_m: float
    bearing_deg: float

# partial optional fields
class CalibrationRow(TypedDict, total=False):
    commanded_deg: float
    observed_deg: float
    note: str          # optional
```

---

## 6. Callable

Annotate the full signature. Import from `collections.abc`, not `typing`.

```python
from collections.abc import Callable

# full signature
def apply(fn: Callable[[float, float], bool], a: float, b: float) -> bool: ...

# no arguments, no return
def run(callback: Callable[[], None]) -> None: ...

# common pattern — threading send function
def start_shaper(send_fn: Callable[[int, int], None]) -> None: ...

# generic higher-order helper
from typing import TypeVar
T = TypeVar("T")
def retry(fn: Callable[[], T], attempts: int) -> T: ...
```

---

## 7. Threading

Declare thread-related attributes with explicit types. `threading.Event` and
`threading.Lock` are never `None`; initialise them in `__init__`.

```python
import threading

class Worker:
    _thread: threading.Thread | None      # None before start / after join
    _stop_event: threading.Event          # always initialised
    _lock: threading.Lock                 # always initialised

    def __init__(self) -> None:
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
```

---

## 8. `Any` and YAML / JSON Dicts

Use `dict[str, Any]` for unvalidated external data. Narrow the type as soon as
the structure is confirmed (via assertion, TypedDict cast, or isinstance check).

```python
from typing import Any
import yaml

def load_raw(path: Path) -> dict[str, Any]:
    with path.open() as f:
        return yaml.safe_load(f)  # type: ignore[return-value]  # safe_load -> Any

# narrowing example
raw = load_raw(config_path)
if "ugv_drive" not in raw:
    raise KeyError("Missing 'ugv_drive' section in config")
drive_cfg: dict[str, Any] = raw["ugv_drive"]
```

---

## 9. argparse

Annotate `main()` as `-> None`. Pyright treats `Namespace` attribute access as
`Any` in standard mode — this is acceptable and requires no workaround.

```python
import argparse

def main() -> None:
    parser = argparse.ArgumentParser(description="...")
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--duration", type=float, default=8.0)
    args = parser.parse_args()
    run(port=args.port, duration=args.duration)  # Any is accepted here
```

---

## 10. Return Type Discipline

Every function and method needs an explicit return annotation.

```python
# always annotate, including None
def stop(self) -> None: ...
def is_ready(self) -> bool: ...
def get_port(self) -> str: ...

# generator
from collections.abc import Generator
def scan_packets() -> Generator[LidarPoint, None, None]: ...

# context manager — use Self for the managed object
from typing import Self

class Pipeline:
    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
```

---

## 11. `cast` — Use Sparingly

Use only when pyright cannot infer a type you can prove is correct. Always add
a comment.

```python
from typing import Any, cast

# yaml.safe_load returns Any; shape validated above
config = cast(dict[str, Any], yaml.safe_load(f))

# cv2 returns Any for some operations; we know the shape
matrix = cast("NDArray[np.float64]", cv2.getOptimalNewCameraMatrix(...))
```

---

## 12. Common Pyright Error Patterns and Fixes

| Error | Likely cause | Fix |
|---|---|---|
| `Cannot access attribute "x" for class "None"` | Attribute may be `None`; not guarded | Add `if self._x is not None:` before access |
| `Type "X \| None" is not assignable to "X"` | Optional not narrowed | Guard with `assert x is not None` or `if x is None: raise` |
| `Return type "None" is not assignable to return type "X"` | Missing `return` branch | Add explicit `return` or raise in all branches |
| `Cannot access member "y" for type "dict[str, Any]"` | Dict subscript not typed | Use `cast` or TypedDict |
| `Argument of type "X" cannot be assigned to parameter of type "Y"` | Wrong type passed | Check annotation on the called function; fix the caller or the callee |
| `"x" is possibly unbound` | Variable assigned only in one branch | Initialise before the branch, e.g. `x: int \| None = None` |
| `Function with declared return type "X" must return value` | Missing return in some path | Add `return` or `raise` to every code path |
| `Operator "\|" not supported for types "type[X]" and "type[Y]"` | Using `X \| Y` without `from __future__ import annotations` | Add the future import |
