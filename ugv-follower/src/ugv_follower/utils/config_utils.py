"""Config loading utilities for ugv-follower."""

from pathlib import Path

import yaml
from loguru import logger


def get_project_root() -> Path:
    """Return the absolute path to the ugv-follower project root.

    This file lives at ``src/ugv_follower/utils/config_utils.py``. Walks
    upward from this file until it finds a directory containing
    ``pyproject.toml``.
    """
    if __file__ is None:
        raise RuntimeError("__file__ is not available in this context")
    candidate = Path(__file__).resolve().parent
    for _ in range(10):
        if (candidate / "pyproject.toml").exists():
            return candidate
        candidate = candidate.parent
    raise RuntimeError(
        f"Could not locate pyproject.toml above {Path(__file__).resolve()}"
    )


def load_yaml(path: str | Path) -> dict[str, object]:
    """Load a YAML file and return its contents as a dictionary.

    Parameters
    ----------
    path : str | Path
        Path to the YAML file to load.

    Returns
    -------
    dict[str, object]
        Parsed YAML contents.

    Raises
    ------
    FileNotFoundError
        If the file does not exist at the given path.
    ValueError
        If the file is empty or contains only null.
    """
    resolved = Path(path).resolve()
    if not resolved.exists():
        logger.error(f"Config file not found: {resolved}")
        raise FileNotFoundError(f"Config file not found: {resolved}")
    with resolved.open("r") as f:
        data = yaml.safe_load(f)
    if data is None:
        logger.error(f"Config file is empty: {resolved}")
        raise ValueError(f"Config file is empty: {resolved}")
    logger.debug(f"Loaded config from {resolved}")
    return data  # type: ignore[no-any-return]
