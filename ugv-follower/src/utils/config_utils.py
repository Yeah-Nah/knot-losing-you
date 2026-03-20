"""Config loading utilities for ugv-follower."""

from pathlib import Path

import yaml
from loguru import logger


def get_project_root() -> Path:
    """Return the absolute path to the ugv-follower project root.

    This file lives at ``src/utils/config_utils.py``, so the project root
    (the directory containing ``pyproject.toml``) is three levels up.
    """
    if __file__ is None:
        raise RuntimeError("__file__ is not available in this context")
    return Path(__file__).resolve().parent.parent.parent


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
