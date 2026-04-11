"""Camera-device preflight checks for V4L2 workflows.

Provides a single place to detect whether a camera device path is already held
open by another process and, when enabled, attempt to release it before a
script starts ``cv2.VideoCapture``.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import time

from loguru import logger

_DEFAULT_RELEASE_COMMAND: tuple[str, ...] = ("fuser", "-k")


def _run_command(command: list[str]) -> subprocess.CompletedProcess[str]:
    """Execute a subprocess command and capture text output.

    Parameters
    ----------
    command : list[str]
        Command and arguments to run.

    Returns
    -------
    subprocess.CompletedProcess[str]
        Completed process object including stdout/stderr.
    """
    return subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
    )


def _parse_pid_tokens(text: str) -> list[int]:
    """Extract process IDs from command output text.

    Parameters
    ----------
    text : str
        Raw output from ``lsof``/``fuser``.

    Returns
    -------
    list[int]
        Sorted unique PID values.
    """
    pids = {int(token) for token in re.findall(r"\b\d+\b", text)}
    return sorted(pids)


def _describe_pid(pid: int) -> str:
    """Return a short process description for a PID.

    Parameters
    ----------
    pid : int
        Process identifier.

    Returns
    -------
    str
        ``"<pid>(<name>)"`` when available, otherwise ``"<pid>"``.
    """
    if shutil.which("ps") is None:
        return str(pid)

    result = _run_command(["ps", "-p", str(pid), "-o", "comm="])
    name = result.stdout.strip()
    return f"{pid}({name})" if name else str(pid)


def _find_device_holders(device_path: str) -> list[int]:
    """Return PIDs currently holding *device_path* open.

    Parameters
    ----------
    device_path : str
        Device file path (for example, ``/dev/video0``).

    Returns
    -------
    list[int]
        Sorted PID list. Empty when no holders are found or lookup tools are
        unavailable.
    """
    if shutil.which("lsof") is not None:
        lsof_result = _run_command(["lsof", "-t", device_path])
        lsof_pids = _parse_pid_tokens(lsof_result.stdout)
        if lsof_pids:
            return lsof_pids

    if shutil.which("fuser") is not None:
        # fuser often prints PID info on stderr; parse both streams.
        fuser_result = _run_command(["fuser", device_path])
        fuser_pids = _parse_pid_tokens(
            fuser_result.stdout + "\n" + fuser_result.stderr
        )
        return fuser_pids

    return []


def _release_device(device_path: str, release_command: list[str]) -> None:
    """Run a release command to terminate holders of *device_path*.

    Parameters
    ----------
    device_path : str
        Device file path to release.
    release_command : list[str]
        Command prefix used to release the device. ``device_path`` is appended
        as the final argument.

    Raises
    ------
    RuntimeError
        If the release command executable cannot be found or exits non-zero.
    """
    if not release_command:
        raise RuntimeError("release_command must contain at least one element")

    executable = release_command[0]
    if shutil.which(executable) is None:
        raise RuntimeError(f"Camera release command not found: {executable}")

    command = [*release_command, device_path]
    result = _run_command(command)
    if result.returncode != 0:
        stderr = result.stderr.strip() or "<no stderr>"
        raise RuntimeError(
            f"Failed to release {device_path} with {' '.join(command)}: {stderr}"
        )


def ensure_camera_device_available(
    device_path: str,
    *,
    auto_release: bool = True,
    release_command: list[str] | None = None,
    settle_time_s: float = 0.25,
) -> None:
    """Ensure a camera device can be opened by this process.

    Parameters
    ----------
    device_path : str
        Device file path (for example, ``/dev/video0``).
    auto_release : bool, default=True
        Whether to attempt terminating holder processes automatically.
    release_command : list[str] | None, default=None
        Optional command prefix used to release the device. When ``None``,
        defaults to ``["fuser", "-k"]``.
    settle_time_s : float, default=0.25
        Delay after release before re-checking holders.

    Raises
    ------
    RuntimeError
        If the device is busy and cannot be released, or if auto-release is
        disabled while holders are present.
    """
    if os.name != "posix":
        logger.debug("Camera preflight skipped on non-posix OS.")
        return

    holders = _find_device_holders(device_path)
    if not holders:
        logger.debug(f"Camera preflight: {device_path} is available.")
        return

    holder_desc = ", ".join(_describe_pid(pid) for pid in holders)
    logger.warning(f"Camera device {device_path} is busy (holders: {holder_desc}).")

    if not auto_release:
        raise RuntimeError(
            f"Camera device {device_path} is busy (holders: {holder_desc})."
        )

    command = (
        list(release_command)
        if release_command is not None
        else list(_DEFAULT_RELEASE_COMMAND)
    )
    logger.info(f"Attempting to release {device_path} via: {' '.join(command)}")
    _release_device(device_path, command)

    time.sleep(max(0.0, settle_time_s))
    remaining = _find_device_holders(device_path)
    if remaining:
        remaining_desc = ", ".join(_describe_pid(pid) for pid in remaining)
        raise RuntimeError(
            f"Camera device {device_path} is still busy after release attempt "
            f"(holders: {remaining_desc})."
        )

    logger.info(f"Camera preflight: released busy holder(s) for {device_path}.")
