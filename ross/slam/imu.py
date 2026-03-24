"""
IMU pre-integration for Sensor Logger exports.

Sensor Logger CSV format (Gyroscope.csv / Accelerometer.csv):
    time, seconds_elapsed, z, y, x
where
    time            – ISO-8601 timestamp string  (I don't use it here)
    seconds_elapsed – float, seconds since recording start
    z, y, x         – sensor readings in device frame (rad/s or m/s²)

The columns are reordered to (x, y, z) internally for consistency with
the camera frame convention used in the rest of the SLAM system.

Usage
-----
    imu = IMUData("Gyroscope.csv", "Accelerometer.csv")
    delta = imu.preintegrate(t_start, t_end)
    # delta.dR  – 3×3 rotation increment  (np.ndarray)
    # delta.dv  – 3-vec velocity increment (np.ndarray)
    # delta.dp  – 3-vec position increment (np.ndarray)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd
from loguru import logger


# Rodrigues' formula for rotation integration

def _skew(v: np.ndarray) -> np.ndarray:
    """Return the 3×3 skew-symmetric matrix for cross-product with *v*."""
    x, y, z = v
    return np.array([
        [0,  -z,  y],
        [z,   0, -x],
        [-y,  x,  0],
    ], dtype=float)


def _rodrigues(omega: np.ndarray, dt: float) -> np.ndarray:
    """Integrate an angular-velocity vector *omega* (rad/s) over *dt* seconds.

    Returns the 3×3 rotation matrix corresponding to rotating by ‖omega‖·dt
    radians around the *omega* axis (Rodrigues' formula).

    Parameters
    ----------
    omega : np.ndarray, shape (3,)
        Angular velocity in rad/s.
    dt : float
        Time step in seconds.

    Returns
    -------
    np.ndarray, shape (3, 3)
        Incremental rotation matrix.
    """
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-9:
        return np.eye(3)
    axis = omega / np.linalg.norm(omega)
    K = _skew(axis)
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


# pre-integration delta R, v, p container

@dataclass
class PreintegrationDelta:
    """Accumulated IMU measurement between two camera frames.

    Attributes
    ----------
    dR : np.ndarray
        3×3 rotation increment (body frame).
    dv : np.ndarray
        3-vec velocity increment (m/s, body frame, gravity NOT removed).
    dp : np.ndarray
        3-vec position increment (m, body frame).
    dt : float
        Total integration time in seconds.
    n_samples : int
        Number of IMU samples integrated.
    """
    dR: np.ndarray = field(default_factory=lambda: np.eye(3))
    dv: np.ndarray = field(default_factory=lambda: np.zeros(3))
    dp: np.ndarray = field(default_factory=lambda: np.zeros(3))
    dt: float = 0.0
    n_samples: int = 0

    def as_pose4x4(self) -> np.ndarray:
        """Return dR and dp packed into a 4×4 homogeneous matrix."""
        T = np.eye(4)
        T[:3, :3] = self.dR
        T[:3,  3] = self.dp
        return T


# IMU data loading and pre-integration

class IMUData:
    """Loads and serves Sensor Logger gyroscope + accelerometer data.

    Parameters
    ----------
    gyro_path : str | Path
        Path to ``Gyroscope.csv``.
    accel_path : str | Path
        Path to ``Accelerometer.csv``.
    gyro_bias : np.ndarray | None
        Optional 3-vec gyroscope bias to subtract (rad/s).
    accel_bias : np.ndarray | None
        Optional 3-vec accelerometer bias to subtract (m/s²).
    """

    # Sensor Logger uses (z, y, x) column order → reindex to (x, y, z)
    _AXIS_COLS = ["x", "y", "z"]

    def __init__(
        self,
        gyro_path: str | Path,
        accel_path: str | Path,
        gyro_bias: np.ndarray | None = None,
        accel_bias: np.ndarray | None = None,
    ):
        self.gyro_bias = gyro_bias if gyro_bias is not None else np.zeros(3)
        self.accel_bias = accel_bias if accel_bias is not None else np.zeros(3)

        self._gyro = self._load(gyro_path)
        self._accel = self._load(accel_path)

        logger.info(
            f"IMU loaded – gyro: {len(self._gyro)} samples "
            f"[{self._gyro['t'].iloc[0]:.3f} … {self._gyro['t'].iloc[-1]:.3f} s], "
            f"accel: {len(self._accel)} samples"
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _load(path: str | Path) -> pd.DataFrame:
        """Read a Sensor Logger CSV and return a tidy DataFrame with columns
        ``t`` (seconds from start), ``x``, ``y``, ``z``."""
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"IMU file not found: {path}")

        df = pd.read_csv(path)
        df.columns = [c.strip().lower() for c in df.columns]

        # Sensor Logger exports: time, seconds_elapsed, z, y, x
        # Rename seconds_elapsed → t and reorder axes
        if "seconds_elapsed" not in df.columns:
            raise ValueError(
                f"Expected 'seconds_elapsed' column in {path.name}. "
                f"Got: {list(df.columns)}"
            )
        


        df = df.rename(columns={"seconds_elapsed": "t"})
        df = df[["t", "x", "y", "z"]].copy()
        df = df.sort_values("t").reset_index(drop=True)
        return df

    def _slice(self, df: pd.DataFrame, t0: float, t1: float) -> pd.DataFrame:
        """Return rows where t0 ≤ t ≤ t1, with guard rows on each side."""
        mask = (df["t"] >= t0) & (df["t"] <= t1)
        subset = df[mask].copy()

        # Prepend the last sample before t0 (for the first half-step)
        before = df[df["t"] < t0]
        if not before.empty:
            subset = pd.concat([before.iloc[[-1]], subset], ignore_index=True)

        # Append the first sample after t1 (for the last half-step)
        after = df[df["t"] > t1]
        if not after.empty:
            subset = pd.concat([subset, after.iloc[[0]]], ignore_index=True)

        return subset.reset_index(drop=True)



    def preintegrate(self, t0: float, t1: float) -> PreintegrationDelta:
        """Integrate IMU measurements between two camera frame timestamps.

        Uses the midpoint (trapezoidal) rule for both rotation and translation.

        Parameters
        ----------
        t0, t1 : float
            Start and end times in seconds (same reference as ``seconds_elapsed``
            in the Sensor Logger CSVs).

        Returns
        -------
        PreintegrationDelta
            Accumulated ΔR, Δv, Δp over [t0, t1].
        """
        gyro_slice  = self._slice(self._gyro,  t0, t1)
        accel_slice = self._slice(self._accel, t0, t1)

        if len(gyro_slice) < 2:
            logger.warning(f"Only {len(gyro_slice)} gyro samples in [{t0:.3f}, {t1:.3f}] – skipping")
            return PreintegrationDelta(dt=t1 - t0)

        # Merge on nearest timestamp so each step has both gyro + accel
        merged = pd.merge_asof(
            gyro_slice.rename(columns={"x": "gx", "y": "gy", "z": "gz"}),
            accel_slice.rename(columns={"x": "ax", "y": "ay", "z": "az"}),
            on="t",
            direction="nearest",
        )

        dR = np.eye(3)
        dv = np.zeros(3)
        dp = np.zeros(3)
        n  = 0

        for i in range(len(merged) - 1):
            r0 = merged.iloc[i]
            r1 = merged.iloc[i + 1]
            dt = float(r1["t"] - r0["t"])
            if dt <= 0:
                continue

            # Midpoint angular velocity (bias corrected)
            omega = 0.5 * (
                np.array([r0["gx"], r0["gy"], r0["gz"]])
                + np.array([r1["gx"], r1["gy"], r1["gz"]])
            ) - self.gyro_bias

            # Midpoint acceleration (bias corrected)
            accel = 0.5 * (
                np.array([r0["ax"], r0["ay"], r0["az"]])
                + np.array([r1["ax"], r1["ay"], r1["az"]])
            ) - self.accel_bias

            dR_step = _rodrigues(omega, dt)

            # Position and velocity integration (body frame, gravity NOT removed)
            # For loosely coupled VIO, dp is mainly used as a sanity check.
            dp += dv * dt + 0.5 * (dR @ accel) * dt ** 2
            dv += (dR @ accel) * dt
            dR = dR @ dR_step
            n  += 1

        return PreintegrationDelta(dR=dR, dv=dv, dp=dp, dt=t1 - t0, n_samples=n)

    @property
    def t_start(self) -> float:
        return float(self._gyro["t"].iloc[0])

    @property
    def t_end(self) -> float:
        return float(self._gyro["t"].iloc[-1])