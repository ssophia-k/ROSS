"""`ross slam` — room scanner (MiDaS depth + ICP + Poisson + YOLO people)."""

from pathlib import Path

import typer

from ross.config import MODELS_DIR, OUTPUTS_DIR


def slam(
    host: str = typer.Option(
        None, "--host", help="robot IP or hostname (default: auto-discover)"
    ),
    model: str = typer.Option(
        "DPT_Hybrid",
        "--model",
        help="MiDaS model — DPT_Large | DPT_Hybrid | MiDaS_small",
    ),
    no_imu: bool = typer.Option(
        False, "--no-imu", help="skip IMU; use pure yaw sweep for pose"
    ),
    no_yolo: bool = typer.Option(False, "--no-yolo", help="skip human detection"),
    output: Path = typer.Option(
        OUTPUTS_DIR / "scan",
        "--output",
        "-o",
        help="output file prefix (writes <prefix>.ply and <prefix>_cloud.ply)",
    ),
) -> None:
    """Capture MJPEG stream, run SLAM, save PLY mesh + point cloud, open viewer."""
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    yolo_weights = str(MODELS_DIR / "yolov8n.pt")

    from ross.slam.pipeline import run

    run(
        output=output,
        host=host,
        model=model,
        use_imu=not no_imu,
        use_yolo=not no_yolo,
        yolo_weights=yolo_weights,
    )
