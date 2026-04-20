from pathlib import Path

from dotenv import load_dotenv

PROJ_ROOT = Path(__file__).resolve().parents[1]
OUTPUTS_DIR = PROJ_ROOT / "outputs"
MODELS_DIR = PROJ_ROOT / "models"

load_dotenv(PROJ_ROOT / ".env")
