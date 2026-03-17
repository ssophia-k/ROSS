from pathlib import Path

from dotenv import load_dotenv

PROJ_ROOT = Path(__file__).resolve().parents[1]
load_dotenv(PROJ_ROOT / ".env")
