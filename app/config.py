"""Paths and credentials. Keys live in ~/costar/.env, outside the project."""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
KIT = Path.home() / "costar"
CATALOG = ROOT / "catalog/properties.json"


def secret(name):
    value = os.environ.get(name)
    if value:
        return value
    try:
        lines = (KIT / ".env").read_text().splitlines()
    except OSError:
        return None
    for line in lines:
        key, _, raw = line.partition("=")
        if key.strip() == name:
            return raw.strip().strip('"').strip("'") or None
    return None


# Free-tier quotas are per model and per day, so the agent falls through this list on 429s.
GEMINI_MODELS = [m.strip() for m in os.environ.get(
    "GEMINI_MODELS",
    "gemini-3.5-flash,gemini-3-flash-preview,gemini-3.5-flash-lite,gemini-3.1-flash-lite,gemini-flash-lite-latest",
).split(",") if m.strip()]
GEMINI_MODEL = os.environ.get("GEMINI_MODEL", GEMINI_MODELS[0])
