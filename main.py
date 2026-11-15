"""Xavier: wheelchair navigator API + static files (Vercel / local).

    .venv/bin/python main.py
"""
from __future__ import annotations

import os
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, RedirectResponse

from app import nav_api
from app.config import ROOT, secret

PORT = int(os.environ.get("PORT", "8080"))
PUBLIC_DIRS = {"viewer", "catalog", "assets"}
PUBLIC_SUFFIXES = {".html", ".js", ".mjs", ".css", ".json", ".glb", ".jpg", ".png", ".svg", ".ico", ".woff2", ".woff", ".wasm", ".gz", ".webp", ".txt", ".cjs", ".gif"}

app = FastAPI(title="Xavier")
app.include_router(nav_api.router)


@app.get("/sdk-config")
def sdk_config():
    return {"key": secret("MATTERPORT_SDK_KEY")}


def public_file(path: str):
    if not path or path == "/":
        return ROOT / "nav.html"
    parts = Path(path).parts
    if any(part.startswith(".") for part in parts):
        return None
    target = (ROOT / path).resolve()
    if not target.is_file() or not target.is_relative_to(ROOT):
        return None
    top = target.relative_to(ROOT).parts
    if len(top) == 1 and target.suffix in PUBLIC_SUFFIXES:
        return target
    if len(top) > 1 and top[0] in PUBLIC_DIRS and target.suffix in PUBLIC_SUFFIXES:
        return target
    return None


@app.get("/")
def home():
    return RedirectResponse("/nav.html", status_code=307)


@app.get("/{path:path}")
def static(path: str):
    target = public_file(path)
    if target is None:
        raise HTTPException(404)
    return FileResponse(target)


if __name__ == "__main__":
    import uvicorn

    print(f"Xavier at http://localhost:{PORT}/", flush=True)
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
