# Xavier

Wheelchair navigator over a Matterport twin (Princeton Forum demo).

## Local

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
export MATTERPORT_SDK_KEY=…
.venv/bin/python main.py
```

Open http://localhost:8080/ → `/nav.html`.

## Vercel

1. Import this repo in Vercel (Framework: FastAPI / Python).
2. Set env `MATTERPORT_SDK_KEY`.
3. Deploy. Entry point is `main.py` (`app = FastAPI()`).

Static assets: `nav.html`, `viewer/`, `catalog/nav/`, `manual-wheelchair.glb`.
API: `/api/nav/…` (global + local planner).
