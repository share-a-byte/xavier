"""Local obstacle-avoidance planner for the wheelchair.

Independent of Matterport rendering and of any sensor driver: callers pass a pose, a
goal (usually the next global waypoint), chair width, and a list of obstacles in map
metres (x, z). Returns a short collision-free path that rejoins the goal, or explains
why none exists so the global planner can replan or the chair can stop.

Obstacles are circles or axis-aligned boxes on the floor plane. The chair is treated as
a disc of radius half its width plus a small turning clearance; that disc must stay
outside every inflated obstacle.
"""
from __future__ import annotations

import heapq
import math

CELL = 0.12  # metres; fine enough for hallway detours, cheap enough for a phone
CLEARANCE_M = 0.08  # extra buffer beyond the chair's half-width for turning
REACH_PAD_M = 1.5  # expand the local window past the start–goal segment
MAX_CELLS = 12_000
STEPS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))


def chair_radius_m(chair_width_cm, clearance_m=CLEARANCE_M):
    width = float(chair_width_cm)
    if not math.isfinite(width) or not 20 <= width <= 200:
        raise ValueError("chair_width_cm must be between 20 and 200")
    return width / 200.0 + clearance_m


def _as_obstacle(raw):
    """Normalise one obstacle to either a circle {x,z,r} or a box {xmin,xmax,zmin,zmax}."""
    if not isinstance(raw, dict):
        raise ValueError("Each obstacle is an object")
    if all(k in raw for k in ("xmin", "xmax", "zmin", "zmax")):
        xmin, xmax = float(raw["xmin"]), float(raw["xmax"])
        zmin, zmax = float(raw["zmin"]), float(raw["zmax"])
        if xmax < xmin:
            xmin, xmax = xmax, xmin
        if zmax < zmin:
            zmin, zmax = zmax, zmin
        return {"kind": "box", "xmin": xmin, "xmax": xmax, "zmin": zmin, "zmax": zmax}
    x, z = float(raw["x"]), float(raw["z"])
    r = float(raw.get("radiusM", raw.get("r", 0.4)))
    if not math.isfinite(x) or not math.isfinite(z) or not math.isfinite(r) or r <= 0:
        raise ValueError("Obstacle needs finite x, z and a positive radius")
    return {"kind": "circle", "x": x, "z": z, "r": r}


def inflate(obstacles, radius):
    """Grow each obstacle by the chair radius so a point-robot plan keeps the disc clear."""
    out = []
    for ob in obstacles:
        item = _as_obstacle(ob)
        if item["kind"] == "circle":
            out.append({**item, "r": item["r"] + radius})
        else:
            out.append({
                "kind": "box",
                "xmin": item["xmin"] - radius, "xmax": item["xmax"] + radius,
                "zmin": item["zmin"] - radius, "zmax": item["zmax"] + radius,
            })
    return out


def hits(x, z, obstacles):
    for ob in obstacles:
        if ob["kind"] == "circle":
            if (x - ob["x"]) ** 2 + (z - ob["z"]) ** 2 <= ob["r"] ** 2:
                return True
        elif ob["xmin"] <= x <= ob["xmax"] and ob["zmin"] <= z <= ob["zmax"]:
            return True
    return False


def window(start, goal, obstacles, pad=REACH_PAD_M):
    xs = [start[0], goal[0]]
    zs = [start[1], goal[1]]
    for ob in obstacles:
        if ob["kind"] == "circle":
            xs.extend([ob["x"] - ob["r"], ob["x"] + ob["r"]])
            zs.extend([ob["z"] - ob["r"], ob["z"] + ob["r"]])
        else:
            xs.extend([ob["xmin"], ob["xmax"]])
            zs.extend([ob["zmin"], ob["zmax"]])
    return min(xs) - pad, max(xs) + pad, min(zs) - pad, max(zs) + pad


def straighten(points, obstacles):
    if len(points) < 3:
        return points
    out, i = [points[0]], 0
    while i < len(points) - 1:
        j = i + 1
        while j + 1 < len(points):
            a, b = points[i], points[j + 1]
            steps = max(int(math.hypot(b[0] - a[0], b[1] - a[1]) / (CELL / 2)), 1)
            if any(hits(a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps, obstacles)
                   for k in range(steps + 1)):
                break
            j += 1
        out.append(points[j])
        i = j
    return out


def _grid_viz(x0, z0, nx, nz, free, chair_radius_m):
    """Sparse free + occupied cell samples for the map overlay."""
    # Aim for a readable lattice without flooding the SVG.
    step_c = max(1, (nx + 19) // 20)
    step_r = max(1, (nz + 19) // 20)
    free_pts, occupied = [], []
    for row in range(0, nz, step_r):
        for col in range(0, nx, step_c):
            pt = {"x": round(x0 + col * CELL, 3), "z": round(z0 + row * CELL, 3)}
            if free[row * nx + col]:
                if len(free_pts) < 400:
                    free_pts.append(pt)
            elif len(occupied) < 350:
                occupied.append(pt)
    return {
        "cellM": CELL,
        "chairRadiusM": round(chair_radius_m, 3),
        "bounds": {
            "xmin": round(x0, 3), "xmax": round(x0 + (nx - 1) * CELL, 3),
            "zmin": round(z0, 3), "zmax": round(z0 + (nz - 1) * CELL, 3),
        },
        "free": free_pts,
        "occupied": occupied,
    }


def plan(start, goal, obstacles=(), chair_width_cm=70, clearance_m=CLEARANCE_M):
    """A short path from start=(x,z) to goal=(x,z) that clears the obstacles for this chair.

    Returns {"found": True, "points": [{"x","z"}, ...], "lengthM": ..., "grid": {...}} or
            {"found": False, "reason": "..."}.
    `grid` always includes a sparse free/occupied lattice for the local search window so the
    demo can draw the occupancy grid on the Matterport overlay.
    """
    sx, sz = float(start[0]), float(start[1])
    gx, gz = float(goal[0]), float(goal[1])
    if not all(math.isfinite(v) for v in (sx, sz, gx, gz)):
        raise ValueError("start and goal must be finite")
    radius = chair_radius_m(chair_width_cm, clearance_m)
    grown = inflate(list(obstacles or []), radius)

    def build_grid():
        x0, x1, z0, z1 = window((sx, sz), (gx, gz), grown)
        nx = int((x1 - x0) / CELL) + 1
        nz = int((z1 - z0) / CELL) + 1
        if nx * nz > MAX_CELLS:
            return None
        free = [True] * (nx * nz)
        for row in range(nz):
            for col in range(nx):
                if hits(x0 + col * CELL, z0 + row * CELL, grown):
                    free[row * nx + col] = False
        return _grid_viz(x0, z0, nx, nz, free, radius)

    def with_grid(payload):
        grid = build_grid()
        if grid:
            payload["grid"] = grid
        return payload

    if hits(sx, sz, grown):
        return with_grid({"found": False, "reason": "The chair is already inside an obstacle (or too close to one)."})
    if hits(gx, gz, grown):
        return with_grid({"found": False, "reason": "The next waypoint sits inside an obstacle. Ask the global planner for another route."})

    # Straight line is fine when nothing is in the way.
    direct = math.hypot(gx - sx, gz - sz)
    steps = max(int(direct / (CELL / 2)), 1)
    if not any(hits(sx + (gx - sx) * k / steps, sz + (gz - sz) * k / steps, grown) for k in range(steps + 1)):
        return with_grid({
            "found": True,
            "points": [{"x": round(sx, 3), "z": round(sz, 3)}, {"x": round(gx, 3), "z": round(gz, 3)}],
            "lengthM": round(direct, 2),
            "mode": "direct",
        })

    x0, x1, z0, z1 = window((sx, sz), (gx, gz), grown)
    nx = int((x1 - x0) / CELL) + 1
    nz = int((z1 - z0) / CELL) + 1
    if nx * nz > MAX_CELLS:
        return {"found": False, "reason": "The local search window is too large for a detour. Treat the hallway as blocked."}

    def index(x, z):
        col = min(max(int(round((x - x0) / CELL)), 0), nx - 1)
        row = min(max(int(round((z - z0) / CELL)), 0), nz - 1)
        return row * nx + col

    def point(i):
        row, col = divmod(i, nx)
        return x0 + col * CELL, z0 + row * CELL

    free = [True] * (nx * nz)
    for row in range(nz):
        for col in range(nx):
            x, z = x0 + col * CELL, z0 + row * CELL
            if hits(x, z, grown):
                free[row * nx + col] = False

    grid = _grid_viz(x0, z0, nx, nz, free, radius)

    start_i, goal_i = index(sx, sz), index(gx, gz)
    if not free[start_i] or not free[goal_i]:
        # Snap to nearest free cell within ~1 m.
        def snap(seed, target):
            best, best_d = None, 1.0
            tr, tc = divmod(seed, nx)
            reach = int(1.0 / CELL) + 1
            for row in range(max(tr - reach, 0), min(tr + reach + 1, nz)):
                for col in range(max(tc - reach, 0), min(tc + reach + 1, nx)):
                    i = row * nx + col
                    if not free[i]:
                        continue
                    px, pz = point(i)
                    d = math.hypot(px - target[0], pz - target[1])
                    if d < best_d:
                        best, best_d = i, d
            return best
        start_i = snap(start_i, (sx, sz)) if not free[start_i] else start_i
        goal_i = snap(goal_i, (gx, gz)) if not free[goal_i] else goal_i
        if start_i is None or goal_i is None:
            return {"found": False, "reason": "No free space near the chair or the next waypoint for a detour.", "grid": grid}

    g_score, came, done = {start_i: 0.0}, {start_i: None}, set()
    heap = [(math.hypot(*([a - b for a, b in zip(point(start_i), point(goal_i))])), start_i)]
    while heap:
        _, current = heapq.heappop(heap)
        if current == goal_i:
            cells = []
            while current is not None:
                cells.append(current)
                current = came[current]
            cells.reverse()
            pts = [(sx, sz)] + [point(i) for i in cells[1:-1]] + [(gx, gz)]
            pts = straighten(pts, grown)
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
            return {
                "found": True,
                "points": [{"x": round(x, 3), "z": round(z, 3)} for x, z in pts],
                "lengthM": round(length, 2),
                "mode": "detour",
                "grid": grid,
            }
        if current in done:
            continue
        done.add(current)
        row, col = divmod(current, nx)
        for dr, dc, step in STEPS:
            r, c = row + dr, col + dc
            if not (0 <= r < nz and 0 <= c < nx):
                continue
            n = r * nx + c
            if not free[n] or n in done:
                continue
            total = g_score[current] + step * CELL
            if total < g_score.get(n, math.inf):
                g_score[n], came[n] = total, current
                px, pz = point(n)
                heapq.heappush(heap, (total + math.hypot(px - gx, pz - gz), n))

    return {
        "found": False,
        "reason": "No collision-free detour around the obstacle. The hallway is blocked for this chair.",
        "grid": grid,
    }


def blocks_segment(a, b, obstacles, chair_width_cm=70, clearance_m=CLEARANCE_M):
    """True when the straight segment from a to b collides for this chair."""
    grown = inflate(list(obstacles or []), chair_radius_m(chair_width_cm, clearance_m))
    ax, az = float(a[0]), float(a[1])
    bx, bz = float(b[0]), float(b[1])
    steps = max(int(math.hypot(bx - ax, bz - az) / (CELL / 2)), 1)
    return any(hits(ax + (bx - ax) * k / steps, az + (bz - az) * k / steps, grown) for k in range(steps + 1))
