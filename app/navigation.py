"""Wheelchair routes on a Matterport scan-point graph: the global planner for the wheelchair demo.

Matterport links every pair of scan points that can see each other, across rooms, atriums and floors, so its graph
is not a map of where a chair can drive. This module

  1. thins that graph to the hops between neighbouring scan points,
  2. applies the team's labels from catalog/nav/<model>.json (corridor, doorway, ramp, elevator, stairs, blocked),
  3. runs A* over the hops somebody has verified, and
  4. describes the route as waypoints and legs (distance, heading, turn) for the chair's local planner.

Positions are Matterport's: metres, y up. Headings are degrees counter-clockwise seen from above, 0 along +x and
90 along -z, so a positive turn is a turn to the left.

Only the standard library is used, so the chair's computer can import this or run it without the web server:

    python3 -m app.navigation 2K7G7cdukZV                  # what is labelled so far
    python3 -m app.navigation 2K7G7cdukZV 12 "Room 105"    # route from scan 12 to a named place
"""
import heapq
import json
import math
import re
import sys
import threading
import time
from datetime import datetime, timezone

from .config import ROOT

NAV = ROOT / "catalog/nav"
MODEL_ID = re.compile(r"[A-Za-z0-9]{11}")
SWEEP_ID = re.compile(r"[A-Za-z0-9_-]{1,40}")

LEVEL_M = 0.08  # scans on one floor agree to a few centimetres; more than this is a step, a ramp or stairs
STAIR_SLOPE = 0.85  # rise over run; anything steeper is two scans that only see each other across a drop
STOREY_M = 2.5  # and so is a whole storey in one hop: stairs get scanned on the way up
RAMP_SLOPE = 1 / 12
CAMERA_M = 1.35  # tripod height, for a scan that has no floor point
ELEVATOR_COST_M = 15.0  # waiting for an elevator counts as this much extra driving
DRIVABLE = ("corridor", "doorway", "ramp", "elevator")
TYPES = DRIVABLE + ("stairs", "blocked", "unknown")
POSE_STATUS = ("following", "detour", "blocked", "arrived", "stopped")

_write = threading.Lock()


def edge_id(a, b):
    return "~".join(sorted((a, b)))


def check_model(model):
    if not MODEL_ID.fullmatch(model or ""):
        raise ValueError("Not a Matterport model id")
    return model


def sweeps_path(model):
    return NAV / f"{check_model(model)}.sweeps.json"


def labels_path(model):
    return NAV / f"{check_model(model)}.json"


def number(value, low=-1e5, high=1e5):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError("Expected a number")
    return round(float(value), 3)


def save_json(path, data):
    with _write:
        path.parent.mkdir(parents=True, exist_ok=True)
        scratch = path.with_suffix(".tmp")
        scratch.write_text(json.dumps(data, indent=1))
        scratch.replace(path)


# 1. The scan points, as the viewer page reads them from the Matterport SDK.

def save_sweeps(model, payload):
    """Check and store the scan points and their lines of sight."""
    sweeps = payload.get("sweeps")
    if not isinstance(sweeps, list) or not 2 <= len(sweeps) <= 5000:
        raise ValueError("Expected a list of scan points")
    out = []
    for s in sweeps:
        if not SWEEP_ID.fullmatch(str(s.get("id", ""))):
            raise ValueError("Bad scan point id")
        floor = s.get("floor")
        out.append({
            "id": s["id"], "x": number(s["x"]), "y": number(s["y"]), "z": number(s["z"]),
            "puckY": None if s.get("puckY") is None else number(s["puckY"]),
            "floor": floor if isinstance(floor, int) and not isinstance(floor, bool) else None,
            "neighbors": [n for n in s.get("neighbors") or [] if isinstance(n, str) and SWEEP_ID.fullmatch(n)],
        })
    if len({s["id"] for s in out}) != len(out):
        raise ValueError("Duplicate scan point")
    floors = [{"sequence": f["sequence"], "name": str(f.get("name") or f"Floor {f['sequence'] + 1}")[:40]}
              for f in payload.get("floors") or [] if isinstance(f.get("sequence"), int)]
    save_json(sweeps_path(model), {"model": model, "savedAt": now(), "floors": floors, "sweeps": out})
    return len(out)


def now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def read(path, default):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default


# 2. From lines of sight to hops, with the team's labels on top.

def hops(nodes, sight):
    """Thin the line-of-sight graph to hops between neighbouring scan points.

    A sight line is dropped when a third scan point, seen from both ends, stands inside the circle that has the line
    as its diameter: the way goes through that point instead. Both shorter lines are kept or replaced the same way,
    so nothing that was connected gets cut off. A level line is only replaced by scans on its own level; otherwise
    a scan on the balcony above could stand in for the floor below it.
    """
    def squared(a, b):
        p, q = nodes[a], nodes[b]
        return (p["x"] - q["x"]) ** 2 + (p["y"] - q["y"]) ** 2 + (p["z"] - q["z"]) ** 2

    def level(a, b):
        return abs(nodes[a]["y"] - nodes[b]["y"]) <= LEVEL_M

    kept = []
    for a in sorted(sight):
        for b in sorted(sight[a]):
            if a >= b:
                continue
            flat, across = level(a, b), squared(a, b)
            between = (c for c in sight[a] & sight[b] if not flat or (level(a, c) and level(b, c)))
            if not any(squared(a, c) + squared(c, b) < across for c in between):
                kept.append((a, b))
    return kept


def measure(nodes, a, b):
    p, q = nodes[a], nodes[b]
    run, rise = math.hypot(q["x"] - p["x"], q["z"] - p["z"]), abs(q["y"] - p["y"])
    return {"id": edge_id(a, b), "a": a, "b": b, "lengthM": round(math.hypot(run, rise), 2),
            "runM": round(run, 2), "riseM": round(rise, 2)}


def guess(edge):
    """What the scan heights alone say about a hop. Level hops stay unknown until a person has looked."""
    run, rise = edge["runM"], edge["riseM"]
    if rise <= LEVEL_M:
        return "unknown", None
    if rise <= STOREY_M and run and rise / run <= STAIR_SLOPE:
        gentle = " (gentle enough for a ramp: check on site)" if rise / run <= RAMP_SLOPE else ""
        return "stairs", f"The floor height changes by {rise:.2f} m{gentle}"
    return "blocked", f"Line of sight across a {rise:.1f} m drop"


class Graph:
    def __init__(self, model, snapshot, labels):
        self.model = model
        self.floors = snapshot.get("floors") or []
        sweeps = snapshot["sweeps"]

        def height(s):
            return s["puckY"] if s.get("puckY") is not None else s["y"] - CAMERA_M

        ordered = sorted(sweeps, key=lambda s: (s["floor"] if s["floor"] is not None else 99, s["x"], s["z"]))
        self.nodes = {s["id"]: {"id": s["id"], "scan": i + 1, "x": s["x"], "y": round(height(s), 3), "z": s["z"],
                                "floor": s["floor"]} for i, s in enumerate(ordered)}
        sight = {s["id"]: set() for s in sweeps}
        for s in sweeps:
            for n in s["neighbors"]:
                if n in sight and n != s["id"]:
                    sight[s["id"]].add(n)
                    sight[n].add(s["id"])
        self.sightlines = sum(len(v) for v in sight.values()) // 2

        self.edges = {}
        for a, b in hops(self.nodes, sight):
            edge = measure(self.nodes, a, b)
            edge["type"], reason = guess(edge)
            edge.update(source="auto", accessible=False)
            if reason:
                edge["reason"] = reason
            self.edges[edge["id"]] = edge
        for label in labels.get("edges") or []:
            a, b, kind = label.get("from"), label.get("to"), label.get("type")
            if a not in self.nodes or b not in self.nodes or a == b or kind not in TYPES or kind == "unknown":
                continue
            edge = self.edges.get(edge_id(a, b))
            if edge is None:
                edge = self.edges[edge_id(a, b)] = {**measure(self.nodes, a, b), "source": "added"}
            else:
                edge["source"] = "label"
            edge.pop("reason", None)
            edge["type"] = kind
            # "Accessible" is a person's statement about the hop, so a hand-edited false always wins.
            edge["accessible"] = kind in DRIVABLE and label.get("wheelchairAccessible") is not False
            for field in ("widthCm", "note"):
                if label.get(field):
                    edge[field] = label[field]

        self.next = {n: [] for n in self.nodes}
        for edge in self.edges.values():
            self.next[edge["a"]].append((edge["b"], edge))
            self.next[edge["b"]].append((edge["a"], edge))
        self.destinations = [d for d in labels.get("destinations") or [] if d.get("sweep") in self.nodes]

    def resolve(self, ref):
        """A scan point from its id, its number on the map, or the name of a saved place."""
        ref = str(ref).strip()
        if ref in self.nodes:
            return ref
        for place in self.destinations:
            if ref.lower() in (place["id"].lower(), place["name"].lower()):
                return place["sweep"]
        if ref.isdigit():
            for node in self.nodes.values():
                if node["scan"] == int(ref):
                    return node["id"]
        raise ValueError(f"No scan point or place called {ref!r}")

    def as_dict(self):
        counts = {kind: 0 for kind in TYPES}
        for edge in self.edges.values():
            counts[edge["type"]] += 1
        return {"model": self.model, "floors": self.floors, "nodes": list(self.nodes.values()),
                "edges": list(self.edges.values()), "destinations": self.destinations, "types": list(TYPES),
                "drivable": list(DRIVABLE), "sightlines": self.sightlines, "counts": counts}


def load(model):
    """The labelled graph, or None while the model's scan points have not been saved."""
    snapshot = read(sweeps_path(model), None)
    if not snapshot or not snapshot.get("sweeps"):
        return None
    return Graph(model, snapshot, read(labels_path(model), {}))


def save_label(model, a, b, kind, width_cm=None, note=None):
    """Label one hop. "unknown" removes the label."""
    graph = load(model)
    if graph is None or a not in graph.nodes or b not in graph.nodes or a == b:
        raise ValueError("No such pair of scan points")
    if kind not in TYPES:
        raise ValueError("Unknown label")
    labels = read(labels_path(model), {})
    edges = [e for e in labels.get("edges") or [] if edge_id(e.get("from", ""), e.get("to", "")) != edge_id(a, b)]
    if kind != "unknown":
        label = {"from": a, "to": b, "type": kind, "wheelchairAccessible": kind in DRIVABLE}
        if width_cm:
            label["widthCm"] = number(width_cm, 20, 1000)
        if note:
            label["note"] = str(note)[:200]
        edges.append(label)
    save_json(labels_path(model), {**labels, "version": 1, "model": model, "updatedAt": now(), "edges": edges})
    return load(model)


def save_places(model, places):
    graph = load(model)
    if graph is None:
        raise ValueError("No scan points saved for this model")
    out, seen = [], set()
    for place in places[:200]:
        name = str(place.get("name") or "").strip()[:60]
        if not name or place.get("sweep") not in graph.nodes:
            raise ValueError("A place needs a name and a scan point")
        slug = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-") or "place"
        while slug in seen:
            slug += "-2"
        seen.add(slug)
        out.append({"id": slug, "name": name, "sweep": place["sweep"]})
    labels = read(labels_path(model), {})
    save_json(labels_path(model), {**labels, "version": 1, "model": model, "updatedAt": now(),
                                   "edges": labels.get("edges") or [], "destinations": out})
    return load(model)


# 3. A* over the Matterport hop graph.

def drivable(edge, preview=False, elevators=True, min_width_cm=None, blocked=()):
    """Whether the wheelchair planner may use this hop.

    Level Matterport hops (`unknown`) are used automatically — that is the dynamic A* graph.
    Stairs and blocked hops are never used. Explicit corridor/doorway/ramp/elevator labels still
    apply when present (including width checks). `preview` is kept for older callers; unknowns
    are allowed either way.
    """
    if edge["id"] in blocked:
        return False
    kind = edge["type"]
    if kind in ("stairs", "blocked"):
        return False
    if kind == "elevator" and not elevators:
        return False
    if kind == "unknown":
        return True
    if not edge["accessible"]:
        return False
    return not (min_width_cm and edge.get("widthCm") and edge["widthCm"] < min_width_cm)


def walkable(edge):
    return edge["type"] != "blocked"


def astar(graph, start, goal, allowed):
    def remaining(n):
        p, q = graph.nodes[n], graph.nodes[goal]
        return math.sqrt((p["x"] - q["x"]) ** 2 + (p["y"] - q["y"]) ** 2 + (p["z"] - q["z"]) ** 2)

    best, came, done = {start: 0.0}, {start: None}, set()
    heap = [(remaining(start), start)]
    while heap:
        _, current = heapq.heappop(heap)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came[current]
            return path[::-1]
        if current in done:
            continue
        done.add(current)
        for other, edge in graph.next[current]:
            if other in done or not allowed(edge):
                continue
            # A hop is as long as the straight line between its ends, so the estimate stays a lower bound.
            total = best[current] + edge["lengthM"] + (ELEVATOR_COST_M if edge["type"] == "elevator" else 0)
            if total < best.get(other, math.inf):
                best[other], came[other] = total, current
                heapq.heappush(heap, (total + remaining(other), other))
    return None


# 4. The route in the chair's terms.

def turn(deg):
    return round((deg + 180) % 360 - 180, 1)


def describe(graph, path, heading_deg=None):
    """Waypoints and legs. Each leg says how far to drive and how far to turn first; the first turn is from
    heading_deg, or zero when the chair starts out facing its first waypoint. After an elevator nobody knows
    which way the chair faces, so that leg's turn is None: the chair has to be told its heading again."""
    nodes = [graph.nodes[n] for n in path]
    legs, heading, carried = [], heading_deg, False
    for p, q in zip(nodes, nodes[1:]):
        edge = graph.edges[edge_id(p["id"], q["id"])]
        leg = {"from": p["id"], "to": q["id"], "type": edge["type"], "verified": edge["type"] != "unknown"}
        if edge["type"] == "elevator":
            leg.update(distanceM=0.0, headingDeg=None, turnDeg=0.0, riseM=round(q["y"] - p["y"], 2))
            carried = True
        else:
            toward = math.degrees(math.atan2(-(q["z"] - p["z"]), q["x"] - p["x"]))
            leg.update(distanceM=edge["runM"], headingDeg=round(toward, 1),
                       turnDeg=None if carried else 0.0 if heading is None else turn(toward - heading))
            heading, carried = toward, False
        for field in ("widthCm", "note"):
            if edge.get(field):
                leg[field] = edge[field]
        legs.append(leg)
    return {
        "waypoints": [{k: n[k] for k in ("id", "scan", "x", "y", "z", "floor")} for n in nodes],
        "legs": legs,
        "lengthM": round(sum(leg["distanceM"] for leg in legs), 2),
        "unverifiedLegs": sum(not leg["verified"] for leg in legs),
        "usesElevator": any(leg["type"] == "elevator" for leg in legs),
    }


def reject_reason(graph, start, goal, elevators=True, min_width_cm=None, blocked=()):
    """Explain why no wheelchair A* route exists (for the demo UI)."""
    blocked = set(blocked)
    if not astar(graph, start, goal, lambda e: walkable(e) and e["type"] != "stairs"
                 and e["id"] not in blocked and not (e["type"] == "elevator" and not elevators)):
        return ("No step-free connection between these places. Stairs or gaps separate them, "
                "or every remaining hop is blocked.")
    if min_width_cm and astar(graph, start, goal, lambda e: drivable(
            e, elevators=elevators, min_width_cm=None, blocked=blocked)):
        return (f"Every route has a passage narrower than the {min_width_cm:.0f} cm chair. "
                "Pick a narrower chair profile.")
    if not elevators and astar(graph, start, goal, lambda e: drivable(
            e, elevators=True, min_width_cm=min_width_cm, blocked=blocked)):
        return "The only wheelchair routes use an elevator, and elevators are turned off."
    if blocked:
        return "No wheelchair route remains after the blocked hallway(s) were closed."
    return "No wheelchair route between these places."


def route(graph, start, goal, preview=False, elevators=True, min_width_cm=None, blocked=(), heading_deg=None):
    """A* wheelchair route on the Matterport hop graph (stairs/blocked excluded)."""
    blocked = set(blocked)
    options = dict(elevators=elevators, min_width_cm=min_width_cm, blocked=blocked)
    path = astar(graph, start, goal, lambda e: drivable(e, preview, **options))
    result = {"start": start, "goal": goal, "mode": "graph", "found": path is not None}
    if path:
        return {**result, **describe(graph, path, heading_deg)}
    result.update(previewFound=False, reason=reject_reason(graph, start, goal, elevators, min_width_cm, blocked))
    return result


def walking(graph, start, goal):
    """The shortest way on foot, stairs included, for comparison with the wheelchair route."""
    path = astar(graph, start, goal, walkable)
    return {"found": True, **describe(graph, path)} if path else {"found": False}


def obstacles_path(model):
    return NAV / f"{check_model(model)}.obstacles.json"


def load_obstacles(model):
    """Manually annotated collision geometry for the demo (circles or boxes on the floor plane)."""
    path = obstacles_path(model)
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    return list(data.get("obstacles") or [])


def save_obstacles(model, obstacles):
    from . import local_planner
    cleaned = []
    for raw in obstacles or []:
        item = local_planner._as_obstacle(raw)
        if item["kind"] == "circle":
            cleaned.append({"id": str(raw.get("id") or f"obs-{len(cleaned)+1}")[:40],
                            "x": round(item["x"], 3), "z": round(item["z"], 3), "radiusM": round(item["r"], 3)})
        else:
            cleaned.append({"id": str(raw.get("id") or f"obs-{len(cleaned)+1}")[:40],
                            "xmin": round(item["xmin"], 3), "xmax": round(item["xmax"], 3),
                            "zmin": round(item["zmin"], 3), "zmax": round(item["zmax"], 3)})
        if len(cleaned) > 200:
            raise ValueError("At most 200 obstacles")
    save_json(obstacles_path(model), {"model": model, "updatedAt": now(), "obstacles": cleaned})
    return cleaned


class Trip:
    """One journey: its route, the hops found blocked on the way, and where the chair says it is."""

    def __init__(self, graph, start, goal, preview=False, elevators=True, min_width_cm=None, heading_deg=None,
                 obstacles=None, mode="simulated"):
        self.model, self.goal, self.start = graph.model, goal, start
        self.options = {"preview": bool(preview), "elevators": bool(elevators), "min_width_cm": min_width_cm}
        self.blocked, self.log, self.pose, self.revision = [], [], None, 0
        self.obstacles = list(obstacles if obstacles is not None else load_obstacles(graph.model))
        self.mode = mode if mode in ("simulated", "live") else "simulated"
        self.walking = walking(graph, start, goal)
        self.plan(graph, start, heading_deg)
        # Odometry is measured from here: the first scan point, facing the given heading or else the first waypoint.
        first = next((leg["headingDeg"] for leg in self.route.get("legs", []) if leg["headingDeg"] is not None), 0.0)
        origin = graph.nodes[start]
        self.origin = {"x": origin["x"], "z": origin["z"], "headingDeg": first if heading_deg is None else heading_deg}

    def note(self, text):
        self.log = (self.log + [{"at": round(time.time(), 2), "text": text}])[-30:]

    def plan(self, graph, start, heading_deg=None):
        self.route = route(graph, start, self.goal, blocked=self.blocked, heading_deg=heading_deg, **self.options)
        self.revision += 1
        if self.route["found"]:
            self.status = "planned"
            self.note(f"Route planned: {len(self.route['waypoints'])} waypoints, {self.route['lengthM']} m")
        else:
            self.status = "no-route"
            self.note(self.route["reason"])

    def set_width(self, graph, chair_width_cm, heading_deg=None):
        """Chair profile changed: replan from the current pose (or start). Never resume a failed plan."""
        width = None if chair_width_cm is None else number(chair_width_cm, 20, 200)
        self.options["min_width_cm"] = width
        self.note(f"Chair width set to {width:.0f} cm" if width else "Chair width cleared")
        resume = self.start
        if self.pose and self.route.get("waypoints"):
            # Nearest waypoint behind or at the chair.
            resume = min(self.route["waypoints"],
                         key=lambda w: math.hypot(w["x"] - self.pose["x"], w["z"] - self.pose["z"]))["id"]
        if heading_deg is None and self.pose:
            heading_deg = self.pose.get("headingDeg")
        self.plan(graph, resume, heading_deg)
        if not self.route["found"]:
            self.status = "no-route"
            self.note("Replan after chair-width change failed. The chair stays stopped.")
        else:
            self.status = "replanned"
        return self.route

    def set_obstacles(self, graph, obstacles, heading_deg=None):
        """Replace the session obstacle list and replan if the current leg is now blocked."""
        from . import local_planner
        self.obstacles = list(obstacles or [])
        # Validate shapes early.
        for ob in self.obstacles:
            local_planner._as_obstacle(ob)
        self.note(f"{len(self.obstacles)} obstacle(s) on the floor")
        width = self.options.get("min_width_cm") or 70
        if self.route.get("found") and self.pose and self.pose.get("waypoint") is not None:
            waypoints = self.route["waypoints"]
            idx = min(max(self.pose["waypoint"], 1), len(waypoints) - 1)
            a, b = waypoints[idx - 1], waypoints[idx]
            if local_planner.blocks_segment((self.pose["x"], self.pose["z"]), (b["x"], b["z"]),
                                           self.obstacles, width):
                return self.report_blocked(graph, a["id"], b["id"], heading_deg)
        return self.route

    def report_blocked(self, graph, a, b, heading_deg=None):
        """The local planner found no way along the hop a to b. Close it for this trip and plan again from a."""
        if edge_id(a, b) not in graph.edges:
            raise ValueError("No such hop")
        if edge_id(a, b) not in self.blocked:
            self.blocked.append(edge_id(a, b))
        self.note(f"Blocked between scan {graph.nodes[a]['scan']} and scan {graph.nodes[b]['scan']}")
        if heading_deg is None and self.pose:
            heading_deg = self.pose["headingDeg"]
        self.plan(graph, a, heading_deg)
        if self.route["found"]:
            self.status = "replanned"
        else:
            self.status = "no-route"
            self.note("No verified way around. The chair stays stopped.")
        return self.route

    def report_pose(self, pose):
        """Where the chair is: {"frame": "map", x, z, headingDeg} in Matterport metres, or {"frame": "odom", forwardM,
        leftM, turnDeg} measured from where the trip began. Optional: status, waypoint (index of the waypoint it is
        heading for) and detour (the local planner's path, as [x, z] or [forwardM, leftM] pairs)."""
        frame = pose.get("frame", "map")
        if frame == "odom":
            place = lambda forward, left: self.from_odom(number(forward), number(left))
            x, z = place(pose["forwardM"], pose["leftM"])
            heading = self.origin["headingDeg"] + number(pose.get("turnDeg", 0))
        elif frame == "map":
            place = lambda px, pz: (number(px), number(pz))
            x, z = place(pose["x"], pose["z"])
            heading = pose.get("headingDeg")
            heading = None if heading is None else number(heading)
        else:
            raise ValueError("frame is map or odom")
        status = pose.get("status", "following")
        if status not in POSE_STATUS:
            raise ValueError("status is one of " + ", ".join(POSE_STATUS))
        waypoints = self.route.get("waypoints") or []
        index = pose.get("waypoint")
        if index is not None and (isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < max(len(waypoints), 1)):
            raise ValueError("waypoint is an index into the route")
        detour = pose.get("detour") or []
        if not isinstance(detour, list) or len(detour) > 400:
            raise ValueError("detour is a list of at most 400 points")
        # Height and floor come from the map: the waypoint the chair last left, or else the nearest one.
        if index is not None and waypoints:
            under = waypoints[max(index - 1, 0)]
        else:
            under = min(waypoints, key=lambda w: math.hypot(w["x"] - x, w["z"] - z), default=None)
        self.pose = {
            "x": round(x, 3), "z": round(z, 3), "y": under["y"] if under else 0.0, "floor": under["floor"] if under else None,
            "headingDeg": None if heading is None else turn(heading), "waypoint": index,
            "detour": [[round(v, 3) for v in place(p[0], p[1])] for p in detour],
            "source": str(pose.get("source") or "wheelchair")[:20], "at": round(time.time(), 2),
        }
        if status != self.status:
            self.note({"following": "Following the route", "detour": "Local detour around an obstacle",
                       "blocked": "Stopped: the way ahead is blocked", "arrived": "Arrived",
                       "stopped": "Stopped"}[status])
        self.status = status
        return self.pose

    def from_odom(self, forward, left):
        theta = math.radians(self.origin["headingDeg"])
        east = forward * math.cos(theta) - left * math.sin(theta)
        north = forward * math.sin(theta) + left * math.cos(theta)
        return self.origin["x"] + east, self.origin["z"] - north

    def state(self):
        return {"model": self.model, "revision": self.revision, "status": self.status, "start": self.start,
                "goal": self.goal, "options": self.options, "origin": self.origin, "route": self.route,
                "walking": self.walking, "blocked": self.blocked, "pose": self.pose, "log": self.log,
                "obstacles": self.obstacles, "mode": self.mode}


def main(argv):
    if not argv or len(argv) not in (1, 3):
        print(__doc__.strip().split("\n\n")[-1])
        return 2
    graph = load(argv[0])
    if graph is None:
        print(f"No scan points saved for {argv[0]}. Open /nav.html?m={argv[0]} once to read them from Matterport.")
        return 1
    if len(argv) == 1:
        counts = graph.as_dict()["counts"]
        print(f"{len(graph.nodes)} scan points, {graph.sightlines} lines of sight, {len(graph.edges)} hops")
        print("  " + ", ".join(f"{n} {kind}" for kind, n in counts.items() if n))
        for place in graph.destinations:
            print(f"  place: {place['name']} (scan {graph.nodes[place['sweep']]['scan']})")
        return 0
    try:
        start, goal = graph.resolve(argv[1]), graph.resolve(argv[2])
    except ValueError as error:
        print(error)
        return 1
    result = route(graph, start, goal)
    if not result["found"]:
        print(result["reason"])
        return 1
    scan = {w["id"]: w["scan"] for w in result["waypoints"]}
    print(f"{result['lengthM']} m over {len(result['legs'])} verified hops")
    for leg in result["legs"]:
        if leg["type"] == "elevator":
            print(f"  elevator from scan {scan[leg['from']]} to scan {scan[leg['to']]}: needs someone to call it")
        elif leg["turnDeg"] is None:
            print(f"  face {leg['headingDeg']:.0f} deg on the map, drive {leg['distanceM']:.2f} m to scan {scan[leg['to']]} ({leg['type']})")
        else:
            side = "left" if leg["turnDeg"] > 0 else "right"
            turning = f"turn {abs(leg['turnDeg']):.0f} deg {side}, " if abs(leg["turnDeg"]) >= 1 else ""
            print(f"  {turning}drive {leg['distanceM']:.2f} m to scan {scan[leg['to']]} ({leg['type']})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
