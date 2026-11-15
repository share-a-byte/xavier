"""HTTP face of the wheelchair navigator: the map page and the chair's local planner both talk to this.

    GET  /api/nav/{model}                 the labelled graph
    PUT  /api/nav/{model}/sweeps          scan points read from the Matterport SDK by the page
    PUT  /api/nav/{model}/label           label one hop
    PUT  /api/nav/{model}/places          named destinations
    GET  /api/nav/{model}/obstacles       annotated collision geometry
    PUT  /api/nav/{model}/obstacles       replace annotated collision geometry
    GET  /api/nav/{model}/route           a route, without starting a trip
    POST /api/nav/{model}/local           local detour around obstacles toward a waypoint
    POST /api/nav/{model}/trip            start a trip        GET the same path: its current state
    POST /api/nav/{model}/trip/pose       the chair reports where it is
    POST /api/nav/{model}/trip/blocked    the chair reports a hop it cannot get through; replies with the new route
    POST /api/nav/{model}/trip/replan     replan after chair width or obstacles change
"""
from fastapi import APIRouter, HTTPException, Request

from . import local_planner, navigation

router = APIRouter(prefix="/api/nav")
TRIPS = {}


def graph_of(model):
    try:
        graph = navigation.load(model)
    except ValueError as error:
        raise HTTPException(404, str(error))
    if graph is None:
        raise HTTPException(404, "No scan points saved for this model yet")
    return graph


async def body_of(request, limit=400_000):
    raw = await request.body()
    if len(raw) > limit:
        raise HTTPException(413)
    try:
        data = await request.json()
    except ValueError:
        raise HTTPException(400, "Expected JSON")
    if not isinstance(data, dict):
        raise HTTPException(400, "Expected a JSON object")
    return data


def checked(action):
    """Run a planner call, turning what it rejects into a 400."""
    try:
        return action()
    except (ValueError, KeyError, TypeError, IndexError) as error:
        raise HTTPException(400, str(error) or type(error).__name__)


def trip_of(model):
    trip = TRIPS.get(model)
    if trip is None:
        raise HTTPException(404, "No trip has been started")
    return trip


@router.get("/{model}")
def graph(model: str):
    try:
        found = navigation.load(model)
    except ValueError as error:
        raise HTTPException(404, str(error))
    return {"saved": False, "model": model} if found is None else {"saved": True, **found.as_dict()}


@router.put("/{model}/sweeps")
async def put_sweeps(model: str, request: Request):
    data = await body_of(request, 4_000_000)
    checked(lambda: navigation.save_sweeps(model, data))
    return {"saved": True, **graph_of(model).as_dict()}


@router.put("/{model}/label")
async def put_label(model: str, request: Request):
    data = await body_of(request)
    found = checked(lambda: navigation.save_label(model, data["from"], data["to"], data["type"], data.get("widthCm"), data.get("note")))
    return {"saved": True, **found.as_dict()}


@router.put("/{model}/places")
async def put_places(model: str, request: Request):
    data = await body_of(request)
    found = checked(lambda: navigation.save_places(model, data["places"]))
    return {"saved": True, **found.as_dict()}


@router.get("/{model}/obstacles")
def get_obstacles(model: str):
    navigation.check_model(model)
    return {"model": model, "obstacles": navigation.load_obstacles(model)}


@router.put("/{model}/obstacles")
async def put_obstacles(model: str, request: Request):
    data = await body_of(request)
    obstacles = checked(lambda: navigation.save_obstacles(model, data.get("obstacles") or []))
    trip = TRIPS.get(model)
    if trip is not None:
        found = graph_of(model)
        checked(lambda: trip.set_obstacles(found, obstacles, data.get("headingDeg")))
        return {"saved": True, "obstacles": obstacles, "trip": trip.state()}
    return {"saved": True, "obstacles": obstacles}


@router.get("/{model}/route")
def get_route(model: str, start: str, goal: str, preview: bool = False, elevators: bool = True,
              min_width_cm: float | None = None, blocked: str = "", heading_deg: float | None = None):
    found = graph_of(model)
    a, b = checked(lambda: (found.resolve(start), found.resolve(goal)))
    closed = [e for e in blocked.split(",") if e]
    return {"route": navigation.route(found, a, b, preview, elevators, min_width_cm, closed, heading_deg),
            "walking": navigation.walking(found, a, b)}


@router.post("/{model}/local")
async def local_detour(model: str, request: Request):
    """Local A* around obstacles toward the next global waypoint. Planning-only; no rendering."""
    data = await body_of(request)
    navigation.check_model(model)

    def run():
        start = data["start"]
        goal = data["goal"]
        sx, sz = float(start["x"]), float(start["z"])
        gx, gz = float(goal["x"]), float(goal["z"])
        width = data.get("chairWidthCm", data.get("minWidthCm", 70))
        obstacles = list(data.get("obstacles") or [])
        trip = TRIPS.get(model)
        if trip is not None and not obstacles:
            obstacles = list(trip.obstacles)
        return local_planner.plan((sx, sz), (gx, gz), obstacles, width)

    return checked(run)


@router.post("/{model}/trip")
async def start_trip(model: str, request: Request):
    data, found = await body_of(request), graph_of(model)

    def begin():
        start, goal = found.resolve(data["start"]), found.resolve(data["goal"])
        width, heading = data.get("minWidthCm"), data.get("headingDeg")
        obstacles = data.get("obstacles")
        if obstacles is None:
            obstacles = navigation.load_obstacles(model)
        return navigation.Trip(
            found, start, goal, bool(data.get("preview")), data.get("elevators", True) is not False,
            None if width is None else navigation.number(width, 20, 1000),
            None if heading is None else navigation.number(heading),
            obstacles=obstacles, mode=str(data.get("mode") or "simulated"),
        )

    TRIPS[model] = checked(begin)
    return TRIPS[model].state()


@router.get("/{model}/trip")
def get_trip(model: str):
    trip = TRIPS.get(model)
    return {"model": model, "revision": 0, "status": "idle"} if trip is None else trip.state()


@router.post("/{model}/trip/pose")
async def post_pose(model: str, request: Request):
    data, trip = await body_of(request), trip_of(model)
    checked(lambda: trip.report_pose(data))
    # The chair learns here whether its route has changed since it last looked.
    return {"revision": trip.revision, "status": trip.status, "mode": trip.mode}


@router.post("/{model}/trip/blocked")
async def post_blocked(model: str, request: Request):
    data, trip, found = await body_of(request), trip_of(model), graph_of(model)

    def close():
        a, b = found.resolve(data["from"]), found.resolve(data["to"])
        heading = data.get("headingDeg")
        return trip.report_blocked(found, a, b, None if heading is None else navigation.number(heading))

    checked(close)
    return trip.state()


@router.post("/{model}/trip/replan")
async def replan_trip(model: str, request: Request):
    """Replan after the chair width or the obstacle list changes. Failed plans stay stopped."""
    data, trip, found = await body_of(request), trip_of(model), graph_of(model)

    def run():
        heading = data.get("headingDeg")
        heading = None if heading is None else navigation.number(heading)
        if "obstacles" in data:
            trip.set_obstacles(found, data.get("obstacles") or [], heading)
        if "minWidthCm" in data or "chairWidthCm" in data:
            width = data.get("minWidthCm", data.get("chairWidthCm"))
            trip.set_width(found, width, heading)
        if data.get("mode") in ("simulated", "live"):
            trip.mode = data["mode"]
        return trip.state()

    return checked(run)
