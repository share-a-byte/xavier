// The wheelchair navigator: a Matterport scan with its labelled scan-point graph drawn over it.
// The server (app/navigation.py) owns the graph, the labels, the routes and the trip. This page reads the scan
// points out of the Matterport SDK, lets the team label hops, shows the trip, and can stand in for the chair:
// the simulated chair reports its position through the same endpoints the real one uses.

const modelFrom = text => (text.match(/[?&]m=([A-Za-z0-9]{11})\b/) || text.trim().match(/^([A-Za-z0-9]{11})$/) || [])[1];
const MODEL = modelFrom(location.search) || '2K7G7cdukZV';

const NAMES = { corridor: 'Corridor', doorway: 'Doorway', ramp: 'Ramp', elevator: 'Elevator',
  stairs: 'Stairs or step', blocked: 'No way through', unknown: 'Not checked' };
const KEYS = { c: 'corridor', d: 'doorway', r: 'ramp', e: 'elevator', s: 'stairs', x: 'blocked', u: 'unknown' };
const HINT = 'Choose From and To, then Plan route. A* finds a step-free path on the Matterport scan graph.';
const SPEED_MPS = 0.8, TURN_RADPS = 2.2, PIVOT_RAD = 0.35;
// The wheelchair model is 0.746 m wide, faces +X, and its wheels touch y = 0.
const CHAIR_SCALE = 0.70 / 0.746, CHAIR_CENTER_M = 0.0825 * CHAIR_SCALE;

const $ = id => document.getElementById(id);
const frame = $('scan'), svg = $('overlay');
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const rad = d => d * Math.PI / 180, deg = r => r * 180 / Math.PI;
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const esc = s => String(s).replace(/[&<>"']/g, c => `&#${c.charCodeAt(0)};`);
const edgeId = (a, b) => [a, b].sort().join('~');
const plural = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;

const state = {
  sdk: null, graph: null, nodes: new Map(), edges: new Map(),
  pose: null, floor: null,            // the viewer's camera and the floor it shows (null: all of them)
  picked: null,                       // { edge } or { node }
  trip: null, tripText: '', epoch: 0, // the server's trip; epoch discards answers that a later action overtook
  chair: null, obstacle: null, reviewing: false, model3d: null,
  obstacles: [],                      // session collision geometry (circles on the floor plane)
  localPreview: null,                 // stitched local A* path for the current global route
  localGrid: null,                    // occupied cells from the local planner (drawn on the overlay)
};
const sim = { on: false, held: false, timer: 0, last: 0, sent: 0, route: null, index: 0, x: 0, z: 0, heading: 0,
  wait: 0, path: null, pathDrawn: null };

async function api(path = '', method = 'GET', body) {
  const options = body === undefined ? { method } : { method, headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
  const response = await fetch(`/api/nav/${MODEL}${path}`, options);
  const data = await response.json().catch(() => null);
  if (data === null) throw Object.assign(new Error('The navigator did not answer.'), { missing: true });
  if (!response.ok) throw new Error(typeof data.detail === 'string' ? data.detail : `The server answered ${response.status}.`);
  return data;
}

function say(text) {
  $('boot').textContent = text;
  $('boot').hidden = !text;
}

let hintTimer = 0;
function flash(text) {
  $('hint').textContent = text;
  clearTimeout(hintTimer);
  hintTimer = setTimeout(() => { $('hint').textContent = HINT; }, 6000);
}

// ---- The scan

async function connect(loaded) {
  await loaded;
  const win = frame.contentWindow;
  for (let i = 0; i < 100 && !win.MP_SDK; i++) await sleep(100);
  if (!win.MP_SDK) throw new Error('The 3D viewer did not start.');
  const sdk = await win.MP_SDK.connect(win);
  const phase = await Promise.race([
    sdk.App.state.waitUntil(s => s.phase === sdk.App.Phase.PLAYING || s.phase === sdk.App.Phase.ERROR).then(s => s.phase),
    sleep(45000).then(() => 'timeout'),
  ]);
  if (phase !== sdk.App.Phase.PLAYING) throw new Error('Matterport could not load this scan. Is the model public, and is this SDK key allowed to open it?');
  return sdk;
}

function collection(observable) {
  return new Promise(resolve => {
    let subscription, done = false;
    const finish = items => {
      if (done) return;
      done = true;
      resolve(items);
      setTimeout(() => subscription?.cancel(), 0);
    };
    subscription = observable.subscribe({ onCollectionUpdated(all) { const items = Object.values(all); if (items.length) finish(items); } });
    setTimeout(() => finish([]), 8000);
  });
}

// Step 1 of the pipeline: every scan point with its position, floor height and lines of sight.
async function readScanPoints() {
  const { sdk } = state;
  const [sweeps, floors] = await Promise.all([collection(sdk.Sweep.data), collection(sdk.Floor.data)]);
  // 360° views that are not aligned to the model have no position worth trusting.
  const placed = sweeps.filter(s => s.enabled !== false && s.alignmentType === sdk.Sweep.Alignment.ALIGNED);
  if (placed.length < 2) throw new Error('This scan has no scan points to build a map from.');
  return api('/sweeps', 'PUT', {
    floors: floors.map(f => ({ sequence: f.sequence, name: f.name })),
    sweeps: placed.map(s => ({
      id: s.id, x: s.position.x, y: s.position.y, z: s.position.z, puckY: s.puckPosition?.y ?? null,
      floor: Number.isInteger(s.floorInfo?.sequence) ? s.floorInfo.sequence : null, neighbors: s.neighbors || [],
    })),
  });
}

function watch(sdk) {
  sdk.Camera.pose.subscribe(pose => { state.pose = pose; schedule(); });
  sdk.Floor.current.subscribe(floor => {
    // While the viewer moves between floors the sequence is undefined; keep showing the one it left.
    if (floor.sequence === -1) state.floor = null;
    else if (Number.isInteger(floor.sequence)) state.floor = floor.sequence;
    paintFloors();
    schedule();
  });
  sdk.Mode.current.subscribe(mode => {
    for (const button of $('views').children) {
      button.setAttribute('aria-pressed', String(mode === (button.dataset.view === 'inside' ? sdk.Mode.Mode.INSIDE : sdk.Mode.Mode.FLOORPLAN)));
    }
  });
  addEventListener('resize', schedule);
}

async function addChairModel(sdk) {
  try {
    const [object] = await sdk.Scene.createObjects(1);
    const node = object.addNode();
    node.addComponent('mp.gltfLoader', { url: `${location.origin}/manual-wheelchair.glb`, localScale: { x: CHAIR_SCALE, y: CHAIR_SCALE, z: CHAIR_SCALE } });
    node.addComponent('mp.ambientLight', { intensity: 1.7 });
    node.addComponent('mp.directionalLight', { intensity: 2.0, position: { x: 2, y: 4, z: 3 } });
    node.position.set(0, -100, 0);
    node.start();
    state.model3d = node;
  } catch { /* the flat marker on the map is enough */ }
}

async function look(fromId, towardId) {
  const { sdk, nodes } = state, toward = nodes.get(towardId);
  try {
    await sdk.Sweep.moveTo(fromId, { transition: sdk.Sweep.Transition.FLY, transitionTime: 700 });
    if (toward) await sdk.Camera.lookAt({ x: toward.x, y: toward.y + 0.6, z: toward.z });
  } catch (error) {
    flash(`The viewer could not move there: ${String(error.message || error).slice(0, 120)}`);
  }
}

// ---- The graph

function useGraph(graph) {
  state.graph = graph;
  state.nodes = new Map(graph.nodes.map(n => [n.id, n]));
  state.edges = new Map(graph.edges.map(e => [e.id, e]));
  fillSelects();
  paintFloors();
  paintMap();
  paintPicked();
  schedule();
}

const floorName = sequence => state.graph.floors.find(f => f.sequence === sequence)?.name ?? (sequence === null ? 'No floor' : `Floor ${sequence + 1}`);
const placeAt = id => state.graph.destinations.find(p => p.sweep === id);
const nodeName = id => placeAt(id)?.name ?? `Scan ${state.nodes.get(id)?.scan ?? '?'}`;
const floorsWithScans = () => [...new Set(state.graph.nodes.map(n => n.floor))].sort((a, b) => a - b);

function fillSelects() {
  const { graph } = state;
  // Named places first, grouped by floor. Remaining unlabeled scans stay as "Scan N".
  const labeled = new Set(graph.destinations.map(p => p.sweep));
  const byFloor = floorsWithScans().map(floor => {
    const onFloor = graph.nodes.filter(n => n.floor === floor);
    const places = graph.destinations
      .filter(p => state.nodes.get(p.sweep)?.floor === floor)
      .map(p => `<option value="${esc(p.sweep)}">${esc(p.name)}</option>`).join('');
    const scans = onFloor.filter(n => !labeled.has(n.id))
      .map(n => `<option value="${esc(n.id)}">Scan ${n.scan}</option>`).join('');
    if (!places && !scans) return '';
    return `<optgroup label="${esc(floorName(floor))}">${places}${scans}</optgroup>`;
  }).join('');
  for (const select of [$('from'), $('to')]) {
    const keep = select.value;
    select.innerHTML = `<option value="">Choose a place or scan point</option>${byFloor}`;
    if (state.nodes.has(keep)) select.value = keep;
  }
}

function paintFloors() {
  if (!state.graph) return;
  $('floors').innerHTML = floorsWithScans().map(f => `<button data-floor="${f}" aria-pressed="${state.floor === f}">${esc(floorName(f))}</button>`).join('')
    + `<button data-floor="all" aria-pressed="${state.floor === null}">All floors</button>`;
}

function paintMap() {
  // Place names stay in the From/To dropdowns; no separate builder list.
}

function pick(picked) {
  state.picked = picked;
  paintPicked();
  schedule();
}

function paintPicked() {
  const box = $('selected');
  if (!box) return;
  const { picked, nodes, edges, graph } = state;
  const edge = picked?.edge && edges.get(picked.edge), node = picked?.node && nodes.get(picked.node);
  if (edge) {
    const a = nodes.get(edge.a), b = nodes.get(edge.b);
    const shape = edge.riseM > 0.08 ? `${edge.runM} m along and ${edge.riseM} m up or down` : `${edge.lengthM} m, level`;
    const origin = edge.source === 'label' ? 'Labelled by the team.'
      : edge.source === 'added' ? 'Added by the team; Matterport has no hop here.'
      : edge.type === 'unknown' ? 'Nobody has checked this hop yet.' : `${esc(edge.reason)}.`;
    box.innerHTML = `<h2>Hop: scan ${a.scan} to scan ${b.scan}</h2>
      <p class="fine">${shape}. ${origin}</p>
      <div class="types">${graph.types.map(type => `<button data-label="${type}" aria-pressed="${edge.type === type}">${NAMES[type]}
        <kbd>${Object.keys(KEYS).find(k => KEYS[k] === type).toUpperCase()}</kbd></button>`).join('')}</div>
      ${edge.accessible ? `<form class="inline" data-form="width"><label for="hop-width">Clear width</label>
        <input id="hop-width" type="number" min="20" max="1000" value="${edge.widthCm ?? ''}"> cm <button>Save</button></form>` : ''}
      <div class="buttons"><button data-look="${esc(edge.a)}" data-toward="${esc(edge.b)}">Look from scan ${a.scan}</button>
        <button data-look="${esc(edge.b)}" data-toward="${esc(edge.a)}">Look from scan ${b.scan}</button></div>
      <p class="fine">Mark a hop as drivable only after checking its width, floor and turning room on site.</p>`;
  } else if (node) {
    const place = placeAt(node.id);
    const hops = graph.edges.filter(e => e.a === node.id || e.b === node.id)
      .map(e => ({ e, other: nodes.get(e.a === node.id ? e.b : e.a) })).sort((p, q) => p.other.scan - q.other.scan);
    box.innerHTML = `<h2>${place ? `${esc(place.name)} · ` : ''}Scan ${node.scan}</h2>
      <p class="fine">${esc(floorName(node.floor))}, floor height ${node.y.toFixed(2)} m</p>
      <div class="buttons"><button data-use="from">Start here</button><button data-use="to">Go here</button><button data-stand>Stand here</button></div>
      <form class="inline" data-form="place"><label for="place-name">Name</label>
        <input id="place-name" maxlength="60" placeholder="Room 105" value="${esc(place?.name ?? '')}"><button>Save place</button></form>
      <h3>Hops from here</h3>
      <ul class="hops">${hops.map(({ e, other }) => `<li><span>Scan ${other.scan}${other.floor !== node.floor ? ` (${esc(floorName(other.floor))})` : ''}
        · ${e.lengthM} m · ${NAMES[e.type]}</span><button data-pick-edge="${esc(e.id)}">Select</button></li>`).join('')}</ul>
      <form class="inline" data-form="link"><label for="link-scan">Link to scan</label>
        <input id="link-scan" type="number" min="1" max="${graph.nodes.length}" required> as
        <select id="link-type" aria-label="Kind of link">${['elevator', 'corridor', 'doorway', 'ramp', 'stairs'].map(t => `<option value="${t}">${NAMES[t]}</option>`).join('')}</select>
        <button>Add</button></form>`;
  } else {
    box.innerHTML = '<h2>Map labels</h2><p class="fine">Click a hop or a scan point on the map.</p>';
  }
}

async function label(edge, type, widthCm = edge.widthCm) {
  try {
    useGraph(await api('/label', 'PUT', { from: edge.a, to: edge.b, type, widthCm: widthCm || null }));
  } catch (error) {
    return flash(error.message);
  }
  // A route planned before this label may no longer hold. A trip that a chair is driving is left alone.
  if (state.trip && !sim.on && ['planned', 'no-route', 'arrived', 'stopped'].includes(state.trip.status)) await plan(state.trip);
}

async function savePlaces(places) {
  try {
    useGraph(await api('/places', 'PUT', { places }));
  } catch (error) {
    flash(error.message);
  }
}

// ---- The trip

// Typical chair widths; Xavier matches the demo model until his measured width is entered.
const CHAIR_PRESETS = { manual: 64, power: 70, xavier: 70 };
const chairWidthCm = () => CHAIR_PRESETS[$('chair-preset')?.value] ?? 70;

async function plan(from) {
  // A* uses Matterport level hops automatically; stairs/blocked stay out. Elevators stay on for demos.
  const request = from
    ? { start: from.start, goal: from.goal, elevators: from.options.elevators !== false,
        minWidthCm: from.options.min_width_cm ?? chairWidthCm(), obstacles: state.obstacles, mode: 'simulated' }
    : { start: $('from').value, goal: $('to').value, elevators: true,
        minWidthCm: chairWidthCm(), obstacles: state.obstacles, mode: 'simulated' };
  if (!request.start || !request.goal) return routeText('Choose where the trip starts and where it ends.', 'problem');
  await stopSim();
  state.epoch++;
  try {
    useTrip(await api('/trip', 'POST', request));
    await rebuildLocalPreview();
  } catch (error) {
    routeText(error.message, 'problem');
  }
}

async function planLocal(start, goal, obstacles = state.obstacles) {
  return api('/local', 'POST', {
    start: { x: start.x, z: start.z }, goal: { x: goal.x, z: goal.z },
    chairWidthCm: chairWidthCm(), obstacles,
  });
}

function mergeGrid(into, grid) {
  if (!grid) return into;
  const key = p => `${p.x},${p.z}`;
  const merge = (prev, next, cap) => {
    const seen = new Set((prev || []).map(key));
    const out = [...(prev || [])];
    for (const p of next || []) {
      if (seen.has(key(p)) || out.length >= cap) continue;
      seen.add(key(p));
      out.push(p);
    }
    return out;
  };
  return {
    cellM: grid.cellM ?? into?.cellM ?? 0.12,
    chairRadiusM: grid.chairRadiusM ?? into?.chairRadiusM,
    bounds: grid.bounds || into?.bounds,
    free: merge(into?.free, grid.free, 500),
    occupied: merge(into?.occupied, grid.occupied, 400),
  };
}

/** Stitch local A* between each global hop so the map shows the path the chair will actually drive. */
async function rebuildLocalPreview() {
  const route = state.trip?.route;
  if (!route?.found) {
    state.localPreview = null;
    state.localGrid = null;
    schedule();
    return;
  }
  const epoch = state.epoch;
  const { waypoints, legs } = route;
  const points = [{ x: waypoints[0].x, z: waypoints[0].z, y: waypoints[0].y }];
  let length = 0, grid = null;
  for (let i = 0; i < legs.length; i++) {
    const a = waypoints[i], b = waypoints[i + 1];
    if (legs[i].type === 'elevator') {
      points.push({ x: b.x, z: b.z, y: b.y });
      continue;
    }
    const local = await planLocal(a, b);
    if (epoch !== state.epoch) return;
    grid = mergeGrid(grid, local.grid);
    if (!local.found) {
      state.localPreview = { found: false, reason: local.reason };
      state.localGrid = grid;
      schedule();
      paintTrip();
      return;
    }
    length += local.lengthM || 0;
    for (const p of local.points.slice(1)) points.push({ x: p.x, z: p.z, y: a.y });
  }
  if (epoch !== state.epoch) return;
  state.localPreview = { found: true, points, lengthM: Math.round(length * 100) / 100 };
  state.localGrid = grid;
  schedule();
  paintTrip();
}

async function replanSession(extra = {}) {
  if (!state.trip) return;
  const epoch = ++state.epoch;
  try {
    const trip = await api('/trip/replan', 'POST', {
      minWidthCm: chairWidthCm(),
      obstacles: state.obstacles,
      headingDeg: state.chair?.headingDeg,
      mode: 'simulated',
      ...extra,
    });
    if (epoch !== state.epoch) return;
    if (!trip.route?.found && sim.on) {
      // Failed replan must never resume movement through a stale route.
      sim.on = false;
      clearInterval(sim.timer);
      Object.assign(sim, { held: false, path: null, pathDrawn: null });
    } else if (trip.route?.found && sim.on) {
      Object.assign(sim, {
        route: trip.route,
        index: Math.min(Math.max(sim.index, 1), Math.max(trip.route.waypoints.length - 1, 1)),
        held: false, path: null, pathDrawn: null,
      });
      await armLeg();
    }
    useTrip(trip);
    if (trip.route?.found && !sim.on) await rebuildLocalPreview();
  } catch (error) {
    flash(error.message);
  }
}

function startPose(trip) {
  const first = trip.route.found && trip.route.waypoints[0];
  return first ? { x: first.x, y: first.y, z: first.z, floor: first.floor, headingDeg: trip.origin.headingDeg, detour: [], waypoint: null } : null;
}

function useTrip(trip) {
  state.trip = trip;
  if (Array.isArray(trip.obstacles)) state.obstacles = trip.obstacles;
  const badge = $('mode-badge');
  if (badge) {
    const live = trip.mode === 'live';
    badge.textContent = live ? 'Live chair' : 'Simulated';
    badge.className = `badge ${live ? 'live' : 'sim'}`;
  }
  if ($('st-mode')) $('st-mode').textContent = trip.mode === 'live' ? 'Live wheelchair' : 'Simulated (demo)';
  state.tripText = JSON.stringify(trip);
  if (!$('from').value) $('from').value = trip.start;
  if (!$('to').value) $('to').value = trip.goal;
  if (!sim.on) {
    state.chair = trip.pose || startPose(trip);
    if (!trip.pose) state.obstacle = null;
  }
  paintTrip();
  schedule();
}

function routeText(text, tone = '') {
  $('route-text').textContent = text;
  $('route-text').className = tone;
}

function paintTrip() {
  const { trip } = state;
  if (!trip) return;
  const { route, walking } = trip;
  const compare = $('compare')?.checked;
  $('legend-matterport').hidden = !compare;
  if (!route.found) {
    routeText(route.reason || 'No wheelchair route.', 'problem');
  } else {
    const elevators = route.usesElevator ? ' · uses elevator' : '';
    let text = `Ours: ${chairWidthCm()} cm chair · ${route.lengthM} m · ${plural(route.legs.length, 'hop')}${elevators}.`;
    if (compare && walking?.found) {
      const stairs = walking.legs.some(l => l.type === 'stairs');
      text += ` Matterport/on-foot: ${walking.lengthM} m${stairs ? ' (uses stairs)' : ''}.`;
    }
    routeText(text);
  }
  $('drive').disabled = !route.found || sim.on;
  $('stop').disabled = !sim.on;
  // Place obstacles as soon as a route exists; detour/block need an active drive.
  $('add-obstacle').disabled = !route.found;
  $('go-around').disabled = !sim.on || sim.held;
  $('block').disabled = !sim.on || sim.held;
  $('clear-obstacles').disabled = !state.obstacles.length;
  if ($('sim-text')) {
    const free = state.localGrid?.free?.length || 0;
    const blocked = state.localGrid?.occupied?.length || 0;
    if (free || blocked) {
      $('sim-text').textContent = state.obstacles.length
        ? `Local grid on map · ${free} free / ${blocked} blocked · ${state.obstacles.length} obstacle(s) · chair ${chairWidthCm()} cm`
        : `Local grid on map · ${free} free cells (all clear) · place an obstacle to block cells · chair ${chairWidthCm()} cm`;
    } else {
      $('sim-text').textContent = 'Plan a route to show the local occupancy grid and drive path on the scan.';
    }
  }

  const pose = state.chair, waypoints = route.waypoints || [];
  const next = pose && Number.isInteger(pose.waypoint) ? waypoints[pose.waypoint] : null;
  const simulated = pose?.source === 'simulator' ? ' (simulated)' : '';
  $('st-chair').textContent = ({ planned: 'Ready at the start', following: 'Driving', detour: 'Going around an obstacle',
    blocked: 'Stopped: the way ahead is blocked', replanned: 'Has a new route', arrived: 'Arrived', stopped: 'Stopped',
    'no-route': 'Stopped: no wheelchair route' }[trip.status] ?? trip.status) + simulated;
  $('st-next').textContent = next ? `${nodeName(next.id)} · ${Math.hypot(next.x - pose.x, next.z - pose.z).toFixed(1)} m away` : '–';
  const preview = state.localPreview;
  if (sim.on && sim.pathDrawn?.length) {
    $('st-local').textContent = `Driving local A* · ${sim.path?.length ?? 0} pts left on this leg`;
  } else if (preview?.found) {
    const bent = state.obstacles.length ? ' · bent around obstacles' : ' · straight (no obstacles yet)';
    $('st-local').textContent = `Local A* · ${preview.lengthM} m${bent}`;
  } else if (preview && !preview.found) {
    $('st-local').textContent = preview.reason || 'Local plan failed';
  } else {
    $('st-local').textContent = { planned: 'Plan a route to build the local path', following: 'Driving',
      detour: 'Local detour', blocked: 'Found no way around', replanned: 'Taking the new route',
      arrived: 'Done', stopped: 'Idle', 'no-route': 'Holding position' }[trip.status] ?? '–';
  }
  $('st-global').textContent = route.found
    ? `Global A*: ${route.lengthM} m, ${plural(waypoints.length, 'waypoint')}${trip.blocked.length ? `, avoiding ${plural(trip.blocked.length, 'blocked hop')}` : ''}`
    : route.reason;
  $('log').innerHTML = [...trip.log].reverse().map(entry => `<li>${esc(entry.text)}</li>`).join('');
}

async function poll() {
  const epoch = state.epoch;
  try {
    const trip = await api('/trip');
    if (epoch === state.epoch && trip.status !== 'idle' && JSON.stringify(trip) !== state.tripText) useTrip(trip);
  } catch { /* the next round will try again */ }
  setTimeout(poll, 400);
}

function reviewNext() {
  const route = state.trip?.route;
  const leg = route?.found && route.legs.find(l => !l.verified);
  state.reviewing = Boolean(leg);
  if (!leg) return;
  pick({ edge: edgeId(leg.from, leg.to) });
  look(leg.from, leg.to);
}

function nextUnknown() {
  const scan = id => state.nodes.get(id).scan;
  const unknown = state.graph.edges.filter(e => e.type === 'unknown')
    .filter(e => state.floor === null || state.nodes.get(e.a).floor === state.floor || state.nodes.get(e.b).floor === state.floor)
    .sort((p, q) => Math.min(scan(p.a), scan(p.b)) - Math.min(scan(q.a), scan(q.b)) || p.id.localeCompare(q.id));
  if (!unknown.length) return flash('Every hop on this floor has a label.');
  const at = unknown.findIndex(e => e.id === state.picked?.edge);
  pick({ edge: unknown[(at + 1) % unknown.length].id });
}

// ---- The simulated chair. Global A* picks waypoints; local A* owns the path between them.

async function startSim() {
  const { trip } = state;
  if (!trip?.route.found || sim.on) return;
  const waypoints = trip.route.waypoints;
  Object.assign(sim, {
    on: true, held: false, route: trip.route, index: Math.min(1, waypoints.length - 1),
    x: waypoints[0].x, z: waypoints[0].z, heading: rad(trip.origin.headingDeg),
    wait: 0, path: null, pathDrawn: null, last: performance.now(), sent: 0,
  });
  follow(0);
  const ok = await armLeg();
  if (!ok) {
    sim.on = false;
    paintTrip();
    return;
  }
  clearInterval(sim.timer);
  sim.timer = setInterval(tick, 40);
  paintTrip();
}

function stopSim(status = 'stopped') {
  if (!sim.on) return;
  sim.on = false;
  clearInterval(sim.timer);
  Object.assign(sim, { path: null, held: false, wait: 0 });
  const sent = report(status);
  paintTrip();
  return sent;
}

/** Local A* from the chair to the next global waypoint (skips elevators). */
async function armLeg() {
  const { waypoints, legs } = sim.route;
  if (sim.index >= waypoints.length) return false;
  const leg = legs[sim.index - 1];
  if (leg?.type === 'elevator') {
    sim.path = null;
    sim.pathDrawn = null;
    return true;
  }
  const goal = waypoints[sim.index];
  const epoch = state.epoch;
  try {
    const local = await planLocal({ x: sim.x, z: sim.z }, goal);
    if (epoch !== state.epoch || !sim.on) return false;
    if (!local.found) {
      flash(local.reason || 'Local planner could not reach the next waypoint.');
      $('st-local').textContent = local.reason || 'No local path';
      return false;
    }
    sim.path = local.points.slice(1).map(p => ({ x: p.x, z: p.z }));
    sim.pathDrawn = local.points.map(p => [p.x, p.z]);
    if (local.grid) state.localGrid = mergeGrid(state.localGrid, local.grid);
    $('st-local').textContent = `Local A* · ${local.lengthM} m to next waypoint`;
    return true;
  } catch (error) {
    flash(error.message);
    return false;
  }
}

function tick() {
  const now = performance.now(), dt = Math.min((now - sim.last) / 1000, 1);
  sim.last = now;
  if (!sim.held) step(dt);
  if (!sim.on) return;
  const status = sim.held ? 'blocked' : sim.wait > 0 ? 'stopped' : 'following';
  if (now - sim.sent > 250) {
    sim.sent = now;
    report(status);
  } else {
    state.chair = chairNow();
  }
  schedule();
}

function step(dt) {
  const { waypoints, legs } = sim.route;
  if (sim.wait > 0) {
    sim.wait -= dt;
    if (sim.wait <= 0) {
      Object.assign(sim, { x: waypoints[sim.index].x, z: waypoints[sim.index].z });
      finishWaypoint();
    }
    return;
  }
  // Nobody drives an elevator leg: the chair waits to be carried to the next floor.
  if ((!sim.path || !sim.path.length) && sim.index > 0 && legs[sim.index - 1].type === 'elevator') {
    sim.wait = 2.5;
    return;
  }
  const target = sim.path?.length ? sim.path[0] : waypoints[sim.index];
  if (!target) return;
  const dx = target.x - sim.x, dz = target.z - sim.z, far = Math.hypot(dx, dz);
  if (far < 0.03) return reachPoint();
  const off = wrap(Math.atan2(-dz, dx) - sim.heading);
  sim.heading = wrap(sim.heading + Math.sign(off) * Math.min(Math.abs(off), TURN_RADPS * dt));
  if (Math.abs(off) <= PIVOT_RAD) {
    const go = Math.min(far, SPEED_MPS * dt);
    sim.x += dx / far * go;
    sim.z += dz / far * go;
  }
}

function reachPoint() {
  if (sim.path?.length) {
    sim.path.shift();
    if (sim.path.length) return;
  }
  finishWaypoint();
}

function finishWaypoint() {
  follow(sim.index);
  if (++sim.index >= sim.route.waypoints.length) {
    sim.index = sim.route.waypoints.length - 1;
    stopSim('arrived');
    return;
  }
  sim.held = true;
  armLeg().then(ok => {
    if (!sim.on) return;
    sim.held = false;
    if (!ok) stopSim('no-route');
    else paintTrip();
  });
}

function chairNow() {
  const waypoints = sim.route.waypoints, under = waypoints[Math.max(sim.index - 1, 0)];
  return { x: sim.x, y: under.y, z: sim.z, floor: under.floor, headingDeg: deg(sim.heading), waypoint: sim.index,
    detour: sim.pathDrawn || [], source: 'simulator' };
}

function report(status) {
  state.chair = chairNow();
  const { x, z, headingDeg, waypoint, detour } = state.chair;
  return api('/trip/pose', 'POST', { frame: 'map', x, z, headingDeg, waypoint, detour, status, source: 'simulator' }).catch(() => {});
}

// Drop an obstacle on the current leg and ask the local planner for a detour that rejoins the next waypoint.
async function goAround() {
  if (!sim.on || sim.held || sim.wait > 0) return;
  const target = sim.route.waypoints[sim.index];
  const dx = target.x - sim.x, dz = target.z - sim.z, far = Math.hypot(dx, dz);
  if (far < 1.2) return flash('Too close to the waypoint for a detour. Try on the next leg.');
  const ux = dx / far, uz = dz / far;
  const obstacle = { id: `sim-${Date.now()}`, x: sim.x + ux * Math.min(far * 0.45, 2.2),
    z: sim.z + uz * Math.min(far * 0.45, 2.2), radiusM: 0.45 };
  state.obstacle = obstacle;
  state.obstacles = [...state.obstacles.filter(o => o.id !== obstacle.id), obstacle];
  const epoch = ++state.epoch;
  try {
    const local = await planLocal({ x: sim.x, z: sim.z }, target);
    if (epoch !== state.epoch) return;
    if (!local.found) {
      $('st-local').textContent = local.reason || 'No local detour';
      flash(local.reason || 'No local detour — try “hallway blocked” to replan globally.');
      paintTrip();
      return;
    }
    sim.path = local.points.slice(1).map(p => ({ x: p.x, z: p.z }));
    sim.pathDrawn = local.points.map(p => [p.x, p.z]);
    $('st-local').textContent = `Local detour · ${local.lengthM} m · rejoins waypoint`;
    report('detour');
    paintTrip();
    schedule();
  } catch (error) {
    flash(error.message);
  }
}

/** Place an obstacle on the first open leg (or ahead of the chair while driving). */
async function placeObstacleAhead() {
  const route = state.trip?.route;
  if (!route?.found) return;
  let x, z;
  if (sim.on) {
    const ux = Math.cos(sim.heading), uz = -Math.sin(sim.heading);
    x = sim.x + ux * 1.6;
    z = sim.z + uz * 1.6;
  } else {
    const { waypoints, legs } = route;
    let i = legs.findIndex(l => l.type !== 'elevator');
    if (i < 0) i = 0;
    const a = waypoints[i], b = waypoints[i + 1];
    x = (a.x + b.x) / 2;
    z = (a.z + b.z) / 2;
  }
  const obstacle = { id: `place-${Date.now()}`, x, z, radiusM: 0.65 };
  state.obstacles = [...state.obstacles, obstacle];
  state.obstacle = obstacle;
  if (sim.on) {
    sim.held = true;
    const ok = await armLeg();
    sim.held = false;
    if (!ok) flash('Local planner cannot pass that obstacle — try hallway blocked for a global replan.');
    else flash('Local path replanned around the obstacle.');
    paintTrip();
  } else {
    const before = state.localPreview?.lengthM;
    await rebuildLocalPreview();
    const after = state.localPreview?.lengthM;
    if (state.localPreview?.found && after != null && before != null && after > before + 0.05) {
      flash(`Local path bent around the obstacle (${before} m → ${after} m). Dashed = hops, blue = drive.`);
    } else if (state.localPreview?.found) {
      flash('Obstacle added — blue local path updated. Dashed grey is still the hop line.');
    } else {
      flash(state.localPreview?.reason || 'Local planner could not route around that obstacle.');
    }
  }
  paintTrip();
  schedule();
}

async function clearObstacles() {
  state.obstacles = [];
  state.obstacle = null;
  if (sim.on) {
    sim.held = true;
    await armLeg();
    sim.held = false;
    paintTrip();
  } else if (state.trip?.route?.found) {
    await rebuildLocalPreview();
  } else {
    schedule();
  }
  paintTrip();
}

// The local planner gives up on the leg ahead, so the global planner closes it and finds another way.
async function blockAhead() {
  if (!sim.on || sim.held || sim.wait > 0 || sim.index < 1) return;
  const waypoints = sim.route.waypoints, from = waypoints[sim.index - 1], to = waypoints[sim.index];
  const ux = Math.cos(sim.heading), uz = -Math.sin(sim.heading);
  Object.assign(sim, { held: true, path: null, pathDrawn: null });
  const obstacle = { id: `block-${Date.now()}`, x: sim.x + ux * 1.2, z: sim.z + uz * 1.2, radiusM: 0.55 };
  state.obstacle = obstacle;
  state.obstacles = [...state.obstacles, obstacle];
  state.epoch++;
  try {
    await report('blocked');
    await sleep(900);
    const trip = await api('/trip/blocked', 'POST', { from: from.id, to: to.id, headingDeg: deg(sim.heading) });
    state.epoch++;
    if (trip.route.found) {
      Object.assign(sim, { route: trip.route, index: Math.min(1, trip.route.waypoints.length - 1), held: false });
      useTrip(trip);
      const ok = await armLeg();
      if (!ok) stopSim('no-route');
      else await rebuildLocalPreview();
    } else {
      sim.on = false;
      clearInterval(sim.timer);
      Object.assign(sim, { held: false, path: null, pathDrawn: null });
      useTrip(trip);
      routeText(trip.route.reason || 'No wheelchair way around. The chair stays stopped.', 'problem');
    }
  } catch (error) {
    sim.held = false;
    flash(error.message);
  }
}

// In the walk-through the camera stands one waypoint behind the chair and watches it go.
function follow(reached) {
  const { sdk, pose } = state, waypoints = sim.route.waypoints;
  if (!pose || pose.mode !== sdk.Mode.Mode.INSIDE) return;
  // At the start, and when the chair comes out of an elevator, there is nothing behind it on its floor: watch from ahead.
  const target = waypoints[reached];
  const stand = [waypoints[reached - 1], waypoints[reached + 1]].find(w => w && w.floor === target.floor);
  if (!stand) return;
  sdk.Sweep.moveTo(stand.id, { transition: sdk.Sweep.Transition.FLY, transitionTime: 700 })
    .then(() => sdk.Camera.lookAt({ x: target.x, y: target.y + 0.5, z: target.z })).catch(() => {});
}

// ---- Drawing

let queued = false;
function schedule() {
  if (queued) return;
  queued = true;
  requestAnimationFrame(() => { queued = false; draw(); });
}

function draw() {
  const { sdk, pose, graph, nodes, edges, trip, chair, picked } = state;
  if (!sdk || !pose || !graph) return;
  placeModel();
  const size = { w: frame.clientWidth, h: frame.clientHeight };
  svg.setAttribute('viewBox', `0 0 ${size.w} ${size.h}`);
  if (pose.mode === sdk.Mode.Mode.TRANSITIONING) { svg.innerHTML = ''; return; }
  const inside = pose.mode === sdk.Mode.Mode.INSIDE;

  // A point on the floor to a point on the screen; null when it is behind the camera or far off screen.
  const at = p => {
    const s = sdk.Conversion.worldToScreen({ x: p.x, y: p.y + 0.03, z: p.z }, pose, size);
    return Number.isFinite(s.x) && Number.isFinite(s.y) && s.z > -1 && s.z < 1 && Math.abs(s.x) < 2e4 && Math.abs(s.y) < 2e4 ? s : null;
  };
  // In the walk-through a line can run behind the camera, so it is drawn in short pieces and the hidden ones are dropped.
  const line = points => {
    let d = '', pen = false;
    const emit = p => {
      const s = at(p);
      if (s) d += `${pen ? 'L' : 'M'}${s.x.toFixed(1)} ${s.y.toFixed(1)}`;
      pen = Boolean(s);
    };
    points.forEach((p, i) => {
      emit(p);
      const q = points[i + 1];
      if (!q || !inside) return;
      const pieces = Math.ceil(Math.hypot(q.x - p.x, q.y - p.y, q.z - p.z) / 0.4);
      for (let k = 1; k < pieces; k++) emit({ x: p.x + (q.x - p.x) * k / pieces, y: p.y + (q.y - p.y) * k / pieces, z: p.z + (q.z - p.z) * k / pieces });
    });
    return d;
  };
  const shown = n => state.floor === null || n.floor === null || n.floor === state.floor;
  const out = [], dots = new Set();
  // With a planned trip, hide the hop spaghetti — show the path, not the whole scan graph.
  const tripActive = Boolean(trip?.route?.found);
  const routeIds = new Set(tripActive ? trip.route.waypoints.map(w => w.id) : []);

  for (const edge of graph.edges) {
    const a = nodes.get(edge.a), b = nodes.get(edge.b), chosen = picked?.edge === edge.id;
    if (tripActive && !inside && !chosen) {
      // Keep only elevators / stairs / closed hops as landmarks; skip unknown corridors.
      if (!['elevator', 'stairs', 'blocked'].includes(edge.type) && !trip.blocked.includes(edge.id)) continue;
    }
    let points;
    if (inside) {
      if (edge.a !== pose.sweep && edge.b !== pose.sweep && !chosen) continue;
      points = [a, b];
    } else if (shown(a) && shown(b)) {
      points = [a, b];
    } else if ((shown(a) || shown(b)) && (edge.type !== 'blocked' || chosen)) {
      const [near, far] = shown(a) ? [a, b] : [b, a], reachM = Math.min(1, 1.4 / (edge.runM || 1.4));
      points = [near, { x: near.x + (far.x - near.x) * reachM, y: near.y, z: near.z + (far.z - near.z) * reachM }];
    } else continue;
    const d = line(points);
    if (!d) continue;
    if (!tripActive) {
      for (const n of [a, b]) if (shown(n)) dots.add(n);
    }
    const closed = trip?.blocked.includes(edge.id) ? ' closed' : '';
    out.push(`${chosen ? `<path class="picked" d="${d}"/>` : ''}<path class="hop ${edge.type}${closed}" d="${d}"/><path class="hit" data-edge="${esc(edge.id)}" d="${d}"/>`);
  }

  const floorY = chair?.y ?? [...nodes.values()].find(n => state.floor === null || n.floor === state.floor)?.y ?? 0;
  // Occupancy grid: free cells only when debugging obstacles; always show blocked cells.
  if (state.obstacles.length) {
    for (const cell of state.localGrid?.free || []) {
      const s = at({ x: cell.x, y: floorY, z: cell.z });
      if (s) out.push(`<rect class="grid-free" x="${(s.x - 2.5).toFixed(1)}" y="${(s.y - 2.5).toFixed(1)}" width="5" height="5"/>`);
    }
  }
  for (const cell of state.localGrid?.occupied || []) {
    const s = at({ x: cell.x, y: floorY, z: cell.z });
    if (s) out.push(`<rect class="grid-cell" x="${(s.x - 3).toFixed(1)}" y="${(s.y - 3).toFixed(1)}" width="6" height="6"/>`);
  }

  const pathOf = route => route?.found
    ? route.waypoints.slice(1).map((w, i) => [route.waypoints[i], w]).filter(([p, q]) => shown(p) || shown(q)).map(line).join('')
    : '';
  if ($('compare')?.checked) {
    const walk = pathOf(trip?.walking);
    if (walk) out.push(`<path class="route-walk" d="${walk}"/>`);
  }
  // One clear blue path (local when available). Skip the dashed global underlay — it reads as clutter.
  const localPts = (sim.on && sim.pathDrawn?.length > 1)
    ? sim.pathDrawn.map(([x, z]) => ({ x, y: chair?.y ?? trip?.route.waypoints[0]?.y ?? 0, z }))
    : state.localPreview?.found
      ? state.localPreview.points
      : null;
  const global = pathOf(trip?.route);
  const drive = localPts?.length > 1 ? line(localPts) : global;
  if (drive) out.push(`<path class="casing" d="${drive}"/><path class="route-chair" d="${drive}"/>`);
  if (chair?.detour?.length > 1 && shown(chair) && !(sim.on && sim.pathDrawn)) {
    out.push(`<path class="route-detour" d="${line(chair.detour.map(([x, z]) => ({ x, y: chair.y, z })))}"/>`);
  }

  if (tripActive) {
    for (const w of trip.route.waypoints) {
      const n = nodes.get(w.id);
      if (n && shown(n)) dots.add(n);
    }
  }
  if (picked?.node && nodes.has(picked.node)) dots.add(nodes.get(picked.node));
  for (const n of dots) {
    const s = at(n);
    if (!s) continue;
    const isEnd = tripActive && (n.id === trip.start || n.id === trip.goal);
    const onRoute = routeIds.has(n.id);
    // Trip view: only start/goal are large labeled pins; intermediate hops are quiet dots.
    const r = isEnd ? 11 : tripActive ? 5 : 9;
    const classes = `node${picked?.node === n.id ? ' picked-node' : ''}${isEnd ? ' standing' : ''}${tripActive && onRoute && !isEnd ? ' via' : ''}`;
    out.push(`<circle class="${classes}" data-node="${esc(n.id)}" cx="${s.x.toFixed(1)}" cy="${s.y.toFixed(1)}" r="${r}"/>`);
    if (isEnd) {
      const place = placeAt(n.id);
      const label = place?.name || (n.id === trip.start ? 'Start' : 'Destination');
      out.push(`<text class="place" x="${s.x.toFixed(1)}" y="${(s.y - 16).toFixed(1)}">${esc(label)}</text>`);
    } else if (!tripActive) {
      const place = placeAt(n.id);
      out.push(`<text x="${s.x.toFixed(1)}" y="${(s.y + 3.5).toFixed(1)}">${place ? '' : n.scan}</text>`);
      if (place) out.push(`<text class="place" x="${s.x.toFixed(1)}" y="${(s.y - 14).toFixed(1)}">${esc(place.name)}</text>`);
    }
  }

  for (const ob of state.obstacles) {
    const s = at({ x: ob.x ?? (ob.xmin + ob.xmax) / 2, y: floorY, z: ob.z ?? (ob.zmin + ob.zmax) / 2 });
    if (!s) continue;
    const r = Math.max(8, (ob.radiusM || 0.4) * 22);
    out.push(`<circle class="obstacle-dot" cx="${s.x.toFixed(1)}" cy="${s.y.toFixed(1)}" r="${r.toFixed(1)}"/>`);
  }
  // In the walk-through the 3D wheelchair stands in the scene; on the plan an arrow shows where it is and where it faces.
  if (chair && shown(chair) && !(inside && state.model3d)) {
    const s = at(chair), yaw = rad(chair.headingDeg ?? 0);
    const ahead = at({ x: chair.x + Math.cos(yaw) * 0.5, y: chair.y, z: chair.z - Math.sin(yaw) * 0.5 });
    if (s) {
      const turn = ahead ? deg(Math.atan2(ahead.y - s.y, ahead.x - s.x)) : 0;
      out.push(`<path class="chair" transform="translate(${s.x.toFixed(1)} ${s.y.toFixed(1)}) rotate(${turn.toFixed(1)})" d="M13 0 L-9 9 L-5 0 L-9 -9 Z"/>`);
    }
  }
  svg.innerHTML = out.join('');
}

function placeModel() {
  const node = state.model3d, chair = state.chair;
  if (!node) return;
  if (!chair) return void node.position.set(0, -100, 0);
  const yaw = rad(chair.headingDeg ?? 0);
  node.position.set(chair.x - CHAIR_CENTER_M * Math.cos(yaw), chair.y, chair.z + CHAIR_CENTER_M * Math.sin(yaw));
  node.quaternion.set(0, Math.sin(yaw / 2), 0, Math.cos(yaw / 2));
}

// ---- Controls

svg.addEventListener('pointerdown', event => {
  // The map is redrawn as the trip updates, so a click could lose its target between press and release.
  const hit = event.target.closest('[data-edge], [data-node]');
  if (hit) pick(hit.dataset.edge ? { edge: hit.dataset.edge } : { node: hit.dataset.node });
});

$('selected')?.addEventListener('click', event => {
  const button = event.target.closest('button'), { picked, edges, sdk } = state;
  if (!button) return;
  const data = button.dataset;
  if (data.label) label(edges.get(picked.edge), data.label);
  else if (data.look) look(data.look, data.toward);
  else if (data.use) { $(data.use).value = picked.node; flash(`${data.use === 'from' ? 'Trip starts' : 'Trip ends'} at ${nodeName(picked.node)}.`); }
  else if ('stand' in data) sdk.Sweep.moveTo(picked.node, { transition: sdk.Sweep.Transition.FLY, transitionTime: 700 }).catch(() => {});
  else if (data.pickEdge) pick({ edge: data.pickEdge });
});

$('selected')?.addEventListener('submit', event => {
  event.preventDefault();
  const kind = event.target.dataset.form, { picked, edges, nodes, graph } = state;
  if (kind === 'width') {
    const edge = edges.get(picked.edge);
    label(edge, edge.type, $('hop-width').valueAsNumber);
  } else if (kind === 'place') {
    const name = $('place-name').value.trim();
    const others = graph.destinations.filter(p => p.sweep !== picked.node);
    savePlaces(name ? [...others, { name, sweep: picked.node }] : others);
  } else if (kind === 'link') {
    const other = graph.nodes.find(n => n.scan === $('link-scan').valueAsNumber);
    if (!other || other.id === picked.node) return flash('Choose another scan point by its number on the map.');
    label({ a: picked.node, b: other.id }, $('link-type').value).then(() => pick({ edge: edgeId(picked.node, other.id) }));
  }
});

$('places')?.addEventListener('click', event => {
  const id = event.target.closest('button')?.dataset.remove;
  if (id) savePlaces(state.graph.destinations.filter(p => p.id !== id));
});

$('views').addEventListener('click', event => {
  const view = event.target.closest('button')?.dataset.view, { sdk } = state;
  if (view && sdk) sdk.Mode.moveTo(view === 'inside' ? sdk.Mode.Mode.INSIDE : sdk.Mode.Mode.FLOORPLAN).catch(() => {});
});

$('floors').addEventListener('click', event => {
  const floor = event.target.closest('button')?.dataset.floor, { sdk } = state;
  if (!floor || !sdk) return;
  (floor === 'all' ? sdk.Floor.showAll() : sdk.Floor.moveTo(Number(floor))).catch(() => {});
});

$('plan').addEventListener('click', () => plan());
$('drive').addEventListener('click', () => { startSim().catch(err => flash(err.message)); });
$('stop').addEventListener('click', () => stopSim());
$('go-around').addEventListener('click', goAround);
$('block').addEventListener('click', blockAhead);
$('add-obstacle').addEventListener('click', placeObstacleAhead);
$('clear-obstacles').addEventListener('click', clearObstacles);
$('chair-preset').addEventListener('change', async () => {
  if (state.trip) {
    await replanSession();
    if (!sim.on && state.trip?.route?.found) await rebuildLocalPreview();
  } else if (state.graph) schedule();
});
$('compare').addEventListener('change', () => {
  $('legend-matterport').hidden = !$('compare').checked;
  if (state.trip) paintTrip();
  schedule();
});
$('next-unknown')?.addEventListener('click', nextUnknown);
$('reread')?.addEventListener('click', async () => {
  try { useGraph(await readScanPoints()); flash('Scan points read again from Matterport.'); } catch (error) { flash(error.message); }
});

addEventListener('keydown', event => {
  if (event.ctrlKey || event.metaKey || event.altKey || event.target.closest?.('input, select, textarea')) return;
  const key = event.key.toLowerCase(), edge = state.picked?.edge && state.edges.get(state.picked.edge);
  if (edge && KEYS[key]) { event.preventDefault(); label(edge, KEYS[key]); }
  else if (key === 'n' && state.graph) { event.preventDefault(); nextUnknown(); }
});

// ---- Start

async function boot() {
  $('hint').textContent = HINT;
  let graph;
  try {
    graph = await api();
  } catch (error) {
    return say(error.missing ? 'This server does not have the navigator yet. Restart it with: .venv/bin/python -m app.server' : error.message);
  }
  const key = await fetch('/sdk-config').then(r => r.json()).then(config => config.key).catch(() => null);
  if (!key) return say('Set MATTERPORT_SDK_KEY to open the scan.');
  const loaded = new Promise(resolve => frame.addEventListener('load', resolve, { once: true }));
  frame.src = `/viewer/showcase.html?${new URLSearchParams({ m: MODEL, play: '1', qs: '1', applicationKey: key })}`;
  try {
    state.sdk = await connect(loaded);
    if (!graph.saved) {
      say('Reading the scan points from Matterport…');
      graph = await readScanPoints();
    }
  } catch (error) {
    return say(error.message);
  }
  useGraph(graph);
  watch(state.sdk);
  addChairModel(state.sdk);
  await state.sdk.Mode.moveTo(state.sdk.Mode.Mode.FLOORPLAN).catch(() => {});
  say('');
  $('plan').disabled = false;
  if ($('reread')) $('reread').disabled = false;
  if ($('next-unknown')) $('next-unknown').disabled = false;
  poll();
}

boot();
