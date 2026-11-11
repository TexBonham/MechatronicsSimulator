// Browser client over the local facade service (VIE-012, ADR 0007). Every value shown comes from a facade message
// (FAC-016); the client writes no files (VIE-017) and names no design (invariant 2).
import { FacadeClient, FacadeError } from "./protocol.js";
import { Replay, poseSeries } from "./replay.js";
import { Timeline } from "./timeline.js";
import { ProbePanel, sampleAt } from "./probes.js";

const WINDOW_POINTS = 512;
const WINDOW_PARTS = 8;

const element = (id) => document.getElementById(id);
const client = new FacadeClient();
const view = new Replay(element("scene"));
const timeline = new Timeline(element("timeline-bar"), element("timeline-list"), (t) => {
  stopPlaying();
  cursorTo(t).catch(failure);
});
const probes = new ProbePanel(element("probe-table"), element("probe-caption"));

const state = {
  handle: null,
  bodies: [],
  fixed: new Set(),
  windows: new Map(),
  cursor: null,
  playing: false,
  clock: 0,
  wall: 0,
  busy: false,
};

function status(text) {
  element("status").textContent = text;
}

function failure(error) {
  if (error instanceof FacadeError) {
    let requirement = "";
    if (error.requirementId) {
      requirement = ` [${error.requirementId}]`;
    }
    status(`error ${error.code}${requirement}: ${error.message}`);
  } else {
    status(`error: ${error.message}`);
  }
}

function socketUrl() {
  const page = new URL(window.location.href);
  const url = new URL("/ws", page);
  url.searchParams.set("token", page.searchParams.get("token") || "");
  if (page.protocol === "https:") {
    url.protocol = "wss:";
  } else {
    url.protocol = "ws:";
  }
  return url.href;
}

// ===== designs and runs ================================================

async function listDesigns() {
  const result = await client.request({ kind: "list_designs", payload: {} });
  const list = element("designs");
  list.replaceChildren();
  for (const design of result.designs) {
    const row = document.createElement("li");
    const pick = document.createElement("button");
    pick.textContent = `${design.name} ${design.version || ""}`;
    pick.title = design.path;
    pick.addEventListener("click", () => {
      element("run-design").value = design.path;
    });
    row.appendChild(pick);
    if (design.errors.length > 0) {
      const errors = document.createElement("span");
      errors.className = "invalid";
      errors.textContent = design.errors.join("; ");
      row.appendChild(errors);
    }
    list.appendChild(row);
  }
}

async function runCase() {
  status("running");
  const result = await client.request({
    kind: "run_case",
    payload: {
      design: element("run-design").value,
      test_case: element("run-case").value,
      seed: Number.parseInt(element("run-seed").value, 10),
      mode: element("run-mode").value,
    },
  });
  addRun(result.archive, `${result.run_id} ${result.status}`);
  status(`run ${result.run_id}: ${result.status}`);
}

function addRun(archive, label) {
  if (!archive) {
    return;
  }
  const row = document.createElement("li");
  const pick = document.createElement("button");
  pick.textContent = label;
  pick.title = archive;
  pick.addEventListener("click", () => openReplay(archive).catch(failure));
  row.appendChild(pick);
  element("runs").appendChild(row);
}

// ===== replay ==========================================================

async function openReplay(archive) {
  stopPlaying();
  const handle = await client.request({ kind: "replay_open", payload: { archive } });
  const geometry = await client.request({ kind: "geometry", payload: { run_id: handle.run_id } });
  const marks = await client.request({ kind: "timeline", payload: { run_id: handle.run_id } });
  state.handle = handle;
  state.bodies = geometry.bodies.map((body) => body.body);
  state.fixed = new Set();
  state.windows = new Map();
  const undrawn = view.load(geometry.bodies);
  element("run-title").textContent =
    `${handle.run_id}: case ${handle.case_id}, design ${handle.design_name} ${handle.design_version}, ` +
    `seed ${handle.seed}, mode ${handle.controller_mode}, ended ${handle.end_reason} at ${handle.end_time} s`;
  element("undrawn").textContent = "";
  if (undrawn.length > 0) {
    element("undrawn").textContent = `no drawable shapes: ${undrawn.join(", ")}`;
  }
  const bodies = element("probe-names");
  bodies.replaceChildren();
  for (const name of state.bodies) {
    const option = document.createElement("option");
    option.value = name;
    bodies.appendChild(option);
  }
  const slider = element("scrub");
  slider.min = String(handle.t_first);
  slider.max = String(handle.t_last);
  timeline.show(marks.entries, handle.t_first, handle.t_last);
  probes.clearProbe();
  await cursorTo(handle.t_first);
  status(`opened ${handle.run_id}`);
}

async function poseWindow(body, t) {
  const handle = state.handle;
  let start = t;
  let end = t + (handle.t_last - handle.t_first) / WINDOW_PARTS;
  if (end > handle.t_last) {
    end = handle.t_last;
  }
  if (end <= start) {
    start = handle.t_first;
    end = handle.t_last;
  }
  if (end <= start) {
    end = start + 1;
  }
  const poses = await client.window(handle.run_id, poseSeries(body), start, end, WINDOW_POINTS);
  poses.start = start;
  poses.end = end;
  return poses;
}

async function placeBodies(t) {
  for (const body of state.bodies) {
    if (state.fixed.has(body)) {
      continue;
    }
    let poses = state.windows.get(body);
    if (poses === undefined || t < poses.start || t > poses.end) {
      try {
        poses = await poseWindow(body, t);
      } catch (error) {
        if (error instanceof FacadeError && error.code === "not_found") {
          state.fixed.add(body);
          continue;
        }
        throw error;
      }
      state.windows.set(body, poses);
    }
    const index = sampleAt(poses.times, t);
    if (index < 0) {
      continue;
    }
    const [x, y, z, roll, pitch, yaw] = poseSeries(body).map((name) => poses.series[name][index]);
    view.setPose(body, { x, y, z, roll, pitch, yaw });
  }
  view.render();
}

async function showCursor(cursor) {
  state.cursor = cursor;
  element("scrub").value = String(cursor.t);
  let mode = "playing";
  if (cursor.paused) {
    mode = "paused";
  }
  element("cursor").textContent = `t = ${cursor.t} s  sample ${cursor.index + 1} of ${cursor.samples}  ${mode}`;
  await placeBodies(cursor.t);
  probes.show(cursor.t);
  await readValue();
}

async function cursorTo(t) {
  if (state.handle === null) {
    return;
  }
  const cursor = await client.request({ kind: "scrub", payload: { run_id: state.handle.run_id, t } });
  await showCursor(cursor);
}

async function step(direction) {
  if (state.handle === null) {
    return;
  }
  stopPlaying();
  const cursor = await client.request({ kind: "step", payload: { run_id: state.handle.run_id, direction } });
  await showCursor(cursor);
}

async function setPaused(paused) {
  if (state.handle === null) {
    return;
  }
  const cursor = await client.request({ kind: "pause", payload: { run_id: state.handle.run_id, paused } });
  state.cursor = cursor;
}

function scrubFromSlider() {
  const handle = state.handle;
  if (handle === null) {
    return;
  }
  stopPlaying();
  cursorTo(Number(element("scrub").value)).catch(failure);
}

// Playback moves the cursor by wall-clock time scaled by the chosen rate; the shown time is the facade's sample.
async function play() {
  if (state.handle === null || state.playing) {
    return;
  }
  await setPaused(false);
  state.playing = true;
  state.clock = state.cursor.t;
  state.wall = performance.now();
  element("play").textContent = "Pause";
  requestAnimationFrame(tick);
}

function stopPlaying() {
  if (!state.playing) {
    return;
  }
  state.playing = false;
  element("play").textContent = "Play";
  setPaused(true).catch(failure);
}

async function tick(now) {
  if (!state.playing) {
    return;
  }
  if (!state.busy) {
    state.busy = true;
    const rate = Number(element("rate").value);
    state.clock += ((now - state.wall) / 1000) * rate;
    state.wall = now;
    try {
      if (state.clock >= state.handle.t_last) {
        await cursorTo(state.handle.t_last);
        stopPlaying();
      } else {
        await cursorTo(state.clock);
      }
    } catch (error) {
      stopPlaying();
      failure(error);
    }
    state.busy = false;
  }
  requestAnimationFrame(tick);
}

// ===== probes ==========================================================

async function openProbe() {
  if (state.handle === null) {
    return;
  }
  const probe = await client.request({
    kind: "probe",
    payload: { run_id: state.handle.run_id, probe: element("probe-kind").value, name: element("probe-name").value },
  });
  probes.set(probe);
  probes.show(state.cursor.t);
}

async function readValue() {
  const ref = element("value-ref").value;
  if (state.handle === null || ref === "" || state.cursor === null) {
    return;
  }
  try {
    const value = await client.request({
      kind: "value_at",
      payload: { run_id: state.handle.run_id, ref, t: state.cursor.t },
    });
    element("value").textContent = `${value.ref} = ${value.value} ${value.unit} at ${value.t} s`;
  } catch (error) {
    element("value").textContent = "";
    failure(error);
  }
}

// ===== wiring ==========================================================

client.onEvent((message) => {
  if (message.localError) {
    status(message.localError);
  } else if (message.kind === "run_progress") {
    status(`run ${message.payload.run_id}: t = ${message.payload.t} s`);
  } else if (message.kind === "run_ended") {
    status(`run ${message.payload.run_id} ended: ${message.payload.reason} at ${message.payload.t} s`);
  } else if (message.kind === "timeline_event") {
    status(`${message.payload.category}: ${message.payload.label}`);
  } else if (message.kind === "error") {
    failure(new FacadeError(message.payload));
  }
});

element("refresh").addEventListener("click", () => listDesigns().catch(failure));
element("run").addEventListener("click", () => runCase().catch(failure));
element("open").addEventListener("click", () => {
  const archive = element("archive").value;
  addRun(archive, archive);
  openReplay(archive).catch(failure);
});
element("scrub").addEventListener("input", scrubFromSlider);
element("back").addEventListener("click", () => step("backward").catch(failure));
element("forward").addEventListener("click", () => step("forward").catch(failure));
element("play").addEventListener("click", () => {
  if (state.playing) {
    stopPlaying();
  } else {
    play().catch(failure);
  }
});
element("probe-open").addEventListener("click", () => openProbe().catch(failure));
element("value-read").addEventListener("click", () => readValue().catch(failure));

client
  .connect(socketUrl())
  .then(() => {
    status("connected");
    return listDesigns();
  })
  .catch(failure);
