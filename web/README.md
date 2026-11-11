# web/: browser client

Browser client over the local facade service (VIE-012, ADR 0007, ADR 0015 proposed). Holds no domain logic: every
value it shows comes from a facade message (FAC-016). Plain ES modules; no build step, no bundler, no npm, no CDN at
runtime. Writes no files (VIE-017) and names no design (invariant 2).

## Run it

```
uv run mechsim serve [--files-root PATH] [--runs PATH] [--port N]
```

Prints one line, `http://127.0.0.1:<port>/?token=<token>`; open it. The page and the WebSocket share the port: plain
GETs serve `/` (`index.html`), top-level `*.html`, `*.js`, `*.css` and `vendor/*`, anything else 404; the client reads
the token from its own URL and opens `/ws?token=<token>` on the same origin. Request paths (design, case, archive) are
relative to `--files-root` (default the repository root).

| File | Holds |
|---|---|
| `index.html`, `style.css` | Page and layout |
| `app.js` | Designs, runs (`run_case`; the list from `list_runs`, or an archive path), `replay_open`, cursor (`scrub`, `pause`, `step`), playback by `advance` (ruling 22), probe and `value_at` wiring; the pose drawn is always the cursor's own sample |
| `protocol.js` | `FacadeClient`: envelope, version 1, message ids, request/response matching, events, `decodeTickBatch` (header + one binary frame to `Float64Array` columns, FAC-019) |
| `replay.js` | three.js scene: bodies from `geometry` (box, sphere, cylinder, capsule, plane by sizes; convex hull as its vertices), posed from `pose:<body>.x y z roll pitch yaw` (Z-Y-X) over `subscribe_series` windows |
| `timeline.js` | Marks from `timeline` entries |
| `probes.js` | Probe panel from `probe` columns at the cursor |

## Vendored

| Library | Version | File | SHA-256 | Source |
|---|---|---|---|---|
| three.js (MIT) | 0.170.0 (r170) | `vendor/three.module.js` | `ce1fa418de16a19495a9f72495580e3015d7745c296d3ce0485897f902ddedfb` | npm `three@0.170.0`, `build/three.module.js` |

three.js 0.170.0 SHA-256 ce1fa418de16a19495a9f72495580e3015d7745c296d3ce0485897f902ddedfb

`tests/unit/test_web_client.py` checks the file against this hash. To upgrade, replace the file with another release's
`build/three.module.js` (self-contained up to r170; later releases split it into `three.core.js`) and update both
lines.

## Tests

Without a browser (ruling 21): `tests/unit/test_web_client.py` checks the kinds the client sends and handles against
`schemas/facade_protocol.json`, same-origin only, the vendored hash, a coarse domain-logic screen, no file writing, no
design names, playback by `advance` with no local simulated clock, the run list from `list_runs`, the drawn
pose taken only from the cursor's own sample, and the static route. A browser-driven test is deferred to M3.

## Manual checklist (owner, once at the gate)

- [ ] `uv run mechsim serve` prints the URL; opening it shows "connected" and the design list.
- [ ] Run P1 from the page (design `designs/pendulum/design.yaml`, case from `test_cases/pendulum/`, seed and mode
      as the case states), or enter an existing P1 archive (`runs/<run id>.h5`) and press Open replay; the bodies draw.
- [ ] The run list shows the archives under `--runs` and the open replays (marked open); a finished run appears in it.
- [ ] Scrub with the slider; the cursor time and the body poses follow; Play, Pause and the step buttons work.
- [ ] Play at each rate: the cursor time advances at about the rate times wall time, playback stops at the last sample,
      and pausing on any frame shows the pose of the sample the readout names.
- [ ] Timeline marks appear; clicking one moves the cursor to it.
- [ ] Probe a joint (kind joint, its name): position, velocity and force columns show with their units at the cursor.
- [ ] A wrong token in the URL leaves the page at an error, not connected.
