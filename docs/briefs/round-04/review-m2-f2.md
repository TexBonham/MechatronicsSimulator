# Review: M2-F2 (browser client, serve command)

Reviewer: Fable (general-purpose). Read-only; spec rows and diff only; never the writer's report; no builds, tests
or JavaScript execution (none is available; judge the JavaScript by reading it against the protocol contract);
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-04-f2`, branch `wip/04-f2`, commit bb25d8f, diff
`git -C <worktree> diff round/04-milestone-2...wip/04-f2`. Brief of record `docs/briefs/round-04/m2-f2.md`; rows
VIE-001, 012, 013, 015, 017 in `docs/spec/vie.md`, FAC-001, 005, 016, 019, 020 in `docs/spec/fac.md`; ADR 0007,
ADR 0015; ruling 21 in `docs/queue.md`; contracts `mechsim/facade/CLAUDE.md` (server, request kinds, binary frames).

Check:
1. The static route: serves only files under `web/`, refuses `..`, absolute and encoded traversal, 404 elsewhere,
   the WebSocket path unchanged, the token still required on the socket; `mechsim serve` prints one URL line and
   `--check` exits 0 without serving. Security posture per ADR 0015 (localhost, token).
2. `web/protocol.js` against the contract: envelope fields, version as the int, message ids, request and response
   matching, one terminal answer per id, `tick_batch` header then one binary frame, byte-length check,
   `Float64Array` columns in header order with time first (FAC-019), error payloads (FAC-020). Any kind the client
   sends or handles that the schema lacks.
3. `web/replay.js`, `timeline.js`, `probes.js`: every displayed number comes from a message (FAC-016); geometry by
   the sizes the `geometry` reply carries (VIE-015); scrub, pause and step requests drive the cursor through the
   facade, not a local clock; no file writes (VIE-017); no design names (invariant 2); plain ES modules, no CDN.
4. The browser-free tests: the protocol-kind scan is sound (both quoting forms, kinds the client handles as well as
   sends), the same-origin check, the vendored hash, the coarse FAC-016 screen excluding `web/vendor/`; VIE-001 and
   VIE-012 conversions honest under ruling 21; the manual checklist in `web/README.md` is complete enough for the owner
   to run once.
5. Ownership: `tests/unit/test_facade_service.py:86` (the CLI import set) was left failing for the coordinator; the
   `page_url` addition to `server.py`; anything else outside the list.
6. The writer notes the protocol has no `list_runs` request so the client lists only runs it started or a typed
   archive path: say whether that is a FAC-011 or VIE-001 gap for milestone 2 or an X2 item.

Report, under 350 words: verdict, findings by severity as `file:line, one sentence, spec row`, owner rulings
needed, cost note.
