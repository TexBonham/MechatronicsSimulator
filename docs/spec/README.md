# Specs

One spec per module, with numbered requirements traced to the source docs (ADR 0008). Every spec starts from
[`0000-template.md`](0000-template.md), which also states how requirement IDs are allocated and withdrawn.

## Planned specs

Written in phase 0b by three briefs: S1 the compiled side, S2 the data side, S3 the run side.

| Prefix | Module | File | Brief |
|---|---|---|---|
| ENG | Engine | `eng.md` | S1 compiled side |
| KER | Kernel | `ker.md` | S1 compiled side |
| SIG | Signals | `sig.md` | S1 compiled side |
| CTL | Controller | `ctl.md` | S1 compiled side |
| FW | Firmware and board interface | `fw.md` | S1 compiled side |
| LIB | Library | `lib.md` | S2 data side |
| MAT | Materials | `mat.md` | S2 data side |
| DES | Design | `des.md` | S2 data side |
| BLD | Builder | `bld.md` | S2 data side |
| PHY | Physics adapters | `phy.md` | S2 data side |
| TST | Testing | `tst.md` | S3 run side |
| REC | Record | `rec.md` | S3 run side |
| INT | Integrity | `int.md` | S3 run side |
| SCO | Scoring | `sco.md` | S3 run side |
| FAC | Facade | `fac.md` | S3 run side |
| VIE | Viewer | `vie.md` | S3 run side |
| INV | Invariants | `inv.md` | S3 run side |
| ACC | Acceptance | `acc.md` | S3 run side |

## Traceability

[`tools/traceability.py`](../../tools/traceability.py) reads the `## Requirements` table of every spec here (all
`*.md` except `0000-template.md`, this README and `traceability.md`), the `req` markers of the Python tests under
`tests/` and the requirement tags of the Catch2 tests under `core/`. It fails on:

- a requirement with no citing test (withdrawn requirements are exempt);
- a test citing an ID no spec defines;
- an ID defined twice;
- a spec table ID, `req` marker or Catch2 tag that looks like an ID but fails `^[A-Z]{2,5}-\d{3}$`;
- a `## Requirements` heading with no table rows, or a table whose header differs from the template;
- a test file that does not parse.

It also generates [`traceability.md`](traceability.md), one table per prefix plus a summary, and fails when the
committed copy is stale.

```sh
python3 tools/traceability.py           # check; exit 1 on violations or a stale matrix
python3 tools/traceability.py --write   # regenerate traceability.md; exit 1 on violations
```

`tests/invariant/test_traceability.py` runs the check over the repository. Stub conventions: `tests/README.md`.
