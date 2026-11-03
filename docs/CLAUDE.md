# docs/

| Path | Holds |
|---|---|
| `Mechatronic Simulator — Architecture and Technology Recommendation.md` | Founding architecture doc; Decisions log rows 1-44; invariants; layout; Risks |
| `Mechatronic Simulator — Requirements.md` | Gantry reference design, G1-G16, acceptance criteria, milestone plan |
| `adr/` | Architecture decision records; index in `adr/README.md` |
| `spec/` | One spec per module, numbered requirements (ADR 0008; written in phase 0b) |
| `rulings/` | Planning rulings, the handoff records for ADRs |
| `milestones.md` | Milestones 1-5, engine packages, MuJoCo retirement criteria, open values |
| `queue.md` | Dispatch queue and next brief |
| `cost-log.md` | Session cost log; archive in `cost-log-archive.md` |

Precedence: the Decisions log wins over the requirements doc; open items live in the architecture doc's Risks and open questions.

## ADR workflow

1. Copy `adr/0000-template.md` to the next free number with a short slug; fill every section.
2. Add a row to the table in `adr/README.md`.
3. Add a row to the architecture doc's Decisions log; mark "Changes doc" Yes where the ADR contradicts doc text.
4. Owner reviews every ADR (W-02).

Process rulings W-01 to W-07 are not ADRs; they stay in `rulings/`.

## Spec format (ID pattern and prefixes set in phase 0c; ADR 0008 governs the rest)

| Aspect | Rule |
|---|---|
| Location | `spec/<module>.md`, one per module |
| Requirements | Numbered, ID `^[A-Z]{2,5}-\d{3}$`, each traced to the source docs |
| Stubs | Every requirement has a test stub, strict expected failure, before code |
| Traceability | Test fails on a requirement without a test or a test citing an unknown ID; matrix regenerated in CI |
| Tolerances | Stated per test in the spec; changing one is a reviewed spec edit |
| Behavior changes | Cite requirement IDs; spec edit in the same commit |
| Review | Stage review rejects behavior not covered by the spec as edited |

Spec template layout: phase 0b.

## Writers

Only the coordinating session writes `cost-log.md` and `queue.md` (W-05, W-07). Agents, worktrees and other machines never do. No `CLAUDE.md` holds cost-log rows.

## Contracts

Empty until phase 1.
