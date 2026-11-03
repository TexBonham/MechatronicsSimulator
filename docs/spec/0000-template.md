<!--
Spec template (ADR 0008). Copy to docs/spec/<prefix lowercase>.md and replace every <placeholder>.

Requirement IDs:
- Pattern ^[A-Z]{2,5}-\d{3}$, using this spec's prefix from the root CLAUDE.md (Test conventions).
- Allocated sequentially within the prefix: the next ID is one above the highest ever used in this file.
- Never reused and never renumbered, so a test citation keeps its meaning.
- A withdrawn requirement keeps its row; its Requirement text becomes "Withdrawn: <reason>". Withdrawn rows need no test.

python3 tools/traceability.py reads only the ## Requirements table; keep its header exactly as below and run the tool
with --write after any change to regenerate docs/spec/traceability.md.
-->

# Spec: <module> (<PREFIX>)

## Purpose

<Two or three sentences: what this module is responsible for and what it deliberately leaves to others.>

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| <OTHER PREFIX> | <PREFIX-NNN, ...> | <OTHER-NNN, ...> |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| <PREFIX>-001 | The <module> shall <behavior>. | <document, section> | <M1> | <unit> | <n/a> |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| <parameter> | not set | <who> |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| <item> | <PREFIX or module that owns it> |
