# ADR 0007: Browser client over a local service, replaceable by a standalone app

Date: 2026-11-02
Status: Accepted
Closes rulings: R-16, R-17
Supersedes: Architecture doc, Technology recommendation, "Viewer and interface" paragraph (PySide6 shell, MuJoCo's
built-in viewer through milestone 3); Decisions log row 27 (desktop app, browser viewer only if easier)

## Context
The architecture doc chose a PySide6 desktop shell and MuJoCo's viewer for replay. The headless core (ADR 0006) and the
own engine (ADR 0001) remove both premises: the core exposes a facade, and MuJoCo is retired once the own engine is
default (ADR 0004). Engine development still needs a fast visual check that does not depend on MuJoCo.

## Decision

| Aspect | Decision |
|---|---|
| Early front end | Browser client over a local Python service, in `web/` |
| Later front end | Replaceable by a standalone app |
| Facade | A versioned protocol with a schema and a client-independent conformance suite |
| Client role | Clients hold no domain logic |
| Streaming and decimation | Live in the service, not the client |
| Protocol choice | By ADR in phase 1 |
| Engine debug viewer | Minimal immediate-mode C++ window for engine development, in `core/viewer_debug/`; replaces reliance on MuJoCo's built-in viewer |

## Consequences
- A second client (standalone app) must pass the same conformance suite; no client gets features the facade lacks.
- The service decides what reaches a client and at what rate, so client performance does not shape the core.
- The debug viewer is the only display code in `core/`; it is a development tool, not a product front end.
- PySide6 is no longer a dependency.

## Open items
- Facade protocol and transport: phase 1 ADR.
- When the standalone app replaces the browser client: not ruled.
