# Agent Instructions

## Agent skills

### Issue tracker

GitHub issues on `CongDjnhh/project_radar`. See `docs/agents/issue-tracker.md`.

### Triage labels

Canonical five-role vocabulary (`needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`). See `docs/agents/triage-labels.md`.

### Domain docs

Multi-context (`GLOSSARY-MAP.md` + per-context `GLOSSARY.md`). See `docs/agents/domain.md`.

## Workspace Rules for C Codebase

### Graphify First Navigation
Whenever exploring, reviewing, or locating symbols in the C codebase (`oob_1843_dss`, `oob_1843_mss`, `awr2x44p`):
1. **Always query the pre-built knowledge graph first** located at `graphify-out/graph.json` or `graphify-out/GRAPH_REPORT.md` before reading raw source files sequentially.
2. Locate the target symbols (functions, structs, variables, call hierarchy) in the graphify database.
3. Open and inspect only the specific target files and lines identified by graphify.
