# Domain Docs

How the engineering skills consume this repo's domain documentation when exploring the codebase.

## Before exploring, read these

- **`GLOSSARY-MAP.md`** at the repo root: points to the appropriate `GLOSSARY.md` for each project context (`oob_1843` vs `awr2944_hecr`). Read the one relevant to the current task.
- **`docs/adr/`**: read ADRs that touch the subsystem being modified. Check project-specific ADRs if present.

## File structure (Multi-context)

```
/
├── GLOSSARY-MAP.md
├── AGENTS.md
├── docs/
│   ├── agents/
│   │   ├── issue-tracker.md
│   │   ├── triage-labels.md
│   │   └── domain.md
│   └── adr/
├── docs/domains/
│   ├── oob_1843/
│   │   └── GLOSSARY.md
│   └── awr2944_hecr/
│       └── GLOSSARY.md
├── oob_1843_dss/
└── oob_1843_mss/
```
