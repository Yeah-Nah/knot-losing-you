---
agent: agent
description: Auto-update the project progress blog by inserting one new entry based on milestones since the last logged update, with stronger emphasis on changes since the latest merged PR. Optional focus can be provided but is not required.
---

You maintain the progress blog in PROGRESS_UPDATES.md.

Objective:
Insert exactly one new progress entry directly into PROGRESS_UPDATES.md, using only milestones that happened after the most recent existing entry in that file.

Inputs:
- OPTIONAL_FOCUS: optional short direction for emphasis. May be empty.
- DATE_OVERRIDE: optional date string. If empty, use today.

Strict scope rules:
1. Identify the last logged entry in [PROGRESS_UPDATES.md](http://_vscodecontentref_/0) and extract:
- Last entry number
- Last entry date
- Current phase section
2. Build milestone candidate set from repository evidence only, constrained to work after the last entry date.
3. Apply moderate-high weighting to changes since the latest merged PR into the current branch:
- Prefer evidence from commit history and changed files after that merge point
- If a reliable merged-PR boundary cannot be determined, fall back to commits after the last entry date and state that fallback briefly
4. Exclude milestones already described in older blog entries.
5. Never invent facts.

Evidence priority:
1. Recent commits and changed files
2. Tests added or updated
3. Config and pipeline/control changes
4. Documentation updates that reflect concrete implementation progress

Writing requirements:
- Match existing tone and format in PROGRESS_UPDATES.md: personal, technical, milestone-focused
- Keep concrete details: scripts, modules, configs, tests, outcomes
- Keep claims verifiable from repo context
- Do not rewrite previous entries unless numbering must be incremented for the new insertion

Optional focus behaviour:
- If OPTIONAL_FOCUS is provided, prioritize relevant milestones first
- If OPTIONAL_FOCUS is empty, proceed automatically with no follow-up questions

Entry structure:
- Header: Entry N: Title
- Date line
- Intro paragraph
- What was done
- Key Achievements
- Optional image block only if a clearly relevant image exists in repo and is tied to this milestone set

Insertion behaviour:
- Insert into the correct phase section in [PROGRESS_UPDATES.md](http://_vscodecontentref_/1)
- Keep newest-first ordering within that phase
- Preserve separators, spacing, headings, and markdown style already used in the file

Execution steps:
1. Read [PROGRESS_UPDATES.md](http://_vscodecontentref_/2) for numbering, phase placement, style, and last entry date
2. Discover milestone evidence in scope (strictly after last entry date, weighted toward since latest merged PR)
3. Draft one new entry
4. Insert entry directly into [PROGRESS_UPDATES.md](http://_vscodecontentref_/3)
5. Return a short status summary only:
- Entry number used
- Phase used
- Date used
- Whether merged-PR boundary was used or fallback was used
- One short paragraph describing what was inserted

Output constraints:
- Do the file edit directly
- Do not output a full file dump unless explicitly asked
- If there is insufficient new evidence for a meaningful entry, do not insert anything and report no-op with reason
