---
description: "Generate a Claude Code prompt for a coding task. Use when you want a prompt to copy into Claude Code for planning and implementation."
argument-hint: "Describe the goal and any high-level guidance for the change..."
agent: "ask"
---

Using the task description provided, generate a Claude Code prompt that includes:

1. **Goal**: A clear statement of what needs to be achieved
2. **High-level guidance**: Steps or hints on how to approach it — including reading and understanding relevant files before making any changes
3. **Constraints** (always include all of these):
   - Follow the existing coding standards and patterns in the codebase @.claude/code_standards.md and .claude/pyright_type_hints_reference.md
   - Start file discovery from .claude/repo_structure_and_script_map.md, then read only the minimum relevant files for the task
   - If any files are critical for safety/correctness, identify them explicitly as Must read before proposing changes
   - In the plan, include a concise Files to inspect list with one-line rationale per file
   - Prefer map-driven discovery over exhaustive hardcoded file lists unless broad review is explicitly required
   - Make minimal changes — do not refactor code that is not directly related to the task
   - Add minimal tests — only what is necessary to validate the change; fewer is better
   - Plan before implementing: read and understand relevant files first, then propose the approach and wait for confirmation before making changes
   - After implementing changes, delete any now-moot code that is directly made obsolete by the task (keep cleanup minimal and in-scope)

Output format requirements (strict):
- Return plain raw text only.
- Do not use Markdown formatting of any kind.
- Do not use headings, bullet markers, numbered Markdown lists, bold, italics, blockquotes, or tables.
- Do not wrap output in code fences.
- Do not use backticks anywhere.
- Use simple line breaks and plain paragraphs only.
- Assess if the task needs low, medium, or high effort and tell this to the user so they can decide which level of
effort to choose for the task in Claude Code.
- Start the first line with: Effort: <low|medium|high>
- Then provide sections in this exact plain-text label format:
  Goal:
  High-level guidance:
  Constraints:
  Must read before proposing changes:
  Files to inspect:
- If the assistant cannot comply with plain raw text, it must output exactly:
  RAW_TEXT_OUTPUT_FAILURE
