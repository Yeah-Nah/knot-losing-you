---
description: "Generate a Claude Code prompt for a coding task. Use when you want a prompt to copy into Claude Code for planning and implementation."
argument-hint: "Describe the goal and any high-level guidance for the change..."
agent: "ask"
---

Using the task description provided, generate a Claude Code prompt that includes:

1. **Goal**: A clear statement of what needs to be achieved
2. **High-level guidance**: Steps or hints on how to approach it — including reading and understanding relevant files before making any changes
3. **Constraints** (always include all of these):
   - Follow the existing coding standards and patterns in the codebase @.claude/code_standards.md and @.claude/pyright_type_hints_reference.md
   - Navigate from @.claude/repo_structure_and_script_map.md to only the minimum relevant files for the task rather than listing files manually
   - In the plan, include a concise Files to inspect list with one-line rationale per file; mark any safety- or correctness-critical files with (critical)
   - Make minimal changes — do not refactor code that is not directly related to the task
   - Add minimal tests — only what is necessary to validate the change; fewer is better. Run only the targeted test file, not the full test suite.
   - Plan before implementing: read and understand relevant files first, then propose the approach and wait for confirmation before making changes
   - After implementing changes, delete any now-moot code that is directly made obsolete by the task (keep cleanup minimal and in-scope)

Output format requirements (strict):
- Return plain raw text only.
- Do not use Markdown formatting of any kind.
- Do not use headings, bullet markers, numbered Markdown lists, bold, italics, blockquotes, or tables.
- Do not wrap output in code fences.
- File paths and symbol names may use backticks; do not use backticks anywhere else.
- Use simple line breaks and plain paragraphs only.
- Assess if the task needs low, medium, or high effort. Output this on the first line for the user's reference only — it is not part of the Claude Code prompt itself.
- Start the first line with: Effort: <low|medium|high>
- Then provide sections in this exact plain-text label format:
  Goal:
  High-level guidance:
  Constraints:
  Files to inspect:
- If the assistant cannot comply with plain raw text, it must output exactly:
  RAW_TEXT_OUTPUT_FAILURE

Formatting style:
```
Plain unformatted text.
No "`" surrounding path files.
Use - for bullet points in the Files to inspect list, but do not use any other Markdown formatting.
```
