---
description: "Generate a Claude Code prompt for a coding task. Use when you want a prompt to copy into Claude Code for planning and implementation."
argument-hint: "Describe the goal and any high-level guidance for the change..."
agent: "ask"
---

Using the task description provided, generate a Claude Code prompt that includes:

1. **Goal**: A clear statement of what needs to be achieved
2. **High-level guidance**: Steps or hints on how to approach it — including reading and understanding relevant files before making any changes
3. **Constraints** (always include all of these):
   - Follow the existing coding standards and patterns in the codebase
   - Make minimal changes — do not refactor code that is not directly related to the task
   - Add minimal tests — only what is necessary to validate the change; fewer is better
   - Plan before implementing: read and understand relevant files first, then propose the approach and wait for confirmation before making changes

Output the prompt as a raw text block so it can be copied directly into Claude Code.
