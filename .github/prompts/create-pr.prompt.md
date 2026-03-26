---
description: "Generate a filled PR description and create the PR on GitHub"
agent: "agent"
tools: [get_changed_files, github-pull-request_activePullRequest, github/*]
---

Generate a pull request description for the current branch compared to `main`.

Fill in every section of the template at [pull_request_template.md](../pull_request_template.md) using the actual diff between this branch and `main`. Use [PROGRESS_UPDATES.md](../../PROGRESS_UPDATES.md) as supplementary context where relevant.

## Section guidance

**Summary**
- 1–2 sentences: what problem does this PR solve and what is the high-level approach?

**Changes**
- Group changes by category (e.g. new scripts, config changes, documentation, tests)
- Keep each bullet concise — a human should be able to skim the list in under a minute
- Call out any breaking changes or migrations explicitly
- Skip trivial/mechanical changes (formatting fixes, comment typos, etc.)

**How to Test**
- Keep the standard commands from the template
- Add any feature-specific steps that are not obvious (e.g. hardware setup, specific CLI flags, expected output to verify)

**Screenshots / Logs**
- Leave a placeholder if none are available; suggest what evidence would be useful for a reviewer

**Checklist**
- Check off (`[x]`) any items that clearly apply based on the diff
- Leave unchecked items that cannot be verified from the diff alone

**Notes for Reviewers**
- Call out non-obvious design decisions, trade-offs, or areas that warrant extra scrutiny
- Note any known limitations or planned follow-up work

Once the description is generated, create the pull request on Yeah-Nah/knot-losing-you from the current branch into main using the GitHub MCP, with a concise title derived from the Summary section and the generated text as the PR body.
