---
id: 1
title: Docusaurus Homepage UI Spec
stage: spec
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: docusaurus-homepage-ui
branch: 1-docusaurus-homepage-ui
user: unknown
command: /sp.specify
labels: ["ui", "docusaurus", "spec"]
links:
  spec: specs/1-docusaurus-homepage-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-docusaurus-homepage-ui/spec.md
  - specs/1-docusaurus-homepage-ui/checklists/requirements.md
tests:
null
---

## Prompt

Title: Docusaurus Homepage UI Specification (Dark Agentic Theme)
Version: 1.0.0
Date: {{today}}

Objective:
Design the main homepage of the Docusaurus site with a bold, futuristic, dark theme that conveys intelligence, innovation, and agentic personality.

UI Requirements:
1️⃣ Hero Section:
- Dark gradient background (Deep Black → Emerald Green glow)
- Big Futuristic heading text: “Physical AI & Humanoid Robotics”
- Subtitle: “Future of Work — Agents + Humans + Robots”
- Primary CTA button: “Start Learning”
- Secondary CTA button: “AI Course Tutorial — 5 mins”
- Modern vector robot illustration on right side

2️⃣ Feature Section (3 cards side-by-side):
- Icon top (AI, Robot, Automation)
- Card titles:
  - "Learn Fast"
  - "Build Intelligent Systems"
  - "Future-Proof Skills"
- Hover animation: glow + scale
- Short crisp text focusing innovation

3️⃣ Footer:
- Darker navy base
- Sections:
  - Docs (Introduction, Robotics Basics)
  - Community (Discord, GitHub, LinkedIn)
  - About (Team, Contact)
- Copyright: “© 2025 Physical AI Project — Built with Docusaurus”

Brand Theme:
- Primary: #00FF88 (Neon Green)
- Secondary: #0A0F10 (Deep Black Navy)
- Accent: #FFFFFF + #02D2F2 (Cyan Glow)
- Icons: Minimalistic vector style

Typography:
- Heading: Bold futuristic display font
- Body: Clean sans-serif (inter or similar)
- Increased letter spacing for headings

Animations:
- Smooth fade-in + micro interactions
- Scroll-triggered reveal animations

Accessibility:
- Contrast AAA standards
- Keyboard navigable
- Alt text for illustrations

Deliverables:
- Docusaurus homepage layout code
- CSS theme customization + global styling

## Response snapshot

The `/sp.specify` command was executed to create a specification for the Docusaurus Homepage UI.
A new branch `1-docusaurus-homepage-ui` was created.
The directory `specs/1-docusaurus-homepage-ui` was created.
The `specs/1-docusaurus-homepage-ui/spec.md` file was generated based on the user's input, outlining user scenarios, functional requirements, and success criteria.
A quality checklist `specs/1-docusaurus-homepage-ui/checklists/requirements.md` was also created and validated, with all items passing.
The specification is now ready for the next phase.

## Outcome

- ✅ Impact: Specification for Docusaurus homepage UI created and validated.
- 🧪 Tests: N/A
- 📁 Files: Two files created: `specs/1-docusaurus-homepage-ui/spec.md` and `specs/1-docusaurus-homepage-ui/checklists/requirements.md`.
- 🔁 Next prompts: User can now run `/sp.plan` or `/sp.clarify`.
- 🧠 Reflection: Manual PHR creation due to `pwsh` script failure.

## Evaluation notes (flywheel)

- Failure modes observed: pwsh script not found, requiring manual branch, directory, spec, and checklist creation.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
