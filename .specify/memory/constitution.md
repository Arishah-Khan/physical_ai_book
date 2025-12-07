<!--
Sync Impact Report:
- Version change: N/A (Initial creation/full overwrite) -> 1.0.0
- List of modified principles: All principles are new based on user input.
- Added sections: All sections are new based on user input.
- Removed sections: N/A
- Templates requiring updates:
    - .specify/templates/plan-template.md: ⚠ pending
    - .specify/templates/spec-template.md: ⚠ pending
    - .specify/templates/tasks-template.md: ⚠ pending
    - .specify/templates/commands/sp.constitution.md: ⚠ pending
    - .specify/templates/commands/sp.phr.md: ⚠ pending
    - .specify/templates/commands/sp.specify.md: ⚠ pending
    - .specify/templates/commands/sp.plan.md: ⚠ pending
    - .specify/templates/commands/sp.tasks.md: ⚠ pending
    - .specify/templates/commands/sp.implement.md: ⚠ pending
    - .specify/templates/commands/sp.git.commit_pr.md: ⚠ pending
    - .specify/templates/commands/sp.clarify.md: ⚠ pending
    - .specify/templates/commands/sp.checklist.md: ⚠ pending
    - .specify/templates/commands/sp.analyze.md: ⚠ pending
    - .specify/templates/commands/sp.adr.md: ⚠ pending
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

Purpose:
This Constitution defines the core rules, standards, and principles for creating an AI-native technical textbook on Physical AI & Humanoid Robotics using Docusaurus, Claude Code, and Spec-Kit Plus. The book must integrate RAG chatbot technology, user personalization, and Urdu translation features, aligned with Panaversity’s strategic educational goals.

## Core Principles

### 1️⃣ Education Excellence & Accuracy
Ensure technical correctness and real-world relevance. All content must follow official documentation of ROS 2, Gazebo, Unity, NVIDIA Isaac, and humanoid robotics standards.

### 2️⃣ AI-Native Experience
The book must use AI agents, RAG search, and interactive learning activities. Students can ask questions from selected text and receive AI-generated assistance.

### 3️⃣ LIBRARY-FIRST Approach
Each component should be modular, reusable, and independently testable.
The Goal → Reusable Intelligence (Sub-agents / Agent Skills).

### 4️⃣ TDD (Test-Driven Documentation)
All chapters must include quizzes, exercises, labs, and validation steps before marking content “Complete”.

### 5️⃣ Personalization & Localization
Content must adapt to user's background and support Urdu translation with a user trigger in each chapter.

### 6️⃣ Open & Versioned Development
Versioning rules: MAJOR.MINOR.PATCH
→ Breaking changes → Major bump
→ New features → Minor
→ Fixes → Patch
All changes must be documented in CHANGELOG.

### 7️⃣ Deployment & Accessibility
The book must be published publicly through GitHub Pages or Vercel, ensuring open access and mobile-friendly UX.

## Section 2: Technical & Security Constraints

- Stack: Docusaurus + FastAPI + Neon Postgres + Qdrant Cloud + Better-Auth + OpenAI Agents
- RAG chatbot must strictly answer from book content
- Data privacy: Store only necessary user data for personalization
- Secure code & dependency management enforced

## Section 3: Workflow & Quality Gates

- Every change must improve clarity, correctness, or interactivity
- AI-generated content must be developer-reviewed before publish
- GitHub PR must verify:
  ✓ No hallucinations
  ✓ Proper citations & references
  ✓ Exercises & assessments included
  ✓ Compliance with Constitution

## Governance

- Constitution overrides all other guidelines
- Amendments require:
  → Documentation of change
  → Approval via commit review
  → Version bump + migration plan
- Violations require immediate correction before merge

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
