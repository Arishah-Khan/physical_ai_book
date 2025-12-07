# Tasks: Docusaurus Introduction SPEC

**Input**: Design documents from `/specs/001-docusaurus-intro-spec/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Ensure `docusaurus-project/docs` directory exists
- [x] T002 Create empty `docusaurus-project/docs/intro.md` if it doesn't exist
- [x] T003 Create empty `docusaurus-project/docs/category.json` if it doesn't exist

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Add Docusaurus front-matter to `docusaurus-project/docs/intro.md`
- [x] T005 Add `sidebar_position` and `title` to `docusaurus-project/docs/intro.md` front-matter
- [x] T006 Structure `docusaurus-project/docs/category.json` for `intro.md`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Understand Course Overview (Priority: P1) 🎯 MVP

**Goal**: Students can quickly understand the course's purpose, scope, and learning outcomes.

**Independent Test**: A new reader can read `intro.md` and articulate the course's main theme and key learning outcomes.

### Implementation for User Story 1

- [x] T007 [US1] Add "Overview of Physical AI" section with 2-3 sentence summary to `docusaurus-project/docs/intro.md`
- [x] T008 [US1] Add "Importance of embodied intelligence" section with 2-3 sentence summary to `docusaurus-project/docs/intro.md`
- [x] T009 [US1] Add "Course focus & theme" section with 2-3 sentence summary to `docusaurus-project/docs/intro.md`
- [x] T010 [US1] Add "Quarter overview" section with 2-3 sentence summary to `docusaurus-project/docs/intro.md`
- [x] T011 [US1] Add "High-level module summary" section with 2-3 sentence summary to `docusaurus-project/docs/intro.md`
- [x] T012 [US1] Add "Learning outcomes" section with 2-3 sentence summary to `docusaurus-project/docs/intro.md`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Prepare for Course (Priority: P2)

**Goal**: Prospective students can identify practical hardware and software requirements.

**Independent Test**: A student can read `intro.md` and list the brief hardware and software requirements.

### Implementation for User Story 2

- [x] T013 [US2] Add "Hardware and software requirements (brief)" section with 2-3 sentence summary to `docusaurus-project/docs/intro.md`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T014 [P] Suggest recommended diagrams or images for `intro.md`
- [x] T015 [P] Define cross-module references within `intro.md`
- [x] T016 [P] Include glossary terms mentioned in `intro.md`
- [x] T017 [P] Include references or recommended reading for `intro.md`
- [x] T018 Add "Notes for future expansion" section to `docusaurus-project/docs/intro.md`
- [x] T019 Ensure overall `intro.md` and `category.json` adhere to professional technical-book standards

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- T014, T015, T016, T017 can run in parallel in Phase 5.
- Different user stories can be worked on in parallel by different team members once foundational tasks are complete.

---

## Parallel Example: Phase 5 (Polish)

```bash
# These tasks can be launched in parallel:
Task: "Suggest recommended diagrams or images for intro.md"
Task: "Define cross-module references within intro.md"
Task: "Include glossary terms mentioned in intro.md"
Task: "Include references or recommended reading for intro.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
