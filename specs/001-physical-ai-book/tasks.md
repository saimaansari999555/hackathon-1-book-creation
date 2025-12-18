# Tasks: Physical AI & Humanoid Robotics Technical Book

**Input**: Design documents from `/specs/001-physical-ai-book/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Tests**: No automated tests are specified for the initial content creation. Validation will be done through manual review and build checks.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Docusaurus site: `frontend/`
- Lab code: `labs/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the Docusaurus project and basic repository structure.

- [x] T001 Initialize a new Docusaurus site in the `frontend/` directory using the `npx create-docusaurus@latest` command.
- [x] T002 [P] Configure site metadata, title, and theme in `frontend/docusaurus.config.ts`.
- [x] T003 [P] Create the initial directory structure for labs: `labs/module-1-ros`, `labs/module-2-isaac`, `labs/module-3-vla`, `labs/capstone`.
- [x] T004 [P] Create a root `README.md` file explaining the project structure and setup instructions.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create the core content structure for the book before filling in the details.

- [x] T005 Create the book's module structure within `frontend/docs/` based on `data-model.md`: `01-Introduction`, `02-ROS-2-Fundamentals`, `03-Digital-Twins`, `04-NVIDIA-Isaac`, `05-VLA-Models`, `06-Capstone-Project`.
- [x] T006 [P] For each module directory in `frontend/docs/`, create placeholder `_category_.json` and `introduction.md` files.
- [x] T007 Configure the sidebar in `frontend/sidebars.js` to automatically generate navigation from the `frontend/docs/` directory structure.
- [x] T008 [P] Add a `Dockerfile` to the `labs/` directory to create a reproducible ROS 2 environment as outlined in `quickstart.md`.

---

## Phase 3: User Story 1 - AI Student Learns Embodied AI (Priority: P1) 🎯 MVP

**Goal**: Provide the essential first steps for a student to get started with ROS 2.

**Independent Test**: A user can follow the "Getting Started" guide, set up their environment, and run the first ROS 2 simulation.

- [x] T009 [US1] Write the content for the "Getting Started" guide in `frontend/docs/01-Introduction/getting-started.md`.
- [x] T010 [US1] Write the first lesson on ROS 2 nodes in `frontend/docs/02-ROS-2-Fundamentals/01-nodes.md`.
- [x] T011 [US1] Create the ROS 2 package for the first lab in `labs/module-1-ros/lab_1_pubsub`.
- [x] T012 [US1] Implement the Python publisher and subscriber nodes for the first lab in `labs/module-1-ros/lab_1_pubsub/lab_1_pubsub/`.
- [x] T013 [US1] Write the step-by-step lab guide in `frontend/docs/02-ROS-2-Fundamentals/02-lab-pubsub.md`, referencing the code in `labs/`.

---

## Phase 4: User Story 2 - Robotics Engineer Bridges Simulation and Reality (Priority: P2)

**Goal**: Enable a user to work with high-fidelity simulation in NVIDIA Isaac Sim.

**Independent Test**: A user can follow the guide to set up Isaac Sim and run a sample ROS 2 integration.

- [x] T014 [US2] Write the introductory content for the NVIDIA Isaac Sim module in `frontend/docs/04-NVIDIA-Isaac/01-introduction.md`.
- [x] T015 [US2] Create the ROS 2 package for the Isaac Sim lab in `labs/module-2-isaac/lab_1_isaac_integration`.
- [x] T016 [US2] Write the lab guide on importing a URDF and controlling it with ROS 2 in Isaac Sim, to be located in `frontend/docs/04-NVIDIA-Isaac/02-lab-isaac-ros.md`.

---

## Phase 5: User Story 3 - Educator Deploys a Reproducible Robotics Curriculum (Priority: P3)

**Goal**: Make the project easily deployable and usable for educators.

**Independent Test**: A user can clone the repository, run `npm install && npm start` in `frontend/`, and see the book site running locally. The GitHub Pages deployment action successfully builds and deploys the site.

- [x] T017 [US3] [P] Enhance the root `README.md` with detailed instructions for setup, development, and deployment.
- [x] T018 [US3] Configure `docusaurus.config.ts` for GitHub Pages deployment (setting `organizationName`, `projectName`, `deploymentBranch`).
- [x] T019 [US3] Create a GitHub Actions workflow file at `.github/workflows/deploy.yml` to automatically build and deploy the Docusaurus site to the `gh-pages` branch on push to `main`.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, review, and deployment.

- [ ] T020 [P] Review all written content for technical accuracy, clarity, and grammatical errors.
- [ ] T021 Validate that all lab code in the `labs/` directory is reproducible by following the instructions in the book.
- [x] T022 Run a final local build of the Docusaurus site using `npm run build` in `frontend/` to ensure there are no errors.
- [ ] T023 Merge the feature branch into `main` to trigger the first production deployment.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before any other phase.
- **Phase 2 (Foundational)** must be completed before the User Story phases.
- **User Story Phases (3, 4, 5)** can be worked on in parallel after Phase 2 is complete, but should be completed in priority order (US1 → US2 → US3) for a logical content progression.
- **Phase 6 (Polish)** is the final step before considering the feature complete.

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete all tasks in Phase 3: User Story 1.
4.  **STOP and VALIDATE**: At this point, the core value is delivered. A student can start learning and complete the first hands-on lab.
