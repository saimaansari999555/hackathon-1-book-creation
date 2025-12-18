<!--
Sync Impact Report:
- Version change: 0.0.0 → 1.0.0
- List of modified principles:
  - PRINCIPLE_1_NAME → Spec-First Development
  - PRINCIPLE_2_NAME → Technical Accuracy and Verifiable Sources
  - PRINCIPLE_3_NAME → Modular, Reproducible, Production-Ready Architecture
  - PRINCIPLE_4_NAME → Clear Instructional Writing
  - PRINCIPLE_5_NAME → Book Standards
  - PRINCIPLE_6_NAME → RAG Chatbot Standards
- Added sections:
  - Spec-Kit Rules
  - Architecture Requirements
  - Constraints
  - Success Criteria
- Removed sections: None
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# AI-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development
Every feature, from a book chapter to a chatbot component, begins with a specification. This includes research, task breakdown, and defined acceptance criteria before any implementation is written. This ensures clarity, alignment, and verifiable progress.

### II. Technical Accuracy and Verifiable Sources
All technical content, code examples, and explanations must be accurate and backed by verifiable sources. Claims, APIs, and architectural patterns must be grounded in official documentation or established best practices, with no placeholder or hallucinated content.

### III. Modular, Reproducible, Production-Ready Architecture
The system architecture must be modular, with clearly defined components and interfaces. Every part of the project, from the book's build process to the chatbot's deployment, must be fully reproducible from the instructions in the repository. All components must be built to a production-ready standard.

### IV. Clear Instructional Writing
The primary audience for the book is Computer Science and AI students or practitioners. All written content must be clear, concise, and pedagogically sound. Explanations should build from first principles where appropriate and aim for deep understanding.

## Book Standards

- **Static Site Generator**: The book will be built using Docusaurus with a TypeScript configuration.
- **Structure**: Content must be organized into a logical structure of modules and weekly lessons.
- **Content Integrity**: No placeholder content (e.g., "lorem ipsum") or hallucinated/non-functional APIs are permitted in the final output.
- **Deployment**: A CI/CD pipeline will automatically deploy the book to GitHub Pages upon commits to the main branch.

## RAG Chatbot Standards

- **Backend Framework**: The RAG chatbot's backend will be implemented using FastAPI.
- **AI Integration**: The chatbot will use the OpenAI Agents or ChatKit SDKs for its core logic.
- **Data Stores**: Vector embeddings will be stored in a Qdrant Cloud instance, while structured metadata will be stored in a Neon Postgres database.
- **Functionality**: The chatbot must support two primary modes: querying the entire book (full-book QA) and querying a user-highlighted section of text (selected-text-only QA).
- **User Experience**: All responses from the chatbot must be streamed and include citations pointing to the source material in the book.

## Spec-Kit Rules

- **Location**: All specification documents must reside in the `specs/<feature>/` directory.
- **Required Artifacts**: Each feature specification must include at least `research.md` and `tasks.md`.
- **Task Granularity**: Tasks defined in `tasks.md` must be atomic, ordered, and verifiable with clear acceptance criteria.
- **Implementation Gate**: No implementation work may begin until the corresponding specification has been reviewed and approved.

## Architecture Requirements

- **Folder Structure**: The project must adhere to a defined folder structure for code, specs, and documentation.
- **API Contracts**: All APIs must have clearly defined contracts for requests and responses, including error states.
- **Data Schemas**: Vector and relational database schemas must be explicitly defined and version-controlled.
- **Data Pipelines**: The data ingestion and embedding pipeline must be documented and reproducible.
- **Integration Plan**: A clear plan for integrating the frontend (Docusaurus) with the backend (FastAPI chatbot) is required.
- **Deployment Strategy**: The project must include documented steps for both local development and production deployment.

## Constraints

- **Libraries**: Only stable, well-documented libraries and frameworks may be used.
- **Cost**: The project must operate within the free tiers of all external services (Qdrant, Neon, etc.).
- **Platform**: The entire project must support cross-platform development and deployment (Windows, macOS, Linux).
- **Reproducibility**: Builds must be fully reproducible. Any developer should be able to clone the repository and run the project using only the instructions in the `README.md`.

## Governance

This Constitution is the single source of truth for project principles and standards. It supersedes all other practices and ad-hoc decisions. All work carried out under this project must comply with this constitution.

- **Amendment Process**: Amendments to this constitution require a documented proposal, review, and approval from the project lead. An approved amendment must include a plan for migrating any existing code or documentation to the new standard.
- **Compliance**: All code reviews and specification approvals must include a check for compliance with this constitution. Any deviation must be explicitly justified and approved.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16