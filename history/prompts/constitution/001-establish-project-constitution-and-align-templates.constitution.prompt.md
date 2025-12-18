---
id: 001
title: Establish Project Constitution and Align Templates
stage: constitution
date: 2025-12-16
surface: agent
model: Gemini
feature: none
branch: main
user: {{USER}}
command: /sp.constitution
labels: [constitution, template, planning]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - none
---

## Prompt

/sp.constitution /sp.constitution

Project:
AI-Driven Technical Book with Embedded RAG Chatbot
(Physical AI & Humanoid Robotics)

Objective:
Create and deploy a Docusaurus-based technical book using Spec-Kit Plus
and Claude Code, published on GitHub Pages, with a production-grade
RAG chatbot embedded in the site.

Core Principles:
- Spec-first development (research → tasks → implementation)
- Technical accuracy and verifiable sources
- Modular, reproducible, production-ready architecture
- Clear instructional writing for CS/AI audience

Book Standards:
- Docusaurus (TypeScript config)
- Module + weekly structure
- No placeholder content or hallucinated APIs
- CI/CD deployment to GitHub Pages

RAG Chatbot Standards:
- FastAPI backend
- OpenAI Agents / ChatKit SDKs
- Qdrant Cloud (vectors) + Neon Postgres (metadata)
- Supports full-book QA and selected-text-only QA
- Streaming responses with citations

Spec-Kit Rules:
- Specs in `specs/<feature>/`
- Required: `research.md`, `tasks.md`
- Tasks must be atomic, ordered, and verifiable
- No implementation without approved specs

Architecture Requirements:
- Defined folder structure
- API contracts
- Vector + relational schemas
- Ingestion and embedding pipeline
- Frontend integration plan
- Local and production deployment steps

Constraints:
- Stable, documented libraries only
- Free tiers only
- Cross-platform support
- Fully reproducible builds

Success Criteria:
- Book builds and deploys correctly
- RAG chatbot answers accurately with citations
- Selected-text QA works
- All tasks completed and documented
- New developer can run project from README alone

## Response snapshot

Successfully created the project constitution, updated dependent templates, and recorded the action in a Prompt History Record.

## Outcome

- ✅ Impact: Major. Established the foundational governance document for the project and aligned planning templates.
- 🧪 Tests: n/a
- 📁 Files: 2 files modified (`.specify/memory/constitution.md`, `.specify/templates/plan-template.md`) and 1 file created (`history/prompts/constitution/001-establish-project-constitution-and-align-templates.constitution.prompt.md`).
- 🔁 Next prompts: User may proceed with feature specification (`/sp.specify`) or planning (`/sp.plan`).
- 🧠 Reflection: The process of creating the constitution and immediately propagating the changes to the planning template is a good example of keeping governance and process in sync.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
