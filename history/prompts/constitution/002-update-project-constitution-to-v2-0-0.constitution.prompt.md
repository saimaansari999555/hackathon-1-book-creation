---
id: 002
title: Update Project Constitution to v2.0.0
stage: constitution
date: 2025-12-23
surface: agent
model: gemini-pro
feature: none
branch: 001-physical-ai-book
user: waizhussain9955
command: /sp.constitution
labels: ["constitution", "refactor"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:
  null
---

## Prompt

/sp.constitution

Project: Physical AI & Humanoid Robotics Book with Embedded RAG Chatbot

Core principles:
- Technical accuracy from primary sources
- Clarity for AI/Robotics learners
- Reproducible systems and explanations
- No unsupported claims

Standards:
- APA citations (Markdown)
- Sources: peer-reviewed papers + official docs (ROS 2, NVIDIA Isaac, OpenAI)
- All content in .md using Docusaurus

RAG requirements:
- Chatbot answers only from book content
- Qdrant Cloud for vector retrieval
- Support “answer from selected text” mode
- No hallucinations outside retrieved context

Tech stack:
- Docusaurus (book)
- FastAPI (backend)
- OpenAI Agents/ChatKit SDKs
- Neon Serverless Postgres
- Qdrant Cloud (Free Tier)

Success criteria:
- Book builds & deploys successfully
- Embedded RAG chatbot answers accurately

## Response snapshot

I have updated the project constitution to version 2.0.0 based on your input. This involved a major overhaul of the principles, standards, and requirements. I also updated the `.specify/templates/plan-template.md` to ensure its "Constitution Check" aligns with the new principles.

## Outcome

- ✅ Impact: High - redefined core project principles.
- 🧪 Tests: n/a
- 📁 Files: 2 files updated.
- 🔁 Next prompts: null
- 🧠 Reflection: The constitution update required propagating changes to dependent templates.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
