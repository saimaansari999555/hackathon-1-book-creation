<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0
- List of modified principles:
  - PRINCIPLE_1_NAME → Technical accuracy from primary sources
  - PRINCIPLE_2_NAME → Clarity for AI/Robotics learners
  - PRINCIPLE_3_NAME → Reproducible systems and explanations
  - PRINCIPLE_4_NAME → No unsupported claims
- Added sections:
  - Standards
  - RAG requirements
  - Tech stack
  - Success criteria
- Removed sections:
  - Spec-Kit Rules
  - Architecture Requirements
  - Constraints
- Templates requiring updates:
  - ⚠ .specify/templates/plan-template.md
  - ⚠ .specify/templates/spec-template.md
  - ⚠ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Technical accuracy from primary sources
All technical content, code examples, and explanations must be accurate and backed by verifiable primary sources.

### II. Clarity for AI/Robotics learners
The primary audience is AI/Robotics learners. All written content must be clear, concise, and pedagogically sound.

### III. Reproducible systems and explanations
The system and explanations must be reproducible. Readers should be able to follow along and achieve the same results.

### IV. No unsupported claims
All claims must be supported by evidence from the specified sources.

## Standards

- APA citations (Markdown)
- Sources: peer-reviewed papers + official docs (ROS 2, NVIDIA Isaac, OpenAI)
- All content in .md using Docusaurus

## RAG requirements

- Chatbot answers only from book content
- Qdrant Cloud for vector retrieval
- Support “answer from selected text” mode
- No hallucinations outside retrieved context

## Tech stack

- Docusaurus (book)
- FastAPI (backend)
- OpenAI Agents/ChatKit SDKs
- Neon Serverless Postgres
- Qdrant Cloud (Free Tier)

## Success criteria

- Book builds & deploys successfully
- Embedded RAG chatbot answers accurately

## Governance

This Constitution is the single source of truth for project principles and standards. It supersedes all other practices and ad-hoc decisions. All work carried out under this project must comply with this constitution.

- **Amendment Process**: Amendments to this constitution require a documented proposal, review, and approval from the project lead. An approved amendment must include a plan for migrating any existing code or documentation to the new standard.
- **Compliance**: All code reviews and specification approvals must include a check for compliance with this constitution. Any deviation must be explicitly justified and approved.

**Version**: 2.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-23
