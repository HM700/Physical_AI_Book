---
id: 1
title: create-project-constitution
stage: constitution
date: 2026-01-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: sp.constitution Physical_AI_Book
labels: [constitution,physical-ai,education]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 -
---

## Prompt

```
sp.constitution
Physical_AI_Book
1. Authority

This document is the root constitutional authority for the Physical AI Book project.
All future specifications, designs, frontend, backend, content, and infrastructure must derive authority from this document.

No lower-level spec may override this constitution.

2. System Definition

System Name: Physical_AI_Book
System Type: Spec-Driven Educational Knowledge System
Primary Artifact: Interactive technical book
Delivery Medium: Docusaurus documentation site
Deployment Target: GitHub Pages

3. Purpose

The purpose of this system is to teach Physical AI:
AI systems that perceive, reason, and act in the physical world, not just in digital environments.

The system bridges:

Digital intelligence (models, agents, planning)

Physical embodiment (robots, sensors, actuators)

Simulation and real-world execution

4. Target Audience

Beginners to Intermediate learners

AI/ML practitioners transitioning to robotics

Software engineers exploring embodied intelligence

Assumed Knowledge:

Basic Python

Introductory AI concepts

5. Educational Principles (Non-Negotiable)

Hands-On First
Learning is driven by building, simulating, and experimenting.

Embodiment Over Abstraction
Intelligence is validated only when it controls a physical or simulated body.

Simulation-Before-Reality
All systems are first developed and tested in simulation.

Spec-Driven Learning
Specifications define learning objectives before content or code exists.

6. Methodology Constraint

The project must follow Spec-Driven Development using Spec-Kit Plus principles.

This implies:

Specs precede implementation

Clear separation of concerns

Versioned evolution of knowledge

Explicit acceptance criteria for behavior

7. Technical Constraints
Documentation

Docusaurus is mandatory for all book content

System Architecture

The project must be logically separated into:

Frontend (documentation & UX)

Backend (intelligence & APIs)

Specs (source of truth)

History (decisions & evolution)

AI & Robotics Domain

The system operates in the domain of:

Physical AI

Embodied intelligence

Humanoid and robotic systems

8. Integrated Intelligence Requirement

The book must embed an AI assistant that:

Answers questions about the book's content

Uses Retrieval-Augmented Generation (RAG)

Can answer questions using only user-selected text

Operates as part of the learning experience

9. Explicit Non-Goals

This system does not aim to:

Be purely theoretical

Teach non-embodied AI only

Replace official robotics documentation

Focus on hardware electronics design

10. Success Definition

The system is successful if:

The book is published and navigable

Learners can build Physical AI systems

Knowledge progresses from specs to execution

Intelligence is demonstrated through embodied behavior

11. Final Authority Statement

This sp.constitution is intentionally minimal and stable.

All curriculum structure, modules, implementations, and sequencing must be defined later, in separate specs, step by step, and only after this constitution is accepted.
```

## Response snapshot

Created the Physical AI Book constitution with all specified principles and requirements. The constitution defines the core principles of hands-on learning, embodiment over abstraction, simulation-before-reality, and spec-driven development. It establishes the technical constraints around using Docusaurus for documentation, the system architecture, and the integrated AI assistant requirement. The document also outlines the target audience, success criteria, and governance structure.

## Outcome

- ✅ Impact: Created initial project constitution at .specify/memory/constitution.md
- 🧪 Tests: N/A for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md with project-specific constitution
- 🔁 Next prompts: Can proceed with creating spec for first feature/module
- 🧠 Reflection: Constitution captures all essential project constraints and principles

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating first module specification